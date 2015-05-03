/*
Initialize_Sensors_test
Fernando Aragon
4/27/15 - Created

Starts all sensors and checks correct operation of all systems.
*/

#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <TinyGPS.h>
#include "MPL3115A2.h"
#include "I2Cdev.h"
#include "MPU6050.h"

//Initialize some variables
File myFile;
int cardCSPin = 53;
unsigned long chars;
unsigned short sentences, failed;
int xtendPin = 36;
String testString = "";
float pressure, intTemp;    // used for weather shield
float voltVideoCam, currVideoCam, voltArdu, currArdu, voltMot1, currMot1, voltMot2, currMot2;
const int videoCamPin = A1;
const int ArduPin = A2;
const int mot1Pin = 28;
const int mot2Pin = 29;
const int resistor = 10;          // Shunt resistor value (in ohms) NEEDS TO BE CHANGED DEPENDING ON EXPECTED CURRENT
const int voltageRef = 5;  // Reference voltage for analog read
int trigPin = 3;
int echoPin = 2;
float factor = 148.0; //  58.0 for centimeters and 148.0 for inches (sound sensor)
float flat, flon;
unsigned long age;
unsigned long date;
unsigned long time;
long duration, distance;
//int checkLight = 13;



// Initialize all program flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeGFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)
int timeout = 2;

void setup() {
  Serial.begin(9600);
  Wire.begin();    // Join i2c bus
  digitalWrite( cardCSPin, OUTPUT);
  pinMode(xtendPin, OUTPUT);
  initialize_sensors();
}

void loop() {}

/*
INITIALIZE_SENSORS turns on and check all systems.
Tasks:  Initialize and check SD Card and check operation
        Create GPS instance
        Turn on/off Xtend and check message transmission/reception
        Read current sensors (module voltages)
        Initialize weather shield
        Initialize IMU and get some data
        Check ultrasound sensors
        Check motor operation
*/
void initialize_sensors() {
  bool testComplete = false;    // complete Xtend test variable
  // Check SD card is working correctly
  Serial.println("---------Starting SD Card Testing---------");
  if (!SD.begin(cardCSPin)) { 
    Serial.println("Card failed, or not present!");
    Serial.println("Exiting...\n");
    return;
  }
  Serial.println("Card initialized...");
  
  // Check Xtend operation
  Serial.println("---------Starting Xtend Testing---------");
  digitalWrite(xtendPin, HIGH);
  Serial.println ("Enter message to be sent to ground computer.");     // signal initalization done
  while(Serial.available() == 0){}
  while (Serial.available())
    testString += Serial.readString();
  Serial.print('Message sent: ');
  Serial.println(testString);
  Serial3.print(testString);
  Serial.println();
  Serial.println ("Awaiting response...");     // signal initalization done
  while(Serial3.available() == 0){}
  while (Serial3.available())
    testString += Serial.read();
  Serial.print('Message received: ');
  Serial.println(testString);
  Serial.println();
  Serial.println("Continue testing? Y/N");
  while (!testComplete) {
    while(Serial.available() == 0){}
    if (Serial.read() == 'n') {
      Serial.println("Xtend module did not initialize correctly.");
      Serial.println("Exiting...\n");
      return;
    }
    else if (Serial.read() == 'y') {
      Serial.println("Xtend module initialized correctly.");
      testComplete = true;
    }
    else {
      Serial.println("Continue testing? Y/N");
    }
  }
  
  // Instantiate Weather Shield
  testComplete = false;
  Serial.println("---------Starting WeatherShield Testing---------");
  int startTime = millis();
  MPL3115A2 myWeather;
  myWeather.begin(); // Get sensor online
  myWeather.setModeBarometer();
  myWeather.setOversampleRate(7); // Set Oversample to the recommended 128
  myWeather.enableEventFlags(); // Enable all three pressure and temp event flags
  pressure = myWeather.readPressure();
  while (pressure == -1) {
    if (millis()-startTime > 5000) {
      Serial.println("Weather shield initialization timed out.");
      Serial.println("Exiting...\n");
      return;
    }
  Serial.println("Weather Shield initialized correctly.");
  }
  
  // Checking all voltages
  testComplete = false;
  Serial.println("---------Reading bus voltages---------");
  voltVideoCam = (analogRead(videoCamPin) * voltageRef) / 1023;
  currVideoCam = voltVideoCam / (10 * resistor);
  Serial.print("Video Cam voltage: ");
  Serial.print(voltVideoCam);
  Serial.print(" V\t\t Video Cam current:");
  Serial.print(currVideoCam);
  Serial.println(" A");
  voltArdu = (analogRead(ArduPin) * voltageRef) / 1023;
  currArdu = voltArdu / (10 * resistor);
  Serial.print("Arduino bus voltage: ");
  Serial.print(voltArdu);
  Serial.print(" V\t\t Arduino bus current:");
  Serial.print(currArdu);
  Serial.println(" A");
  voltMot1 = (analogRead(mot1Pin) * voltageRef) / 1023;
  currMot1 = voltMot1 / (10 * resistor);
  Serial.print("Motor 1 voltage: ");
  Serial.print(voltMot1);
  Serial.print(" V\t\t Motor 1 current:");
  Serial.print(currMot1);
  Serial.println(" A");
  voltMot2 = (analogRead(mot2Pin) * voltageRef) / 1023;
  currMot2 = voltMot2 / (10 * resistor);
  Serial.print("Motor 2 voltage: ");
  Serial.print(voltMot2);
  Serial.print(" V\t\t Motor 2 current:");
  Serial.print(currMot2);
  Serial.println(" A");
  Serial.println("Continue testing? Y/N");
  while (!testComplete) {
    while(Serial.available() == 0){}
    if (Serial.read() == 'n') {
      Serial.println("Current sensors not working correctly.");
      Serial.println("Exiting...\n");
      return;
    }
    else if (Serial.read() == 'y') {
      Serial.println("Current sensors working correctly.");
      testComplete = true;
    }
    else {
      Serial.println("Continue testing? Y/N");
    }
  }
  
  //Instantiate IMU
    Serial.println("---------Starting IMU Testing---------");
    testComplete = false;
  MPU6050 myIMU;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  Serial.println(myIMU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  startTime = millis();
  while (millis()-startTime < 5000) {
    Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
  }
  Serial.println("Continue testing? Y/N");
  while (!testComplete) {
    while(Serial.available() == 0){}
    if (Serial.read() == 'n') {
      Serial.println("Imu did not initialize correctly.");
      Serial.println("Exiting...\n");
      return;
    }
    else if (Serial.read() == 'y') {
      Serial.println("Imu initialized correctly.");
      testComplete = true;
    }
    else {
      Serial.println("Continue testing? Y/N");
    }
  }
  
  
  // Check sound sensors
  Serial.println("\n---------Ultra Sound Sensor Testing---------");
  testComplete = false;
  
  while (!testComplete) {
    startTime = millis();
    while (millis()-startTime < 4000) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration / factor;
      Serial.print("Distance from sensor: ");
      Serial.print(distance);
      Serial.println(" in.");
    }
    Serial.println("Continue testing? Y/N");
    while(Serial.available() == 0){}
    if (Serial.read() == 'n') {
      Serial.println("Current sensors not working correctly.");
      Serial.println("Exiting...\n");
      return;
    }
    else if (Serial.read() == 'y') {
      Serial.println("Ultra sound sensor working correctly.");
      testComplete = true;
    }
  }
  
  
  // Testing motors
  Serial.println("\n---------Stating motors testing---------");
  Serial.println("Running right motor for 3 sec");
  digitalWrite(mot1Pin, HIGH);
  delay(3000);
  digitalWrite(mot1Pin, LOW);
  Serial.println("Running left motor for 3 sec");
  digitalWrite(mot2Pin, HIGH);
  delay(3000);
  digitalWrite(mot2Pin, LOW);  
  Serial.println("Running both motors for 3 sec");
  digitalWrite(mot1Pin, HIGH);
  digitalWrite(mot2Pin, HIGH);
  delay(3000);
  digitalWrite(mot1Pin, LOW);
  digitalWrite(mot2Pin, LOW);
  Serial.println("Continue testing? Y/N");
    while(Serial.available() == 0){}
    if (Serial.read() == 'n') {
      Serial.println("Current sensors not working correctly.");
      Serial.println("Exiting...\n");
      return;
    }
 
  // Instantiate GPS
  Serial.println("\n---------Starting GPS---------");
  testComplete = false;
  TinyGPS gps;
  float flat, flon;
  unsigned long age;
  char incomingData;
  gps.stats(&chars, &sentences, &failed);    //check status of GPS
  if (chars == 0)
    Serial.println("** No characters received from GPS **");
    Serial.println("Not doing anything else.");
    return;
  Serial.println("Displaying GPS data. Enter any key to stop (when GPS has fix).\n");
  while(!testComplete) {
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(" AGE=");
    Serial.print(age);
    Serial.print(" LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    if (Serial.available()) {
      Serial.flush();
      testComplete = true;
    }
  }
  Serial.println("Continue testing? Y/N");
    while(Serial.available() == 0){};
    if (Serial.read() == 'n') {
      Serial.println("Current sensors not working correctly.");
      Serial.println("Exiting...\n");
      return;
    }
  
}
