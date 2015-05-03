/*
UHABS-3 testor testing
Author: Fernando Aragon
History:
3.17.15 - Created

Description:
  Program to test sensor operation for ME-419 class.
  Sensors:
    GPS, Weathershield, IMU, Magnetometer, Arducam, SDshield,
  Objective:
    Read data from sensors
    Get int/ext temperature
    Get air pressure
    Calculate air density
    Get GPS time, position(latitude, longitude, altitude), velocity, attitude(roll, pitch, yaw)
*/

#include<I2Cdev.h>
#include<MPU6050_6Axis_MotionApps20.h>
#include<Wire.h>
#include <MPL3115A2.h>
#include <TinyGPS.h>

//Create instances of all necessary sensors
MPL3115A2 myPressure;
TinyGPS gps;

//*********************** DEBUGGING VARS *********************
// testing sensor variable
int tempPin = A0;   // temp36 sensor pin
int powerTxPin = 36;  // shutdown pin from Xtend module
//int gLED = 24;  // green light LED ( XTEND power/TX)  DEBUGGING
//int rLED = 22;  // red light LED (TXTEND RX  DEBUGGING

// A - all sensors
// W - weather shield
// T - temperature sensor
// X - xtend transmitter
// C - SD card board
// G - GPDS
char TESTING_SENSOR = 'W';
//*********************** DEBUGGING VARS *********************


void setup() {
  // Initialize communications
  Wire.begin();          // Join i2c bus
  Serial.begin(9600);    // Start serial comm.
  Serial3.begin(115200);  // Start Xtend serial bus
  //Serial1.begin(9600);  // Start GPS serial bus
  
  
  // INITIALIZE WEATHER SHIELD
  myPressure.begin();    // Get weather shield online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags
  
  // INITIALIZE XTEND PIN and testing LEDs
  pinMode(powerTxPin, OUTPUT);
  digitalWrite(powerTxPin, HIGH);
  //pinMode(rLED, OUTPUT);
  //pinMode(gLED, OUTPUT);
  //digitalWrite(gLED, LOW);
}

void loop() {
  switch(TESTING_SENSOR) {  //separate every sensor to do individual testing
    case 'W':
      Serial.println("\n In case WeatherShield\n ");
      test_weatherShield();
      break;
    case 'T':
      Serial.println("\n In case Temp36\n ");
      test_temperature();
      break;
    case 'X':
      //Serial.println("\n In case Xtend\n ");
      test_transmitter();
      break;
    case 'C':
      //Serial.println("\n In case SDCard\n ");
      test_SD();
      break;
    case 'G':
      //Serial.println("\n In case SDCard\n ");
      test_GPS();
      break;
    case 'A':
      Serial.println("\n In case 'All'\n ");
      test_weatherShield();
      delay(500);
      test_temperature();
      delay(500);
      test_transmitter();
      delay(500);
      test_SD();
      delay(500);
      test_GPS();
      delay(500);
      break;
    default:
      Serial.println("Skipped all cases!\n");
      delay(500);
      
  }
}


void test_weatherShield() {
  Serial.println("\n In test_weatherShield()\n ");
  float pressure = myPressure.readPressure()/6894.75;
  float WStemp = myPressure.readTempF();
  Serial.print("Pressure(psi):");
  Serial.print(pressure, 2);
  Serial.print("  WS_Temp(f):");
  Serial.println(WStemp, 2);
}


void test_temperature() {
  Serial.println("\n In test_temperature()\n ");
  float rawAnlg = analogRead(tempPin);    // get analog signal from controller
  float rawV = rawAnlg*4.8828;    // convert to voltage (V)
  float temp = 25 - ((750 - rawV)/10);    // convert to temperature (C)
  Serial.print("Temp(C):");
  Serial.println(temp);
  delay(500);
}


void test_transmitter() {
//  char incomingData = 0;
//  Serial2.write("OK");
//  Serial2.write(0x20);
//  incomingData = Serial2.read();
//  Serial.print(incomingData);
//  char string = '\0';
//  char incomingData = '\0';
//  while (Serial.available() > 0) {
//    incomingData = Serial.read();
//    string = string + incomingData;
//  }
  Serial3.print("OK");
//  Serial3.println(incomingData);
  delay(2000);
//}
}
     //if (incomingData == 'H') {
      //digitalWrite(powerTxPin, HIGH);
      //break;
    //}
    //else if (incomingData == 'L') {
      //digitalWrite(powerTxPin, LOW);
     //break;
    //}
    //else {
      //Serial.println("Nothing happening!\n");
    //}
  //}
  //digitalWrite(txPin, HIGH);
  //delay(2000);
  //digitalWrite(txPin, LOW);
  //delay(2000);
  //digitalWrite(txPin, HIGH);
  //delay(2000);
  //digitalWrite(txPin, LOW);
  
void test_SD() {
  delay(500);
}

void test_GPS() {
  
}
