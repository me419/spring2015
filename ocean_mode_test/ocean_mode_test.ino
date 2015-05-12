/*
Ocean_mode_test

Fernando Aragon
5/9/15 - Created
*/

#include<TinyGPS.h>
#include <Wire.h>
#include<stdlib.h>
#include "I2Cdev.h"
#include "MPL3115A2.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mag.h"
#include <SD.h>
#include <SPI.h>

// Code flags
bool AUTOPILOT = false;
bool TXTIME = false;               // signals if it is transmission time
bool GPSFIX = false;


// Global variables
//float targetCoords[] = {21.285625, -157.673312}; //[latitude, longitude] Sandy Beach Park
int transmissionTime[] = {900, 1000};        // 9am - 10am
File myFile;


float  intTemp, extTemp, airPress, lat, lon, alt, vel, ypr[3], ypr_rate[3], motoVolt;  //, vCamVolt, txVolt;    // variables for sensor readings
char  tmpintTemp[20], tmpextTemp[20],tmpairPress[20], tmplat[20], tmplon[20], tmpalt[20], tmpvel[20], tmpRoll[20], tmpPitch[20], tmpYaw[20], tmpRollR[20], tmpPitchR[20], tmpYawR[20], tmpMotoVolt[20];    // temporary holders to float2string conversion
bool newData;
//variables for the imu
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// telemetry string
String telemetryStr;      // Telemetry string - Start/end string(<>) - field delimiter ($) - identifier (capital letter after field delimiter) - delimiter related fields (,)
                        // EX: <$I'inttemp',$E'exttemp'$P'airpress'$L'lat,lon'...>
                        // Order:
                        // 1 - I:int. temp
                        // 2 - E:ext. temp
                        // 3 - P:air press
                        // 4 - L:latitude and longitude
                        // 5 - A:altitude
                        // 6 - U:velocity
                        // 7 - T:roll,pitch,yaw
                        // 8 - R:attitute rates (droll, dpitch, dyaw)
                        // 9 - V:motor voltage
                        // 10 - B:motor voltage
                        // 11 - Z:Autopilot mode, 
                        // modes are 1/0



// timeouts (all in milliseconds)
int receive_comm_timeout = 2000;


//timers
unsigned long int send_comm_LAST = 0;
unsigned long int receive_comm_LAST = 0;
unsigned long int save_tele_LAST = 0;
unsigned long int queue_gps_LAST = 0;
unsigned long int read_sensors_LAST = 0;
unsigned long int nav_calculation_LAST = 0;
unsigned long int init_nav_LAST = 0;

// function frequency (all in milliseconds)
int send_comm_FREQ = 30000;
int receive_comm_FREQ =10000;
int save_tele_FREQ =10000;
int queue_gps_FREQ =1000;
int read_sensors_FREQ =500;
int nav_calculation_FREQ=500;
int init_nav_FREQ = 5000;


//Pin assignment
int txPin = 36;                  // Connects to pin 7 XTEND power/shutdown module
int tempPin = A15;
int arduPin = A13;
int motPin = A14;
int CSPin = 53;


// Navigation variables
float currCoords[2];
float a, b, c, d, theta;
int rightMotorPin = 28;
int leftMotorPin = 29;
//float currentCoords[2];
float targetCoords[2];





// initialize sensor intances
MPL3115A2 myWShield;
TinyGPS gps;
MPU6050 imu;

//---------------------------------------SETUP---------------------------------------------------
void setup() {
  Serial2.begin(9600);    //connect to xtend serial line
  Serial.begin(9600);
  
  Wire.begin();        // Join i2c bus
  
  // configure IMU
  imu.initialize();  // Get imu online
  devStatus = imu.dmpInitialize();
  imu.setXGyroOffset(220);
  imu.setYGyroOffset(76);
  imu.setZGyroOffset(-85);
  imu.setZAccelOffset(1788);
  if (devStatus == 0) {
    imu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = imu.getIntStatus();
    dmpReady = true;
    packetSize = imu.dmpGetFIFOPacketSize();
  }
  
  // configure Weather Shield
  myWShield.begin(); // Get Weather shield online
  myWShield.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myWShield.setOversampleRate(7); // Set Oversample to the recommended 128
  myWShield.enableEventFlags(); // Enable all three pressure and temp event flags
  
  setup_SDcard();
  
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  
  
  
  
  currCoords[0] = 152.150541041; 
  currCoords[1] = 167.616914685;
  targetCoords[0] = 33.483634713; 
  targetCoords[1] = 157.021305651;
  init_nav();
  currCoords[0] = 140.128374784;
  currCoords[1] = 39.5568395864;
  check_nav();
  
  
  
  
  
  
  
  
  
}


//---------------------------------------MAIN LOOP------------------------------------------------
void loop() {
//  queue_imu();
//  Serial.print("ypr\t");
//  Serial.print(degrees(ypr[0]));
//  Serial.print("\t");
//  Serial.print(degrees(ypr[0]));
//  Serial.print("\t");
//  Serial.println(degrees(ypr[2]));
//  delay(5);
  
  
//  if ((millis()-send_comm_LAST) > send_comm_FREQ){send_comm();}
//  if ((millis()-receive_comm_LAST) > receive_comm_FREQ) {receive_comm(receive_comm_timeout);}
//  if ((millis()-save_tele_LAST) > save_tele_FREQ) {save_tele();}
//  if ((millis()-queue_gps_LAST) > queue_gps_FREQ) {queue_gps();}
//  if((millis()-read_sensors_LAST) > read_sensors_FREQ) {read_sensors();}
//  if((millis()-nav_calculation_LAST) > nav_calculation_LAST && GPSFIX) {nav_calculation();}
   if ((millis()-init_nav_LAST) > init_nav_LAST) {init_nav();}


  
  
//  currentCoords[0] = 106.198814065;
//  currentCoords[1] = 352.000704306;
//  check_nav();
  
}


//---------------------------------------RECEIVE_COMM---------------------------------------------
void receive_comm(int secTimeOut) {
  Serial.println("\nRECEIVE COMM!");
  receive_comm_LAST =millis();
  
  // flow control variables
  bool beg = false;
  bool fin = false;
  bool thru = false;
  bool sep = false;
  bool save = false;
  char current;
  
  // temporary variables
  String lat, lon, startTime, endTime, autoMsg;
  
  int time = millis();
//  Serial.println("Before while loop");
  while (Serial2.available()) {
    Serial.println("Entered while loop");
//    if (secTimeOut < millis() - time) {
//      Serial.println("Timed out...");  //DEBUGGING
//      break;
//    }
    Serial.println("Entered while loop");
    char incomingByte = Serial2.read();
    
    Serial.println(incomingByte);  //DEBUGGING
    
    if (incomingByte == '<') {
      thru = true;
      beg = true;
      lat = "";
      lon = "";
      startTime = "";
      endTime = "";
      autoMsg = "";
      continue;
    }
    if (incomingByte == '>') {
//      Serial.println("\n");  //DEBUGGING
      thru = false;
      fin = true;
      if (beg && fin) {
        save = true;
        break;
      }
    }
    if (incomingByte == ',') {
      sep = true;
      continue;
    }
    if (incomingByte == '$') {
      current = Serial2.read();
      sep = false;
//      Serial.println(current);  //DEBUGGING
      continue;
    }
    if (thru) {
      if (current == 'L' && sep) {
//        Serial.println(" adding to lon");  //DEBUGGING
        lon = String(lon + incomingByte);
      }
      else if (current == 'L') {
//        Serial.println(" adding to lat");  //DEBUGGING
        lat = String(lat + incomingByte);
      }
      else if (current == 'A') {
//        Serial.println(" adding to abort");  //DEBUGGING
        autoMsg = String(autoMsg + incomingByte);
      }
      else if (current == 'T' && sep) {
//        Serial.println(" adding to endTime");  //DEBUGGING
        endTime = String(endTime+ incomingByte);
      }
      else if (current == 'T') {
//        Serial.println(" adding to startTime");  //DEBUGGING
        startTime = String(startTime + incomingByte);
      }
    }
//    delay(1000);  //DEBUGGING
  }
  if (save) {
    if (targetCoords[0] != lat.toFloat())
      targetCoords[0] = lat.toFloat();
    if (targetCoords[1] != lon.toFloat())
      targetCoords[1] = lon.toFloat();
    if (AUTOPILOT != (bool)autoMsg)
      AUTOPILOT = (bool)autoMsg;
    if (transmissionTime[0] != startTime.toInt())
      transmissionTime[0] = startTime.toInt();
    if (transmissionTime[1] != endTime.toInt())
      transmissionTime[1] = endTime.toInt();
    Serial.print("Printing lat: ");  //DEBUGGING
    Serial.println(lat);   //DEBUGGING
    Serial.print("Printing lon: ");  //DEBUGGING
    Serial.println(lon);   //DEBUGGING
    Serial.print("Printing abort: ");  //DEBUGGING
    Serial.println(autoMsg);   //DEBUGGING
    Serial.print("Printing startTime: ");  //DEBUGGING
    Serial.println(startTime);   //DEBUGGING
    Serial.print("Printing endTime: ");  //DEBUGGING
    Serial.println(endTime);   //DEBUGGING
    
    Serial.println();
  }
}


//---------------------------------------SEND_COMM-----------------------------------------------
void send_comm() {
  Serial.print("\nSEND COMM!");
  send_comm_LAST =millis();
  String outStr = "<";
  outStr += telemetryStr;
  outStr += ">";
  Serial2.print(outStr);
  Serial.println("\nTelemetry sent!");
  Serial.println(outStr);
  Serial.println();
}


//---------------------------------------SAVE_TELE-----------------------------------------------
void save_tele() {
  Serial.print("\nSAVE TELE!");
  save_tele_LAST =millis();
  String saveString = String(save_tele_LAST);
  saveString += ",";
  saveString += telemetryStr;
  if (myFile) {
    myFile.println(saveString);
    myFile.flush();
    }
    
  // if the file isn't open, pop up an error and blink LED:
  else {
    Serial.println("\nfile error - log file not available! Stopping");  //DEBUG
  } 
}

/*
SETUP SDCARD check correct operation of card, selects file name and creates it
*/
void setup_SDcard() {
  // Initialize card
  pinMode(CSPin, OUTPUT);
  
  // check if card is initialized correctly
  if (!SD.begin(CSPin)) {
    Serial.println("Card failed, or not present!");  //DEBUG
    Serial.println("Not doing anything else.");  //DEBUG
    return;    // stop program
  }
  Serial.println("card initialized.");  //DEBUG

// Open data file
  String fileName = "test1.csv";
  char buf[16];
  int cnt = 1;
  
  // Find a unique file name
  fileName.toCharArray(buf,16);
  while (SD.exists(buf)){
    cnt += 1;
    fileName = "test"+String(cnt)+".csv";
    fileName.toCharArray(buf,16);
  }
  Serial.println("Opening new logfile: "+fileName);  //DEBUG
  Serial.println("");  //DEBUG
  myFile = SD.open(buf, FILE_WRITE);
}


//---------------------------------------READ_SENSORS--------------------------------------------
void read_sensors() {
  
  Serial.println("\nREAD SENSORS");
  read_sensors_LAST =millis();
  telemetryStr = "";
  
  // Read internal temperature
  intTemp = myWShield.readTempF();
  dtostrf(intTemp,5 ,2, &tmpintTemp[0]);
  add2telemetry("I", tmpintTemp);

  // Read external temperature
  int rawTempReading = analogRead(tempPin);
  float tempVolt = (rawTempReading* 5.0)/1024.0;
  extTemp = (tempVolt - 0.5) * 100;      // temp in Celsius
  extTemp = (extTemp * 9.0 / 5.0) + 32.0;    // temp in fahrenheit
  dtostrf(extTemp,5 ,2, &tmpextTemp[0]);
  add2telemetry("E", tmpextTemp);
  
  //Read air pressure
  airPress = myWShield.readPressure();
  dtostrf(airPress,9 ,2, &tmpairPress[0]);
  add2telemetry("P", tmpairPress);
  
  
  // Read GPS
  queue_gps();
  dtostrf(currCoords[0],10, 6, &tmplon[0]);
  add2telemetry("L", tmplon);
  dtostrf(currCoords[1],10, 6, &tmplat[0]);
  add2telemetry("", tmplat);
  dtostrf(alt,5, 2, &tmpalt[0]);
  add2telemetry("A", tmpalt);
  dtostrf(vel,5, 2, &tmpvel[0]);
  add2telemetry("U", tmpvel);
  
  // Read attitute and attitute rates 
  //  y: -180(fron to front) to 180(front to front)
  //  p: -90(front to top) to 90(back to top)
  //  r: -90(left to top) to 90(right to top)
  get_att_rates();
  dtostrf(degrees(ypr[2]),5, 2, &tmpRoll[0]);
  add2telemetry("T", tmpRoll);
  dtostrf(degrees(ypr[1]),5, 2, &tmpPitch[0]);
  add2telemetry("", tmpPitch);
  dtostrf(degrees(ypr[0]),5, 2, &tmpYaw[0]);
  add2telemetry("", tmpYaw);
  
  dtostrf(degrees(ypr_rate[2]),5, 2, &tmpRollR[0]);
  add2telemetry("R", tmpRollR);
  dtostrf(degrees(ypr_rate[1]),5, 2, &tmpPitchR[0]);
  add2telemetry("", tmpPitchR);
  dtostrf(degrees(ypr_rate[0]),5, 2, &tmpYawR[0]);
  add2telemetry("", tmpYawR);
  

  // Read voltage
  const int voltageRef = 5;  // Reference voltage for analog read
  float motoValue = analogRead(motPin);
  motoVolt = (motoValue * voltageRef) / 1023;
  dtostrf(motoVolt,3, 2, &tmpMotoVolt[0]);
  add2telemetry("V", tmpMotoVolt);
  
//  const int RS = 10;          // Shunt resistor value (in ohms)
//  const int VOLTAGE_REF = 5;  // Reference voltage for analog read
//  float sensorValue = analogRead(arduPin);   // Variable to store value from analog read
//  // Remap the ADC value into a voltage number (5V reference)
//  ArduVolt = (sensorValue * VOLTAGE_REF) / 1023;
//  float current = sensorValue / (10 * RS);       // Calculated current value
//  dtostrf(vel,3, 2, &tmpMotoVolt[0]);
//  add2telemetry(tmpArduVolt);
  
  // Autopilot signal
  telemetryStr += "$Z";
  if (AUTOPILOT) {
    telemetryStr += "1";
  }
  else {
    telemetryStr += "0";
  }
  
  Serial.println(telemetryStr);  //DEBUG
  Serial.println();  //DEBUG
}

/*
ADD2TELEMETRY converts float to strings to append to telemetry string
*/
void add2telemetry(String identifier, char *tmp) {
  if (identifier == "") {
    telemetryStr += String(tmp);
    telemetryStr += ",";
  }
  else {
    telemetryStr += "$";
    telemetryStr += identifier;
    telemetryStr += String(tmp);
    telemetryStr += ",";
  }
}


//---------------------------------------QUEUE _GPS----------------------------------------------
void queue_gps() {
  unsigned long age;
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial1.available()) {
      char c = Serial1.read(); //get GPS data
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData) {
    gps.f_get_position(&lat, &lon, &age);
    alt = gps.altitude();    // dont know which to use
    alt = gps.f_altitude();
    vel = gps.f_speed_mph();
  }
  else {
    // return if no GPS fix 
//    lat = 00.000000;
//    lon = 00.000000;
//    alt = 000;
//    vel = 000;
    lon = -151.547689;
    currCoords[0] = lon;
    currCoords[1] = lat;
    lat = 29.764590;
    alt = 56.43;
    vel = 564;
    check_nav();
  }
}


//---------------------------------------QUEUE _IMU----------------------------------------------
void queue_imu() {
  mpuInterrupt = false;
  mpuIntStatus = imu.getIntStatus();
  fifoCount = imu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        imu.resetFIFO();
  } 
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = imu.getFIFOCount();
    imu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    imu.dmpGetQuaternion(&q, fifoBuffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void get_att_rates() {
  float dYaw[4];
  float dPitch[4];
  float dRoll[4];
  float lastRoll, lastPitch, lastYaw, dt;
  int lastTime;
  float rollSum = 0;
  float yawSum = 0;
  float pitchSum = 0;
  for (int i= 1; i < 6; i++) {
    queue_imu();
    if (i == 1) {
      lastYaw = ypr[0];
      lastPitch = ypr[1];
      lastRoll = ypr[2];
      lastTime = millis();
      continue;
    }
    dt = (millis()-lastTime)/1000.0;
    dYaw[i] = (ypr[0]-lastYaw)/dt;
    dPitch[i] = (ypr[1]-lastPitch)/dt;
    dRoll[i] = (ypr[2]-lastRoll)/dt;
    
    lastYaw = ypr[0];
    lastPitch = ypr[1];
    lastRoll = ypr[2];
    lastTime = millis();
  }
  for (int j= 1; j < 5; j++) {
    yawSum += dYaw[j];
    pitchSum += dPitch[j];
    rollSum += dRoll[j];
  }
  ypr_rate[0] = yawSum/4.0;
  ypr_rate[1] = pitchSum/4.0;
  ypr_rate[2] = rollSum/4.0;
}

void dmpDataReady() {
    mpuInterrupt = true;
}
//---------------------------------------NAV_CALCULATION-----------------------------------------
void nav_calculation() {}

void turn_left() {
  Serial.println("Left");
  digitalWrite(rightMotorPin, HIGH);
  digitalWrite(leftMotorPin, LOW);
}

void turn_right() {
  Serial.println("Right");
  digitalWrite(rightMotorPin, LOW);
  digitalWrite(leftMotorPin, HIGH);
}

void go_straight() {
  Serial.println("Straight");
  digitalWrite(rightMotorPin, HIGH);
  digitalWrite(leftMotorPin, HIGH);
}

void init_nav() {
  double dy = targetCoords[1] - currCoords[1];
  double dx = targetCoords[0] - currCoords[0];
  float r = sqrt(pow(dy,2) + pow(dx,2))* 0.2;
  theta = atan2(dy, dx );
//  Serial.print(theta);
  float phi = theta - PI/2.0;
  float omega = theta + PI/2.0;
  a = currCoords[0] + r * cos(phi);
  b = currCoords[1] + r * sin(phi);
  c = currCoords[0] + r * cos(omega);
  d = currCoords[1] + r * sin(omega);
}

void check_nav() {
  Serial.println("theta:");
  Serial.println(theta);
  Serial.println(c);
  Serial.println(d);
  if (theta > -(PI/2.0) && theta < (PI/2.0)) {
    if (currCoords[1] <= (currCoords[0]-a)*tan(theta) + b) {
      turn_left();
    }
    else if (currCoords[1] >= (currCoords[0]-c)*tan(theta) + d) {
      turn_right();
    }
    else {
      go_straight();
    }
  }
  else {
    if (currCoords[1] >= (currCoords[0]-a)*tan(theta) + b) {
      turn_left();
    }
    else if (currCoords[1] <= (currCoords[0]-c)*tan(theta) + d) {
      turn_right();
    }
    else {
      go_straight();
    }
  }
}
