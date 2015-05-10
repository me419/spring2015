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
#include "MPU6050.h"
#include "mag.h"


// Code flags
bool AUTOPILOT = false;
bool TXTIME = false;               // signals if it is transmission time
bool GPSFIX = false;


// Global variables
float targetCoords[] = {21.285625, -157.673312}; //[latitude, longitude] Sandy Beach Park
int transmissionTime[] = {900, 1000};        // 9am - 10am




int lastAtt[3];    // last recorded attitude [roll, pitch yaw]
float  intTemp, extTemp, airPress, lat, lon, alt, vel, ypr[3], ArduVolt;  //, vCamVolt, txVolt;    // variables for sensor readings
char  tmpintTemp[20], tmpextTemp[20],tmpairPress[20], tmplat[20], tmplon[20], tmpalt[20], tmpvel[20], tmpArduVolt[20];    // temporary holders to float2string conversion
bool newData;
uint8_t fifoBuffer[64]; // FIFO storage buffer
//Quaternion q; 

String telemetryStr;      // Telemetry string - Start/end string(<>) - field delimiter ($) - identifier (capital letter after field delimiter) - delimiter related fields (,)
                        // EX: <$I'inttemp'$E'exttemp'$P'airpress'$L'lat,lon'...>
                        // Order:
                        // 1 - I:int. temp
                        // 2 - E:ext. temp
                        // 3 - P:air press
                        // 4 - L:latitude and longitude
                        // 5 - A:altitude
                        // 6 - V:velocity
                        // 7 - T:(roll,pitch,yaw)
                        // 8 - R:attitute rates
                        // 9 - V:arduino voltage
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


// function frequency (all in milliseconds)
int send_comm_FREQ =30000;
int receive_comm_FREQ =10000;
int save_tele_FREQ =30000;
int queue_gps_FREQ =1000;
int read_sensors_FREQ =500;
int nav_calculation_FREQ=500;


//Pin assignment
int txPin = 36;                  // Connects to pin 7 XTEND power/shutdown module
int tempPin = A15;
int arduPin = A13;
int motPin = A14;


// initialize sensor intances
MPL3115A2 myWShield;
TinyGPS gps;
MPU6050 imu;

//---------------------------------------SETUP---------------------------------------------------
void setup() {
  Serial2.begin(9600);    //connect to xtend serial line
  Serial.begin(9600);
  
  Wire.begin();        // Join i2c bus
  myWShield.begin(); // Get Weather shield online
  
  // configure Weather Shield
  myWShield.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myWShield.setOversampleRate(7); // Set Oversample to the recommended 128
  myWShield.enableEventFlags(); // Enable all three pressure and temp event flags 
}


//---------------------------------------MAIN LOOP------------------------------------------------
void loop() {
  if ((millis()-send_comm_LAST) > send_comm_FREQ){send_comm();}
  if ((millis()-receive_comm_LAST) > receive_comm_FREQ) {receive_comm(receive_comm_timeout);}
  if ((millis()-save_tele_LAST) > save_tele_FREQ) {save_tele();}
  if ((millis()-queue_gps_LAST) > queue_gps_FREQ) {queue_gps();}
  if((millis()-read_sensors_LAST) > read_sensors_FREQ) {read_sensors();}
  if((millis()-nav_calculation_LAST) > nav_calculation_LAST && GPSFIX) {nav_calculation();}
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
  Serial.println("Before while loop");
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
  Serial2.print(telemetryStr);
  Serial.println("\nTelemetry sent!");
  Serial.println(telemetryStr);
  Serial.println();
}


//---------------------------------------SAVE_TELE-----------------------------------------------
void save_tele() {
  
}


//---------------------------------------READ_SENSORS--------------------------------------------
void read_sensors() {
  
  Serial.println("\nREAD SENSORS");
  read_sensors_LAST =millis();
  telemetryStr = "";
  
  // Read internal temperature
  intTemp = myWShield.readTempF();
  dtostrf(intTemp,5 ,2, &tmpintTemp[0]);
  add2telemetry(tmpintTemp);

  // Read external temperature
  int rawTempReading = analogRead(tempPin);
  float tempVolt = (rawTempReading* 5.0)/1024.0;
  extTemp = (tempVolt - 0.5) * 100;      // temp in Celsius
  extTemp = (extTemp * 9.0 / 5.0) + 32.0;    // temp in fahrenheit
  dtostrf(extTemp,5 ,2, &tmpextTemp[0]);
  add2telemetry(tmpextTemp);
  
  //Read air pressure
  airPress = myWShield.readPressure();
  dtostrf(airPress,9 ,2, &tmpairPress[0]);
  add2telemetry(tmpairPress);
  
  // Read GPS
  unsigned long age;    //needed for GPS function
//  lat = 00.000000;
//  lon = 00.000000;
   lon = -151.547689;
   lat = 29.764590;

  alt = 56.43;
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
  dtostrf(lon,10, 6, &tmplon[0]);
  add2telemetry(tmplon);
  dtostrf(lat,10, 6, &tmplat[0]);
  add2telemetry(tmplat);
  dtostrf(alt,5, 2, &tmpalt[0]);
  add2telemetry(tmpalt);
  dtostrf(vel,5, 2, &tmpvel[0]);
  add2telemetry(tmpvel);
  
  // Read attitute
  //  y: -180(fron to front) to 180(front to front)
  //  p: -90(front to top) to 90(back to top)
  //  r: -90(left to top) to 90(right to top)
  imu.dmpGetQuaternion(&q, fifoBuffer);
  imu.dmpGetGravity(&gravity, &q);
  imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  
  
  // Read voltage
  const int RS = 10;          // Shunt resistor value (in ohms)
  const int VOLTAGE_REF = 5;  // Reference voltage for analog read
  float sensorValue = analogRead(arduPin);   // Variable to store value from analog read
  // Remap the ADC value into a voltage number (5V reference)
  ArduVolt = (sensorValue * VOLTAGE_REF) / 1023;
  float current = sensorValue / (10 * RS);       // Calculated current value
  dtostrf(vel,3, 2, &tmpArduVolt[0]);
  add2telemetry(tmpArduVolt);






  
  Serial.println(telemetryStr);
  Serial.println();
}

/*
ADD2TELEMETRY converts float to strings to append to telemetry string
*/
void add2telemetry(char *tmp) {
  telemetryStr += String(tmp);
  telemetryStr += ",";
  
//  if (signal == 'i') {
//    telemetryStr += String(intgr);
//    telemetryStr +=",";
//  }
//  else if (signal == 'f') {
//    dtostrf(extTemp,7 ,2, &tmpChar[0]);
//    telemetryStr += String(tmpChar);
//    telemetryStr +=",";
//  }
}


//---------------------------------------QUEUE _GPS----------------------------------------------
void queue_gps() {
  
}

//---------------------------------------NAV_CALCULATION-----------------------------------------
void nav_calculation() {
  
}
