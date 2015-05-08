/*
Read_sensors_test
Fernando Aragon
4/4/15 - Created

Read all the sensor values and update current module state:
  Read air pressure from weather shield
  Read time, position (latitude, longitude, altitude) from GPS
  Calculate velocity
  Read internal/external temperature
  Read attitute(roll, pitch, yaw) from IMU
  Calculate attitute rates
  Measure bus currents/voltages
  Update current state of the module with all readings
*/

// String: <INTTEMP,EXTTEMP,AIRPRESS,LAT,LON,ALT,VEL,ROL,PITCH,YAW,ROLL_DOT,PITCH_DOT,YAW_DOT,VOLT>

#include<TinyGPS.h>
#include <Wire.h>
#include<stdlib.h>
#include "mag.h"
#include "I2Cdev.h"
#include "MPL3115A2.h"
//#include "MPU6050.h"

// initialize pins
int tempPin = A15;
int arduPin = A13;
int motPin = A14;

// initialize some variables
float  intTemp, extTemp, airPress, lat, lon, alt, vel, ArduVolt;  //, vCamVolt, txVolt;
char  tmpintTemp[20], tmpextTemp[20],tmpairPress[20], tmplat[20], tmplon[20], tmpalt[20], tmpvel[20], tmpArduVolt[20];
unsigned long time, date;
bool newData;
String telemetryStr;



// initialize sensor intances
MPL3115A2 myWShield;
TinyGPS gps;
//MPU6050 imu;


//INITIALIZE VARIABLES FOR IMU  --> ROLL, PITCH, YAW and each rate.


// Initialize all programming flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)


void setup() {
  Serial.begin(9600);  //DEBUGGING
  Wire.begin();        // Join i2c bus
  myWShield.begin(); // Get Weather shield online
  
  // configure Weather Shield
  myWShield.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myWShield.setOversampleRate(7); // Set Oversample to the recommended 128
  myWShield.enableEventFlags(); // Enable all three pressure and temp event flags 
}

void loop() {
  read_sensors();
  delay(500);
}


/*
READ_SENSORS reads all the sensor values and update current module state.
Tasks: Read air pressure from weather shield
       Read time, position (latitude, longitude, altitude) from GPS
       Calculate velocity
       Read internal/external temperature
       Read attitute(roll, pitch, yaw) from IMU
       Calculate attitute rates
       Measure bus currents/voltages
       Update current state of the module with all readings
*/
void read_sensors() {
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
  lat = 00.000000;
  lon = 00.000000;
  alt = 00.00;
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
}



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
