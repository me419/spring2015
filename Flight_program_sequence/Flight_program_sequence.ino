/*
UHABS-3 flight program
Author: Fernando Aragon
History:
3.17.15 - Created

Program for UHABS-3 operation for ME-419 class.
Sensor are GPS, Weathershield, IMU, Magnetometer, Arducam, SDshield.
Gathers, saves and sends telemetry.
Releases parachute
Monitors battery levels and runs navigation algorithm.
Based on example files of each sensor
*/

#include<I2Cdev.h>
#include<MPU6050_6Axis_MotionApps20.h>
#include<Wire.h>
#include <MPL3115A2.h>

//Create instances of all necessary sensors
MPL3115A2 myPressure;

//*********************** DEBUGGING VARS *********************
// Debbugging variables
bool NAV_FLAG = false;  //DEBUGGING
bool RELEASE_FLAG = true;  //DEBUGGING
// denugging modes for each one of the sensors
// W is for weathershield
// M is for IMU
// G is for GPS
char TESTING_SENSOR = 'W';  //DEBUGGING

//*********************** DEBUGGING VARS *********************


void setup() {
  // Imitialize communications
  Wire.begin();          // Join i2c bus
  Serial.begin(9600);    // Start serial comm.
  
  // initialize all sensors
  myPressure.begin();    // Get weather shield online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
}

void loop() {
  Serial.println("Starting main loop...\n");  //DEBUGGING
  if (!NAV_FLAG) {
    //Serial.println("Not NAV_FLAG if loop\n");  //DEBUGGING
    void read_sensors();
    if (!RELEASE_FLAG) {
      //Serial.println("Not RELEASE_FLAG if loop\n");  //DEBUGGING
      flight_sequence();
    }
    else {
      //Serial.println("Yes RELEASE_FLAG if loop\n");  //DEBUGGING
      landing_sequence();
    }
  }
  else {
    //Serial.println("Yes NAV_FLAG if loop\n");  //DEBIUGGING
    nav_sequence();
  }
  delay(1000);
}


void read_sensors() {
}


void flight_sequence() {
}


void landing_sequence() {
}


void nav_sequence() {
}

//******************* EXTRA DEBUGGING FUNCTIONS****************************
  
 
 
 // if ARMED_RELEASE_FLAG
 // Check sonar sensor
 
 // if ground < 10ft
 // release parachute
 
 
 // Calculate attitude rates 
 
 // Get voltages


// Create string for transmission


// Listen for request


// Send string


// Change target coordinates if requested


// NAV Mode
