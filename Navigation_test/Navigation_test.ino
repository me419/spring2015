/*
Navigation_test
Fernando Aragon
4/17/15 - Created

Description:
Test navigation algorithm by:
  Getting drift vector from GPS readings
  Calculate target and heading vector
  Get actual heading
  Calculate and actuate thrust rotation (one motor)
  Run both motors
  Check battery voltage
  
For hardware connections and software details access project website
*/

#include<TinyGPS.h>
#include <Wire.h>
#include "NanoSatisfi_MAG3110.h"
//#include<math.h>


// initialize some variables
float dist2target = 2.0;    // (mi) needs to be changed by another function
float targetCoords[] = {21.285625, -157.673312}; //Latitude, longitude Sandy Beach Park
float currCoords[2], oldCoords[2];    // Lat/Lon readings to determine drift
float driftVect[2], targetVect[2], headingVect[2];    // dift is for current, target is from current pos to target and heading is both combined
float northVect[2] = {1.0, 0.0};
float currHead, targetHead;    // actual heading and req heading to target ccordinates
int headEpsilon = 5;    // correct course if heading is off by more than epsilon degrees
int rotFactor;    //Need to figure out rotation deg/millisecond
int driftDelay = 15000;    //ms to sleep between GPS readings
float diffHead;

float volt1, volt2;    // battery voltage to start navigation
float minVolt = 1.0;    // should change to cut-off voltage
int rightMotorPin = 28;
int leftMotorPin = 29;
float earthR = 3959.1;    // Earths radius (miles)
//const float pi = 3.141592;
//const float r2d = 180/PI;    // conv rads to degrees
//const float d2r = PI/80;    //conv degrees to rads



// initialize sensor instances
TinyGPS gps;  //instance of gps
NanoSatisfi_MAG3110 mag;    // instance of magnetometer

// Initialize all programming flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeGFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)
int timeout = 2;


void setup() {
  Serial.begin(9600);  //Arduino-PC serial comm
  Serial1.begin(9600); // start GPS serial comm on Serial1
  mag.configMag();          // turn the MAG3110 on
  Serial.println("----Starting GPS Testing---");
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
}

void loop() {
  get_drift();
  navigation_loop();
}

/*
GET_DRIFT calculates dirft (water current) vector
Tasks:  Read GPS coordinates
        Wait predetermined time
        Read GPS coordinates
        Calculate vector
*/
void get_drift() {
  unsigned long age;    //needed for GPS function
  gps.f_get_position(&oldCoords[0], &oldCoords[1], &age);
  delay(driftDelay);
  gps.f_get_position(&currCoords[0], &currCoords[1], &age);
  //float latDelt = (currCoords[0] - oldCoords[0])*((earthR*2*pi)/360.0);    //formulas to calculate lat/lon deltas
  //float lonDelt = (currCoords[1] - oldCoords[1])*earthR*cos(currCoords[0]*d2r);
  vector_diff(oldCoords[0], oldCoords[1], currCoords[0], currCoords[1], &driftVect[0]);
}

/*
NAVIGATION_LOOP goes through navigation algorithm until cut-off voltage is reached
Taskes:  Calculate target vector
         Calculate heading vector
         Calculate heading required (degrees)
         Get current heading
         Calculate if rotation needed and run single motor
         Move forward
         Check battery voltage
*/
void navigation_loop() {
  String turn = "straight";
  unsigned long age;    //needed for GPS function
  while (volt1 > minVolt && volt2 > minVolt && dist2target > 0.003788) {    // voltages within limit and more than 20ft from target
    gps.f_get_position(&currCoords[0], &currCoords[1], &age);
    vector_diff(currCoords[0], currCoords[1], targetCoords[0], targetCoords[1], &targetVect[0]);
    headingVect[0] = targetVect[0] - driftVect[0]; headingVect[1] = targetVect[1] - driftVect[1];
    //ratio = (vector_dot(&headingVect, &northVect))/(vector_mag(&headingVect)*vector_mag(northVect));
    float angle = atan2(headingVect[0], headingVect[1]);
    if (angle < 0)
      targetHead = degrees(angle + 2*PI);
    else
      targetHead = degrees(angle);
    currHead = mag.getHeading(mag.x_value(),mag.y_value(),mag.z_value());
    diffHead = targetHead - currHead;
    if (diffHead >= 0) {
      if (diffHead < 180) {turn = "left";}
    }
    if (diffHead >= -180) {
     if (diffHead <= 0) { turn = "right"; }
    } 
    if (diffHead < -180) { turn = "left"; }
    if (diffHead >= 180) {turn = "right";}
    
    if (turn == "straight") {
      digitalWrite(leftMotorPin,HIGH);
      digitalWrite(rightMotorPin, HIGH);
    }
    if (turn == "left")
      left_turn();
    if (turn == "right")
      right_turn();
    
  //volt1 =
  //volt2 = 
  }
}

/*
VECTOR_DIFF gives vector with delta lat & lon
*/
void vector_diff(float lat1, float lon1, float lat2, float lon2, float *vector) {
  float latDelt = (lat2 - lat1)*((earthR*2*PI)/360.0);    //formulas to calculate lat/lon deltas
  float lonDelt = (lon2 - lon1)*earthR*cos(radians(lat2));
  vector[0] = latDelt; vector[1] = lonDelt;
}

///*
//VECTOR_DOT claculates dot product
//*/
//float vector_dot(float *vector1, float *vector2) {
//  return vector1[0]*vector2[0] + vector1[1]*vector2[1]
//}
//
///*
//VECT_MAG calculate vector magnitude
//*/
//float vector_mag(float *vector) {
//  return sqrt(sq(vector[0]) + sq(vector[1]))
//}

/*
RIGHT_TURN reads heading and rotates left motor until within headEpsilon (var) degrees from target heading
*/
void right_turn() {
  if(currHead + headEpsilon > targetHead){
    if(currHead - headEpsilon < targetHead) {
      digitalWrite(leftMotorPin, LOW);
      digitalWrite(rightMotorPin, LOW);
      return;
    }
  }
  diffHead = targetHead - currHead;
  if(diffHead < -180) {
    return;
  }
  if(diffHead >= 0) {
    if(diffHead < 180){
      return;
      }
  }
  digitalWrite(leftMotorPin, HIGH);
  digitalWrite(rightMotorPin, LOW); 
  currHead = mag.getHeading(mag.x_value(),mag.y_value(),mag.z_value());
  right_turn();
}

/*
LEFT_TURN reads heading and rotates right motor until within headEpsilon (var) degrees from target heading
*/
void left_turn() {
  
}







/*
Navigation program

- Read battery voltages
- Get drift
  - Read GPS coordinates
  - Wait 15 sec
  - Read GPS coordinates again
- Navigate (while batteries above cut off power)
  - Calculate target vector
  - Calculate heading vector (subtract one another to get it)
  - Calculate heading required in degrees
  - Read magnetometer heading
  - If headings offset for more than delta
    - Calculate time to thrust for rotation
    - Run single motor
- Run both motor to go straight
- Check battery voltage


*/











