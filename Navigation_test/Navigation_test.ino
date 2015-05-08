/*
Navigation_test
Fernando Aragon
4/17/15 - Created
5/6/15 - Added mag library functionality and change way to get GPS data

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
#include "mag.h"

// ********************LCD INITIALIZATIONS*************************
#include <LiquidCrystal.h> //For LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ******************************************************************


// initialize some variables
//float distance;    // (mi) needs to be changed by another function
//float targetCoords[] = {21.285625, -157.673312}; //Latitude, longitude Sandy Beach Park
//float targetCoords[] = {21.294292, -157.817138}; //Latitude, longitude Midfield
//float targetCoords[] = {21.295792, -157.817084}; //Latitude, longitude NorthEast parking lot
//float targetCoords[] = {21.295337, -157.817170}; //Latitude, longitude SouthEast parking lot
//float targetCoords[] = {21.296980, -157.816864}; //Latitude, longitude fire hydrant by Sakamaki
//float targetCoords[] = {21.296580, -157.816864}; //Latitude, longitude sign across dole
float targetCoords[] = {21.296820, -157.817440}; //Latitude, longitude fire hydrant towards law building

float currCoords[2], oldCoords[2];    // Lat/Lon readings to determine drift
float driftVect[2], targetVect[2], headingVect[2];    // dift is for current, target is from current pos to target and heading is both combined
float northVect[2] = {1.0, 0.0};
int currHead, targetHead;    // actual heading and req heading to target ccordinates
int headEpsilon = 5;    // correct course if heading is off by more than epsilon degrees
float diffHead;
bool newData;
float goalDist = 0.003788;  // radius distance mission considered successful (20ft) 1mi=5280ft 
int driftDelay = 15000;    //ms to sleep between GPS readings

float motVolt;// battery voltage to start navigation
float minVolt = 1.0;    // minimum cut-offvoltage 
int rightMotorPin = 28;
int leftMotorPin = 29;
float earthR = 3959.1;    // Earths radius (miles)
//const float pi = 3.141592;
//const float r2d = 180/PI;    // conv rads to degrees
//const float d2r = PI/80;    //conv degrees to rads


int LEDPin = 13;
bool LEDState = false;


// Initialize all programming flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeGFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)
int timeout = 2;

// initialize sensor instances
TinyGPS gps;  //instance of gps
mag myMag;    // instance of magnetometer



void setup() {
  Serial.begin(9600);  //Arduino-PC serial comm
  Serial1.begin(9600); // start GPS serial comm on Serial1
  Wire.begin();
  lcd.begin(16, 2);
  myMag.init_setup(); // turn the MAG3110 on
  myMag.define(1,1,1);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  
//  *****************CALIBRATION*****************
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
// variables for calibration loop
  float xArr[200];
  float yArr[200];
  float zArr[200];
  int xmin, xmax, ymax, ymin, zmin, zmax;
  // calibration loop
  for(int i = 0; i < 200; i++) {
    //read x, y, z values and add to array
    myMag.raw_values(&xArr[i], &yArr[i], &zArr[i]);
    
    //set initial values for all minimums and maximums
    if(i == 0){
      
      xmin = xArr[i];
      xmax = xArr[i];
      
      ymin = yArr[i];
      ymax = yArr[i];
      
      zmin = zArr[i];
      zmax = zArr[i]; 
    }
  
    //check to see if current datum is the minimum or maximum x-value
    if (xmax < xArr[i]){xmax = xArr[i];}
    if (xmin > xArr[i]){xmin = xArr[i];}
  
    //check to see if current datum is the minimum or maximum y-value
    if (ymax < yArr[i]){ymax = yArr[i];}
    if (ymin > yArr[i]){ymin = yArr[i];}
    
    //check to see if current datum is the minimum or maximum z-value
    if (zmax < zArr[i]){zmax = zArr[i];}
    if (zmin > zArr[i]){zmin = zArr[i];}
    
    if(LEDState) {
      digitalWrite(LEDPin, LOW);
      LEDState = false;
    }
    else if (!LEDState) {
      digitalWrite(LEDPin, HIGH);
      LEDState = true;
    }
    delay(100); //delay between each datum
  }
  
  lcd.setCursor(0,0);
  lcd.print("Done calibrating");
  digitalWrite(LEDPin, LOW);
  
  float xVal = 1.0/(xmax - xmin);
  float yVal = 1.0/(ymax - ymin);
  float zVal = 1.0/(zmax - zmin);
  myMag.define(xVal, yVal, zVal);
  delay(2000);
  lcd.clear();

//***************CALIBRATION*********************
}

void loop() {
  //get_drift();
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
  queue_gps(&oldCoords[0], &oldCoords[1]);
  //unsigned long age;    //needed for GPS function
  //gps.f_get_position(&oldCoords[0], &oldCoords[1], &age);
  delay(driftDelay);
  queue_gps(&currCoords[0], &currCoords[1]);
  //float latDelt = (currCoords[0] - oldCoords[0])*((earthR*2*pi)/360.0);    //formulas to calculate lat/lon deltas
  //float lonDelt = (currCoords[1] - oldCoords[1])*earthR*cos(currCoords[0]*d2r);
  vector_diff(oldCoords[0], oldCoords[1], currCoords[0], currCoords[1], &driftVect[0]);
//  Serial.print("vector_driff:");
//  Serial.println(driftVect[0]);
//  Serial.print(", ");
//  Serial.println(driftVect[0]);
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
  
//  while (motVolt > minVolt && dist2target > goalDist) {    // voltages within limit and more than 20ft from target
  while(true) {
    Serial.println("In while loop");
    queue_gps(&currCoords[0], &currCoords[1]);
    vector_diff(currCoords[0], currCoords[1], targetCoords[0], targetCoords[1], &targetVect[0]);
    headingVect[0] = targetVect[0] - driftVect[0]; headingVect[1] = targetVect[1] - driftVect[1];
    //ratio = (vector_dot(&headingVect, &northVect))/(vector_mag(&headingVect)*vector_mag(northVect));
    float angle = atan2(headingVect[0], headingVect[1]);
    if (angle < 0)
      targetHead = degrees(angle + 2*PI);
    else
      targetHead = degrees(angle);
      
//    myMag.get_values(&x, &y, &z);
    currHead = getting_heading();
//    currHead = mag.getHeading(mag.x_value(),mag.y_value(),mag.z_value());
    diffHead = targetHead - currHead;
    if (diffHead >= 0) {
      if (diffHead < 180) {turn = "left";}
    }
    if (diffHead >= -180) {
     if (diffHead <= 0) { turn = "right"; }
    } 
    if (diffHead < -180) { turn = "left"; }
    if (diffHead >= 180) {turn = "right";}
    if (abs(diffHead) <= 5) {turn == "straight";}
    if (abs(currHead) >= 355) {turn == "straight";}
    
    if (turn == "straight") {
      digitalWrite(leftMotorPin,HIGH);
      digitalWrite(rightMotorPin, HIGH);
      lcd_print('S');
    }
    if (turn == "left")
      left_turn();
    if (turn == "right")
      right_turn();
    
  //motVolt =
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
  lcd_print('R');
//  currHead = mag.getHeading(mag.x_value(),mag.y_value(),mag.z_value()); 
  currHead = getting_heading();
  right_turn();
}

/*
LEFT_TURN reads heading and rotates right motor until within headEpsilon (var) degrees from target heading
*/
void left_turn() {
  if(currHead + headEpsilon > targetHead) {
    if(currHead - headEpsilon < targetHead) {
      digitalWrite(leftMotorPin, LOW);
      digitalWrite(rightMotorPin, LOW);
      return;
    }
  }
  diffHead = targetHead - currHead;
  if(diffHead >= -180) {
    if (diffHead <=0) {
      return;
    }
  }
  if(diffHead >= 180) {
    return;
  }
  
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, HIGH);
  lcd_print('L');
  currHead = getting_heading();
//  currHead = mag.getHeading(mag.x_value(),mag.y_value(),mag.z_value());
  left_turn();
}



// *******************************LCD FUNCTIONS********************************

void clear_disp(){
  lcd.clear();
}

void lcd_print(char param) {
//  int heading;
//  lcd.setCursor(0, 0);
//  lcd.print(currCoords[0]);
//  lcd.print(", ");
//  lcd.print(currCoords[1]);
//  lcd.setCursor(0, 1);
//  heading = getting_heading();
//  lcd.print(heading);
//  lcd.setCursor(7, 1);
//  lcd.print(param);
  
  
  
  float distance;
  lcd.setCursor(0, 0);
  lcd.print("T H:");
  lcd.print(targetHead);
  lcd.print(" D:");
  distance = dist2targ();
  lcd.print(distance);
  
  lcd.setCursor(0, 1);
  lcd.print("C H:");
  if  (newData) {lcd.print(currHead);}
  else {lcd.print("NF");}
  lcd.print(" T:");
  if (param == 'R') {lcd.print("Rgt");}
  else if (param == 'L') {lcd.print("Lft");}
  else if (param == 'S') {lcd.print("Str");}
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

//*************************************WORKED***********************************




void queue_gps(float *latitude, float *longitude) {
  unsigned long age;    //needed for GPS function
  float flat, flon;
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial1.available()) {
      char c = Serial1.read(); //get GPS data
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData) {
    gps.f_get_position(&flat, &flon, &age);
    *latitude = flat;
    *longitude = flon;
  }
}


int getting_heading() {
  int heading;
  float x, y, z;
  myMag.get_values(&x, &y, &z);
  heading = myMag.get_heading(x, y);
  return heading;
}


float dist2targ() {
  float R = 3959.1*1760;  // distance in ft
  float lenDegEq = (R*2.0*PI)/(360.0);
  float lat2 = targetCoords[0];
  float lon2 = targetCoords[1];
  float lat1 = currCoords[0];
  float lon1 = currCoords[1];
  float dlon = lon2 - lon1;
  float dlat = lat2 - lat1;
  float distLon = dlon*cos(lat1)*lenDegEq;
  float distLat = dlon*lenDegEq;
  float dist = sqrt(distLon*distLon + distLat*distLat);
  return dist;
}







