
#include<TinyGPS.h>
#include<Wire.h>
#include <LiquidCrystal.h> //For LCD
#include "mag.h";

mag myMag;
TinyGPS gps;  //instance of gps
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// global variables
float currCoords[2];
unsigned long age;
unsigned long chars;
unsigned short sentences, failed;
float x, y, z;
int heading;
bool newData;
int LEDPin = 13;
bool LEDState = false;

//float x_scale= 1.0/(-384 + 448); //offset scale factor: 1.0/(max_x - min_x)
//float y_scale= 1.0/(330 + 497);  //offset scale factor: 1.0/(max_y - min_y)
//float z_scale= 1.0/(252 - 188);  //offset scale factor: 1.0/(max_z - min_z)


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
  lcd.begin(16, 2);
  Wire.begin();
  myMag.init_setup();
  
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
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(xmax);
  lcd.print(", ");
  lcd.print(xmin);
  lcd.print(", ");
  lcd.print(xVal);
  lcd.setCursor(0,1);
  lcd.print("Y:");
  lcd.print(ymax);
  lcd.print(", ");
  lcd.print(ymin);
  lcd.print(", ");
  lcd.print(yVal);
  delay(4000);
}

void loop() {
  queue_gps();
  myMag.get_values(&x, &y, &z);
  heading = myMag.get_heading(x, y);
  print_info();
  delay(60);
}

void queue_gps() {
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial1.available()) {
      char c = Serial1.read(); //get GPS data
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData) {
    gps.f_get_position(&currCoords[0], &currCoords[1], &age);
  }
}

void print_info() {
  lcd.clear();
  if (newData) {
    lcd.setCursor(0,0);
    lcd.print(currCoords[0]*1000000);
    lcd.setCursor(0,1);
    lcd.print(currCoords[1]*1000000);
  }
  else {
    lcd.setCursor(0,0);
    lcd.print("No fix");
  }
  lcd.setCursor(13,1);
  lcd.print(heading);
}
