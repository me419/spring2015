/*
An Arduino code example for interfacing with the HMC5883

by: Jordan McConnell
 SparkFun Electronics
 created on: 6/30/11
 license: OSHW 1.0, http://freedomdefined.org/OSHW

Analog input 4 I2C SDA
Analog input 5 I2C SCL
*/

#include <Wire.h> //I2C Arduino Library
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#define address 0x1E //0011110b, I2C 7bit address of HMC5883


float x_scale= 1.0/(-384 + 448); //offset scale factor: 1.0/(max_x - min_x)
float y_scale= 1.0/(330 + 497);    //offset scale factor: 1.0/(max_y - min_y)
float z_scale= 1.0/(252 - 188);  //offset scale factor: 1.0/(max_z - min_z)


void setup(){
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  lcd.begin(16, 2);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void loop(){
  lcd.clear();
  float head;  // heading
  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  lcd.setCursor(0, 0);
  //Print out values of each axis
//  Serial.print("x: ");
  lcd.print(x*x_scale);
//  Serial.print(x);
  lcd.print(", ");
//  Serial.print("  y: ");
  lcd.print(y*y_scale);
//  Serial.print(y);
  lcd.print(", ");
//  Serial.print("  z: ");
  lcd.print(z*z_scale);
//  Serial.println(z);
  head = measure_heading(x, y);
  lcd.setCursor(0, 1);
  lcd.print("heading: ");
  lcd.print(head);
  
  delay(200);
}

int measure_heading(int x, int y) {
  
  float heading=0;  
  
  //defines value of heading for various x,y,z values
  if ((x == 0)&&(y < 0))  
    heading= PI/2.0; 
     
  if ((x == 0)&&(y > 0))  
    heading=3.0*PI/2.0; 
     
  if (x < 0)  
    heading = PI - atan(y/x);  
    
  if ((x > 0)&&(y < 0))  
    heading = -atan(y/x);
    
  if ((x > 0)&&(y > 0))  
    heading = 2.0*PI - atan(y/x); 
     
  return  int(degrees(heading));  //convert heading from radians to degrees and return
}
