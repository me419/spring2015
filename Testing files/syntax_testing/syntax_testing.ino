/*
  Arduino Starter Kit example
 Project 11  - Crystal Ball
 
 This sketch is written to accompany Project 11 in the
 Arduino Starter Kit
 
 Parts required:
 220 ohm resistor
 10 kilohm resistor
 10 kilohm potentiometer
 16x2 LCD screen
 
 */
 
// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// set up a constant for the tilt switchPin
const int switchPin = 6;

// variable to hold the value of the switchPin
//int switchState = 0;

// variable to hold previous value of the switchpin
//int prevSwitchState = 0;

// a variable to choose which reply from the crystal ball
int reply;

void setup() {
  // set up the number of columns and rows on the LCD 
  lcd.begin(16, 2);
}

void loop() {
  lcd.clear();
  delay(2000);
  // set the cursor to column 0, line 0     
  lcd.setCursor(0, 0);
  // print some text
  lcd.print("Hello");
  // move the cursor to the second line
  //  lcd.setCursor(0, 1);
  delay(2000);
}


  
  
