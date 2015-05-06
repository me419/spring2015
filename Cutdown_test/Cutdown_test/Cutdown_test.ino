/*
  Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when pressing a pushbutton attached to pin 2. 
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/Button
 */

#include <Servo.h>

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 28;     // the number of the pushbutton pin
int buttonState = 0; 
boolean ran = false;

void setup() {
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
}

boolean Cutdown_sequence() {
  int trigPin = 3;  //Trig
  int echoPin = 2;  //Echo
  int cutPin = 6;  //Cutdown
  long timerMinutes = 1;
  
  int count = 0;
  long duration;  //Measured duration in ms
  long initTime = millis();
  long mfkldsfkls;
  long dist,prevdist,time,prevtime; //Variables for determining velocity
  float velocity; //Velocity in m/s
  
  Servo myservo;
  myservo.attach(cutPin);
  myservo.write(65);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(cutPin, OUTPUT);
  initTime = millis();
  time = millis();
  dist = 0;
  Serial.println(initTime);
  Serial.println(time);
  Serial.println(time-initTime);
  
  //while(true){
  while((time-initTime) < long(1000)*long(60)*long(timerMinutes)){
    //Serial.println(time);
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
 
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH, 9000);
 
    prevdist = dist;
    prevtime = time;
    time = millis();
    dist = (duration/2) / 29.1; //distance in Centimeters
    
    Serial.print(dist);
    Serial.print(" ");
    Serial.println(velocity);
    
    velocity = (float(dist) - float(prevdist))/(float(time) - float(prevtime))*-10.0; //Velocity in m/s
    
    if(velocity > 3.5  && velocity < 7.0) { // Check if velocity is in range
      if(count > 0){      //Check if previous velocity was in range
        myservo.write(100);
        delay(1500);
        myservo.write(65);
        return true;
      }
      else{count += 1;}      //If previous was not in range, increment count
    }
    else{count = 0;}    //If velocity is not in range, reset count
    delay(25);
  }
  myservo.write(100);
  delay(1500);
  myservo.write(60);
  return false;
}

void loop(){
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  
  

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {} 
  else {
    if(ran == false){
      Serial.println("Time");
      Cutdown_sequence();
      ran = true;
    }
  }
}
