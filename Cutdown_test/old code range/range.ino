/*
Fernando Aragon
4/8/15
Range sensor testing for UHABS3 parachute module

Description:
Code initializes card and writes values of range sensors into csv file.
LED_13 will be solid if SDcard works or blink if there is any problem.
Conversion factor for distance can be changed in line ##.

Wiring:
HC-SR04 Ping distance sensor
VCC to arduino 5v, GND to Arduino GND
Echo to Arduino pin 2 and 4 
Triger to Arduino pin 3 and 5
(make sure echo and triggere correspond to same sensor referring in line 34)

SD Card reader
VCC to Arduino 3.3V, GND to Arduino GND
DO to Arduino pin 50
DI to Arduino pin 51
SCK to Arduino pin 52
CS to Arduino pin 53

More info at: http://goo.gl/kJ8Gl
Original code improvements to the Ping sketch sourced from Trollmaker.com
Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
*/

#include <SD.h>
#include <SPI.h>

// give all pins reference names 
#define trigPin1 3
#define echoPin1 2
#define trigPin2 5
#define echoPin2 4
#define led 11
#define led2 10
#define cardLED 13
#define CSPin 53

// define conversion factor and delay
#define factor 148 //  58 for centimeters and 148 for inches
#define timeDelay 90 // delay between loops (ms)

// declarations
File myFile;
String dataString = "";


void setup() {
  // Start serial comm back to computer
  Serial.begin (9600);
  
  // Set mode for sensor and LED pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(cardLED, OUTPUT);
  digitalWrite(cardLED, LOW);
//  pinMode(led, OUTPUT);
//  pinMode(led2, OUTPUT);
  
  // Initialize card
  pinMode(CSPin, OUTPUT);
  
  // If card is initialized correctly LED_13 will be solid otherwise it will blink
  if (!SD.begin(CSPin)) {
    Serial.println("Card failed, or not present!");  //DEBUG
    Serial.println("Not doing anything else.");  //DEBUG
    while (true) {
      blinkLED();
    }
    return;    // stop program
  }
  Serial.println("card initialized.");  //DEBUG
  digitalWrite(cardLED, HIGH);

// Open data file
  String dataFname = "test1.csv";
  char buf[16];
  int cnt = 1;
  // Find a unique file name
  dataFname.toCharArray(buf,16);
  while (SD.exists(buf)){
//    Serial.println("File <<"+dataFname+">> exists!");
    cnt += 1;
    dataFname = "test"+String(cnt)+".csv";
    dataFname.toCharArray(buf,16);
  }
  Serial.println("Opening new logfile: "+dataFname);  //DEBUG
  Serial.println("");  //DEBUG
  myFile = SD.open(buf, FILE_WRITE);
  myFile.println("Milliseconds, Sensor1, Sensor2");
  Serial.println("Milliseconds, Sensor1, Sensor2");  //DEBUG
}


void loop() {
  read_Sensor();
  save_Data();
  delay(timeDelay);
}

/*
Blinks LED_13 if there is any problem with SDCard
*/
void blinkLED() {
  digitalWrite(cardLED, HIGH);
  delay(250);
  digitalWrite(cardLED, LOW);
  delay(250);
}

/*
Saves data string (time and sensor values) in a csv file to be viewed in excel
*/
void save_Data() {
  // if the file is available, write to it:
  if (myFile) {
    myFile.println(dataString);
    myFile.flush();
    
    // Also print string to serial port
    Serial.println(dataString);  //DEBUG
    }
    
  // if the file isn't open, pop up an error and blink LED:
  else {
    Serial.println("\nfile error - log file not available! Stopping");  //DEBUG
    while(true){
      blinkLED;
    }
  }
 dataString = ""; 
}

/*
Reads range in both sensors and creates string with these values 
*/
void read_Sensor() {
  // Initialize string with time from last reset (ms)
  dataString += String(millis());
  long duration1, distance1, duration2, distance2;
  
  // take sensor1 measurement
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  
  // take sensor2 measurement
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  
  // get distance form signal response
  // divide by 58 for centimeters and 148 for inches
  // sensors should work for 2cm - 400cm (according to manufacturer)
  distance1 = duration1 / factor;
  distance2 = duration2 / factor;
  
  // Create csv data string
  dataString += ", ";
  dataString += String(distance1);
  dataString += ", ";
  dataString += String(distance2);
  
//  WE ARE NOT DEALING WITH LEDS
  
//  if (distance1 < 200) {  // This is where the LED On/Off happens
//    digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
//  digitalWrite(led2,LOW);
//}
//  else {
//    digitalWrite(led,LOW);
//    digitalWrite(led2,HIGH);
//  }
//  if (distance1 >= 200 || distance1 <= 0){
//    Serial.println("Out of range");
//  }
//  else {
//    Serial.print(distance);
//    Serial.println(" cm");
//  }
}
