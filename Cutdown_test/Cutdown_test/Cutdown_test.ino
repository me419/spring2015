/*
me419 astronautics spring 2015
Ultrasonic Sensor HC-SR04 based cutdown code




*/


boolean Cutdown_sequence() {
  int trigPin = 3;    //Trig - green Jumper
  int echoPin = 2;    //Echo - yellow Jumper
  long duration;
  long dist,prevdist,time,prevtime;
  float velocity;
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  time = millis();
  dist = 0;
  
  while(time < (1000*60*3)){
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
 
    velocity = (float(dist) - float(prevdist))/(float(time) - float(prevtime))*-10.0;
    
    if(velocity > 3.5  && velocity < 7.0) {
      Serial.print(velocity);
      Serial.println(); 
    }
    
    delay(200);
  }
  
}
