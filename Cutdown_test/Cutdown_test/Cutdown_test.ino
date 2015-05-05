/*
me419 astronautics spring 2015
Ultrasonic Sensor HC-SR04 based cutdown code

When called, Cutdown_sequence will begin a 3 minute countdown timer.
For the duration of the timer, the HC-SR04 Ultrasonic rangefinder will be 
activated and measure distance to ground; if the distance is beyond the 
range of the sensor, 0 is returned for the distance. At each step a numerical
derivative is calculated to determine velocity and if velocity falls within a 
predetermined range for 3 consecutive datapoints then a cutdown circuit
on pin 0 is activated and the function will return True. If the proper velocity
is not found during the 3 minutes, the cutdown circuit will be activated and
the function will return False.

*/


boolean Cutdown_sequence() {
  int trigPin = 3;  //Trig
  int echoPin = 2;  //Echo
  int cutPin = 30;  //Cutdown
  int count = 0;
  long duration;  //Measured duration in ms
  long dist,prevdist,time,prevtime; //Variables for determining velocity
  float velocity; //Velocity in m/s
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(cutPin, OUTPUT);
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
 
    velocity = (float(dist) - float(prevdist))/(float(time) - float(prevtime))*-10.0; //Velocity in m/s
    
    if(velocity > 3.5  && velocity < 7.0) { // Check if velocity is in range
      if(count > 0){      //Check if previous velocity was in range
        digitalWrite(cutPin, HIGH);
        delay(5000);
        digitalWrite(cutPin, LOW);
        return true;
      }
      else{count += 1;}      //If previous was not in range, increment count
    }
    else{count = 0;}    //If velocity is not in range, reset count
    delay(25);
  }
  digitalWrite(cutPin, HIGH);
  delay(5000);
  digitalWrite(cutPin, LOW);
  return false;
}

void setup(){}
void loop(){}
