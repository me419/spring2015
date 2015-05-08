/*
Check_for_request_test
Fernando Aragon
3/4/15 - Created

Test receiving ground communication transmission by:
  Reading incoming ground request
  Change request flag
  Change target location coordinates (for nav phase)
  Change abort flag
  Change time of transmission
*/

#include <string.h>

// initialize some variables
float targetCoords[] = {21.285625, -157.673312}; //Latitude, longitude Sandy Beach Park
int transmissionTime[] = {900, 1000};        // 9am - 10am
int txPin = 36;                  // Connects to pin 7 XTEND power/shutdown module

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
  Serial3.begin(9600);    // Open serial port to XTEND (Connected to serial 3)
  Serial.begin(9600);    // Open serial pc comm port for debugging
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);    // power XTEND
}

void loop() {
  check_for_request(timeout);
//  Serial.print(targetCoords[0],6);  //DEBUGGING
//  Serial.print(" | ");  //DEBUGGING
//  Serial.print(targetCoords[1],6);  //DEBUGGING
//  Serial.print(" | ");  //DEBUGGING
//  Serial.print(transmissionTime[0]);  //DEBUGGING
//  Serial.print(" | ");  //DEBUGGING
//  Serial.print(transmissionTime[1]);  //DEBUGGING
//  Serial.print(" | ");  //DEBUGGING
//  Serial.println(abortFLAG);  //DEBUGGING
//  delay(2000);  //DEBUGGING
}



/*
CHECK_FOR_REQUEST retrieves and parses message from ground.
Tasks:  Read incoming ground request
        Change request flag
        Change target location coordinates (for nav phase)
        Change abort flag
        Change time of transmission
*/
void check_for_request(int secTimeOut) {
//  Serial.println("Inside check_for_request()...");  //DEBUGGING
  // flow control variables
  bool beg = false;
  bool fin = false;
  bool thru = false;
  bool sep = false;
  bool save = false;
  char current;
  
  // temporary variables
  String lat, lon, startTime, endTime, abortMsg;
  
  int time = millis();
  while (Serial3.available()) {
    if (secTimeOut*2000 < millis() - time) {
//      Serial.println("Timed out...");  //DEBUGGING
      break;
    }
    char incomingByte = Serial3.read();
//    Serial.println(incomingByte);  //DEBUGGING
    if (incomingByte == '<') {
      thru = true;
      beg = true;
      lat = "";
      lon = "";
      startTime = "";
      endTime = "";
      abortMsg = "";
      continue;
    }
    if (incomingByte == '>') {
//      Serial.println("\n");  //DEBUGGING
      thru = false;
      fin = true;
      if (beg && fin) {
        save = true;
        break;
      }
    }
    if (incomingByte == ',') {
      sep = true;
      continue;
    }
    if (incomingByte == '$') {
      current = Serial3.read();
      sep = false;
//      Serial.println(current);  //DEBUGGING
      continue;
    }
    if (thru) {
      if (current == 'L' && sep) {
//        Serial.println(" adding to lon");  //DEBUGGING
        lon = String(lon + incomingByte);
      }
      else if (current == 'L') {
//        Serial.println(" adding to lat");  //DEBUGGING
        lat = String(lat + incomingByte);
      }
      else if (current == 'A') {
//        Serial.println(" adding to abort");  //DEBUGGING
        abortMsg = String(abortMsg + incomingByte);
      }
      else if (current == 'T' && sep) {
//        Serial.println(" adding to endTime");  //DEBUGGING
        endTime = String(endTime+ incomingByte);
      }
      else if (current == 'T') {
//        Serial.println(" adding to startTime");  //DEBUGGING
        startTime = String(startTime + incomingByte);
      }
    }
//    delay(1000);  //DEBUGGING
  }
  if (save) {
    if (targetCoords[0] != lat.toFloat())
      targetCoords[0] = lat.toFloat();
    if (targetCoords[1] != lon.toFloat())
      targetCoords[1] = lon.toFloat();
    if (abortFLAG != (bool)abortMsg)
      abortFLAG = (bool)abortMsg;
    if (transmissionTime[0] != startTime.toInt())
      transmissionTime[0] = startTime.toInt();
    if (transmissionTime[1] != endTime.toInt())
      transmissionTime[1] = endTime.toInt();
      
      
    Serial.print("Printing lat: ");  //DEBUGGING
    Serial.println(lat);   //DEBUGGING
    Serial.print("Printing lon: ");  //DEBUGGING
    Serial.println(lon);   //DEBUGGING
    Serial.print("Printing abort: ");  //DEBUGGING
    Serial.println(abortMsg);   //DEBUGGING
    Serial.print("Printing startTime: ");  //DEBUGGING
    Serial.println(startTime);   //DEBUGGING
    Serial.print("Printing endTime: ");  //DEBUGGING
    Serial.println(endTime);   //DEBUGGING
  }
  
}
