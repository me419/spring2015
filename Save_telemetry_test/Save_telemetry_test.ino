/*
Send_telemetry_test
Fernando Aragon
5/4/15 - Created

Test saving telemetry by:
  Writing telemetry string to SD card
*/

#include <SD.h>
#include <SPI.h>

//Initialize some variables
String telemetryStr = "";
File telemetryFile;
int CSPin = 53;
 char fileName[16] = "test1";
// char fileName = 'test1';


// Initialize all program flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeGFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)
int timeout = 2;


void setup() {
  Serial.begin(9600);
  sdCard_setup ();
}


void loop() {
  save_telemetry(); // put your main code here, to run repeatedly:
}


void sdCard_setup () {
 // Initialize card
  pinMode(CSPin, OUTPUT);
  
  // If card is initialized correctly LED_13 will be solid otherwise it will blink
  if (!SD.begin(CSPin)) {
    Serial.println("Card failed, or not present!");  //DEBUG
//    Serial.println("Not doing anything else.");  //DEBUG
    return;    // stop program
  }
  Serial.println("card initialized.");  //DEBUG

// Open data file
  String dataFname = "flight1.csv";
  char buf[16];
  int cnt = 1;
  
  // Find a unique file name
  dataFname.toCharArray(buf,16);
  while (SD.exists(buf)){
    
//    Serial.println("File <<"+dataFname+">> exists!");
    cnt += 1;
    dataFname = "flight"+String(cnt)+".csv";
    dataFname.toCharArray(buf,16);
  }
  Serial.println("Opening new logfile: "+dataFname);  //DEBUG
  Serial.println("");  //DEBUG
  telemetryFile = SD.open(buf, FILE_WRITE);
//  fileName = buf;
  telemetryFile.close();
}

/*
SAVE TELEMETRY saves telemetry string to SD card
*/
void save_telemetry() {
  // if the file is available, write to it:
  if (telemetryFile) {
    telemetryFile.println(telemetryStr);
    telemetryFile.flush();
    
    // Also print string to serial port
    Serial.println(telemetryStr);  //DEBUG
    }
    
  // if the file isn't open, pop up an error and blink LED:
  else {
    Serial.println("\nfile error - log file not available! Stopping");  //DEBUG
  }
 telemetryStr = ""; 
}







