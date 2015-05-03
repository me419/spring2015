/*
Put_TX_to_sleep_test
Fernando Aragon
4/5/15 - Created

Shut down Xtend module for to save power
*/

// initialize some variables
int txPin = 36;                  // Connects to pin 7 XTEND power/shutdown module
char txState = 'ON';        // Signals if Xtend is ON or OFF

// Initialize all programming flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)

void setup() {
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);    // power XTEND
}

// Powercycle Xtend module
void loop() {
//  if (millis() == 5000) //DEBUGGING
    put_TX_to_sleep();
}


/*
PUT_TX_TO_SLEEP shuts down Xtend module during inactive communication time (txFLAG is false)
*/
void put_TX_to_sleep() {
  digitalWrite(txPin,LOW);    // power XTEND
}
