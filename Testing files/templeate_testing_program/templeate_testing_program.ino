/*
Send_telemetry_test
Fernando Aragon
4/23/15 - Created

Test sending telemetry to ground station by:
  Connecting to Serial port and transmitting telemetry string
*/


//Initialize some variables


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
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
OTHER FUNCTS NAMES and purpose
Tasks:  1
        2
        3
        4
*/
