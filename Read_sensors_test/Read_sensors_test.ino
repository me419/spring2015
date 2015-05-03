/*
Read_sensors_test
Fernando Aragon
4/4/15 - Created

Read all the sensor values and update current module state:
  Read air pressure from weather shield
  Read time, position (latitude, longitude, altitude) from GPS
  Calculate velocity
  Read internal/external temperature
  Read attitute(roll, pitch, yaw) from IMU
  Calculate attitute rates
  Measure bus currents/voltages
  Update current state of the module with all readings
*/

#include <string.h>

// initialize some variables
float airPress, lat, lon, vel, intTemp, extTemp, mcuVolt, vCamVolt, txVolt;
unsigned long time, date;
//INITIALIZE VARIABLES FOR IMU  --> ROLL, PITCH, YAW and each rate.


// Initialize all programming flags
bool requestFLAG = false;           // signals there's request pending
bool navSetUpFLAG = false;          // signals parachute released (lasts predetermined time)
bool txFLAG = false;               // signals if it is transmission time 
bool abortFLAG = false;            //signals if aborting sequence begins
bool lowAltitudeFLAG= false;      // signals if modeul is under predetermined altitude
bool recordingFLAG = false;        // signals when is time to take picture
int fligthtModeFLAG = 1;          // signals current operation mode (Ascent/Flight/Descent/Nav)


void setup() {
  Serial.begin(9600);  //DEBUGGING
}

void loop() {
  read_sensors();
}


/*
READ_SENSORS reads all the sensor values and update current module state.
Tasks: Read air pressure from weather shield
       Read time, position (latitude, longitude, altitude) from GPS
       Calculate velocity
       Read internal/external temperature
       Read attitute(roll, pitch, yaw) from IMU
       Calculate attitute rates
       Measure bus currents/voltages
       Update current state of the module with all readings
*/
void read_sensors() {
}
