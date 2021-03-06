//The data pin for the trainer input
//const int inputPin = A3;
//The code is hard-coded for this number of channels.
//I should be able to detect this while decoding the PPM signal though
const uint8_t channels = 2;
//My trainer port output non-inverted logic signals, this may differ.
//const int level = HIGH;
int led = 13;
unsigned int offset = 400;

struct RC_Channel {
unsigned int channel[channels];
};

struct RC_Channel rc;
void setup() {
//Our wireless serial device is setup to 57.6k baud
  Serial3.begin(9600);
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
//pinMode(inputPin, INPUT);
}

//Parse the PPM input data
void readTransmitter() {
//  Serial.println("In readTransmitter...");
  char incomingByte = 0x00;
  if (Serial3.available() > 0) {
    incomingByte = Serial3.read();
    Serial.print("Received: ");
    Serial.println(incomingByte);
  }
}
//}
//Encode a frame and write it to the serial output
//Will expand this. Might move to COBS encoding
void sendData() {
//Our frame header is 0x00 0x00 because it should never come up in our data.
Serial3.write((uint8_t)0x00);
Serial3.write((uint8_t)0x00);
//Write the number of channels we will send. More important if we dynamicly set the channel count
//Serial3.write(channels);
//An int is 2 bytes long, so split it and send it over serial
Serial3.write("Hello!");
//for (int i = 0; i < channels; ++i) {
//Serial3.write(rc.channel[i] & 0xFF);
//Serial3.write((rc.channel[i] >> 8) & 0xFF);
//}
}
void loop() {
readTransmitter();
/*
for (int i = 0; i < channels; ++i) {
Serial.print("Channel ");
Serial.print(i);
Serial.print(": ");
Serial.println(rc.channel[i]);
}
*/
sendData();
delay(1000);
//Reset our channel data so in the event a read of the PPM fails we turn things off
//Right now this won't happen as the readTransmitter function is a blocking function
//This needs to be made better, and receiver side safety should be implemented as well
for (int i = 0; i < channels; ++i) {
rc.channel[i] = 0;
}
}
