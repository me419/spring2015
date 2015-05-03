/*
UHABS3 motor circuit test
Fernando Aragon

History:
4/14/15 - Created
*/

#define actPin 31


void setup() {
  Serial.begin(9600);
  pinMode(actPin, OUTPUT);
  digitalWrite(actPin, LOW);
}

void loop() {
  char incomingData;
  if (Serial.available()) {
    incomingData = Serial.read();
    if (incomingData == 'H')
      digitalWrite(actPin, HIGH);
    else if (incomingData == 'L')
      digitalWrite(actPin, LOW);
    else
      Serial.println("Did not recognize input!");
  }
  delay(2000);
}
