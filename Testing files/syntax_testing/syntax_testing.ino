/*
Navigation program

- Read battery voltages
- Get drift
  - Read GPS coordinates
  - Wait 15 sec
  - Read GPS coordinates again
- Navigate (while batteries above cut off power)
  - Calculate target vector
  - Calculate heading vector (subtract one another to get it)
  - Calculate heading required in degrees
  - Read magnetometer heading
  - If headings offset for more than delta
    - Calculate time to thrust for rotation
    - Run single motor
- Run both motor to go straight
- Check battery voltage


*/

//#include<math.h>
//float lat1 = 21.285625;
//float lon1 = -157.673312;
//float lat2 = -21.285625;
//float lon2 = 157.673312;
//float var12 = atan(lat1/lon1);
//float var11 = atan(lat1/lon2);
//float var13 = atan(lat2/lon1);
//float var14 = atan(lat2/lon2);
//float var22 = atan2(lat1, lon1);
//float var21 = atan2(lat1, lon2);
//float var23 = atan2(lat2, lon1) + 2*PI;
//float var24 = atan2(lat2, lon2) + 2*PI;
//int var = 1;

void setup() {
  Serial.begin(9600);
}

void loop() {
  
//  Serial.println("ATAN: ");
//  Serial.print("Q1: ");
//  Serial.print(degrees(var11));
//  Serial.print("  Q2: ");
//  Serial.print(degrees(var12));
//  Serial.print("  Q3: ");
//  Serial.print(degrees(var13));
//  Serial.print("  Q4: ");
//  Serial.println(degrees(var14));
//  Serial.println("ATAN2: ");
//  Serial.print("Q1: ");
//  Serial.print(degrees(var21));
//  Serial.print("  Q2: ");
//  Serial.print(degrees(var22));
//  Serial.print("  Q3: ");
//  Serial.print(degrees(var23));
//  Serial.print("  Q4: ");
//  Serial.println(degrees(var24));
  if(Serial.available()) {
    String var = "left";
    if (var == "left")
      Serial.println("if part.");
    else
      Serial.println("else part");
  }
  delay(1000);
}




  
  
