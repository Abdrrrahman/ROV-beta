#include <Servo.h>

Servo escH;
Servo escV;
int potPinH = A5;
int potPinV = A4;

void setup() {
  escV.attach(9);
  escH.attach(10);
  escV.writeMicroseconds(1000);  
  escH.writeMicroseconds(1000);  
  delay(3000);  
}

void loop() {
  // We read the inputed data throught the pot
  int potValueH = analogRead(potPinH);
  int potValueV = analogRead(potPinV);
  
  // if the number is between 1500 and 2000, it will go forward, making 1500 the stop point and 2000
  // the heighest speed
  // if the number is between 1500 and 1000, it will go backward, making 1500 the stop point and 1000
  // the heighest speed
  int escSpeedH = map(potValueH, 0, 1023, 1000, 2000);  
  int escSpeedV = map(potValueV, 0, 1023, 1000, 2000);
  escH.writeMicroseconds(escSpeedH);
  escV.writeMicroseconds(escSpeedV);

  // some delay to stop overusing
  delay(10);  
}
