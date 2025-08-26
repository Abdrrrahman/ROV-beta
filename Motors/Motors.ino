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
  int potValueH = analogRead(potPinH);
  int escSpeedH = map(potValueH, 0, 1023, 1000, 2000);  
  escH.writeMicroseconds(escSpeedH);

  int potValueV = analogRead(potPinV);
  int escSpeedV = map(potValueV, 0, 1023, 1000, 2000);  
  escV.writeMicroseconds(escSpeedV);
  delay(10);  
}
