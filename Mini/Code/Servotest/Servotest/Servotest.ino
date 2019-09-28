#include <Servo.h>

const char PIN_FOOTPRINT_LEFT_SERVO   = 10;  // PWM      Footplate, Servo, Left, Position, 1ms..2ms=-90°..+90°
const char PIN_FOOTPRINT_RIGHT_SERVO  = 9;  // PWM      Footplate, Servo, Right, Position, 1ms..2ms=-90°..+90°

Servo sServoLeft;                   // Servo of the left footprint
Servo sServoRight;                  // Servo of the right footprint

int iPosition;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  sServoLeft.attach (PIN_FOOTPRINT_LEFT_SERVO);
  sServoLeft.writeMicroseconds (1500);
  sServoRight.attach (PIN_FOOTPRINT_RIGHT_SERVO);
  sServoRight.writeMicroseconds (1500);
  //sServoLeft.write (90);
  //sServoRight.write (90);

}

void loop() {
  // put your main code here, to run repeatedly:
  iPosition+=1;
  if (iPosition>100)
  iPosition=0;
  delay (10);
  Serial.println (iPosition);
  //sServoLeft.writeMicroseconds (1500+iPosition);
  //sServoRight.writeMicroseconds (1500+iPosition);
  //sServoLeft.write (90+iPosition);
  //sServoRight.write (90+iPosition);
  
}
