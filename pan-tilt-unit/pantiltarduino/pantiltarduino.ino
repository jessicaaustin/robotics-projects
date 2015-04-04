#include <Servo.h> 
 
Servo pan_servo;
Servo tilt_servo;
int incomingByte;
 
void setup()
{
  // attach the servos and startup the serial connection
  pan_servo.attach(9);
  tilt_servo.attach(8);
  Serial.begin(9600);
  resetAll();
}
 
void loop()
{
  
  // check to see if something was sent via the serial connection
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
   
    // move the servos based on the byte sent
    if (incomingByte == 'e') {
      moveServo(tilt_servo, 1);
    } else if (incomingByte == 'x') {
      moveServo(tilt_servo, -1);
    } else if (incomingByte == 'd') {
      moveServo(pan_servo, 1);
    } else if (incomingByte == 's') {
      moveServo(pan_servo, -1);
    } else if (incomingByte == 'r') {
      resetAll();
    } else if (incomingByte == '/') {
      Serial.println("pan: " + pan_servo.read());
      Serial.println("tilt: " + tilt_servo.read());
    }
  }

}
 
// move the servo a given amount
void moveServo(Servo servo, int delta) {
  int previousValue = servo.read();
  int newValue = previousValue + delta;
  if (newValue > 180 || newValue < 30) {
    return;
  }
  servo.write(newValue);
}
 
// put the servos back to the "default" position
void resetAll() {
  reset(pan_servo);
  reset(tilt_servo);
}
 
// put a servo back to the "default" position (100 deg)
void reset(Servo servo) {
  
  int newPos = 130;
  int previousPos = servo.read();
  if (newPos > previousPos) {
    for (int i=previousPos; i<newPos; i++) {
      servo.write(i);
      delay(15);
    }
  } else {
    for (int i=previousPos; i>newPos; i--) {
      servo.write(i);
      delay(15);
    }
  }
  
}
