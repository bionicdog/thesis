#include <Servo.h>

Servo servo; // servo controller (multiple can exist)

int servo_pin = 3; // PWM pin for servo control
int pos = 0;    // servo starting position

void setup() {
  servo.attach(servo_pin); // start servo control
}

void loop() {
  /*
  steering(0);
  delay(5000);
  steering(1);
  delay(1000);
  steering(0);
  delay(5000);
  steering(-1);
  delay(1000);
  */
}

void steering(int value) {
  switch (value) {
    case 0:
      servo.write(95); // go straight
      break;
    case 1:
      servo.write(70); // turn right
      break;
    case -1:
      servo.write(120); // turn left
      break;
  }
}
