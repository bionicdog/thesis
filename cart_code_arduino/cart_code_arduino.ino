#include <Servo.h>

Servo servo, esc; // servo controller (multiple can exist)

int servo_pin = 9; // PWM pin for servo control
int pos = 0;    // servo starting position

int esc_pin = 11;

void setup() {
  servo.attach(servo_pin); // start servo control
  esc.attach(esc_pin);

  esc.write(95);
  servo.write(95);
  delay(5000);
}

void loop() {
  //*
  esc.write(95);
  steering(0);
  delay(5000);
  steering(1);
  delay(1000);
  steering(0);
  delay(5000);
  steering(-1);
  delay(1000);
  //*/
  /*
  servo.write(95);
  delay(1000);
  servo.write(70);
  delay(1000);
  servo.write(120);
  delay(1000);
  servo.write(95);
  delay(5000);
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
