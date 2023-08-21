#include <Servo.h>

Servo servo, esc; // servo controller (multiple can exist)

int servo_pin = 11; // PWM pin for servo control
int pos = 0;    // servo starting position

int esc_pin = 9;

void setup() {
  servo.attach(servo_pin); // start servo control
  esc.attach(esc_pin);

  set_speed(0); // set zero on ESC
  steering(0); // drive straight
  delay(100);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    // extract data
    int speedValue = getValue(data, ';', 0).toInt();
    int steeringValue = getValue(data, ';', 1).toInt();
    Serial.print(speedValue);
    Serial.print(" ");
    Serial.println(steeringValue);
    // operate kart
    set_speed(speedValue);
    steering(steeringValue);
    delay(5);
  }
  // speed test
  /*
  for (int x=-100; x <=50; x += 10) {
    set_speed(x);
    // Serial.println(x);
    delay(1000);
  }
  set_speed(0);
  delay(1000);
  */
  // steering test
  /*
  steering(0);
  delay(5000);
  steering(15);
  delay(1000);
  steering(10);
  delay(1000);
  steering(0);
  delay(5000);
  steering(-15);
  delay(1000);
  */
}

void set_speed(int valueSpeed) { // percents
  // 0 < forward < 100, -100 < backwards < 0
  // Top forward speed is on write(135), top backwards speed is on write(75), when write(90) is set as 0
  if (valueSpeed == 0) {esc.write(90);}
  else {
    int pwmValue = 0;
    if (valueSpeed > 0) {
      pwmValue = map(valueSpeed, 0, 100, 90, 135);
    }
    else if (valueSpeed < 0) {
      pwmValue = map(valueSpeed, -100, 0, 75, 90);
    }
    // Serial.print(pwmValue);
    // Serial.print("-");
    esc.write(pwmValue);
  }
}

void steering(int valueDegrees) {
  // left < 0, right > 0 in degrees
  // keep steering in between -15 en 15 °
  if (-15 > valueDegrees) {valueDegrees = -15;}
  else if (15 < valueDegrees) {valueDegrees = 15;}

  int pwmValue = map(valueDegrees, -22, 15, 45, 15);
  servo.write(pwmValue);
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
