#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
String incomingByte = ""; 
float I = 0;

// PWM bounds for continuous rotation or range servos
const int STOP = 375;
const int FORWARD = 650;
const int REVERSE = 175;
const int ROTATE_360_TIME = 1200;

// Servo channel assignments
const int servoChannel0 = 0;
const int home0 = 650;

const int servoChannel1 = 1;
const int home1 = 485;

const int servoChannel2 = 2;
const int home2 = 285;

const int servoChannel3 = 3;
const int home3 = 378;

const int servoChannel4 = 4; // New grabber servo
const int home4 = 300;       // Adjust based on calibration

// Angle variables
float theta0, theta1, theta2, theta3, theta4;
float cur0 = 90, cur1 = 90, cur2 = 0, cur3 = 0, cur4 = 0;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);

  pwm.setPWM(servoChannel0, 0, home0);
  pwm.setPWM(servoChannel1, 0, home1);
  pwm.setPWM(servoChannel2, 0, home2);
  pwm.setPWM(servoChannel3, 0, home3);
  pwm.setPWM(servoChannel4, 0, home4); // Initialize grabber
}

void loop() {
  if (Serial.available() > 0) {
    readsplit();  // Parses incoming serial string into theta0–4

    for (int i = 1; i <= 30; i++) {
      // You can uncomment these if you want other servos to move too
      // SetPosch0(cur0 + (theta0 - cur0) * i / 30);
      SetPosch1(cur1 + (theta1 - cur1) * i / 30);
      // SetPosch2(cur2 + (theta2 - cur2) * i / 30);
      // SetPosch3(cur3 + (theta3 - cur3) * i / 30);
      SetPosch4(cur4 + (theta4 - cur4) * i / 30); // Move grabber

      delay(33);
    }

    cur0 = theta0;
    cur1 = theta1;
    cur2 = theta2;
    cur3 = theta3;
    cur4 = theta4;
  }
}

int SetPosch0(float angle) {
  float temp = (angle / 90) * 237.5 + 412.5;
  pwm.setPWM(servoChannel0, 0, temp);
}

int SetPosch1(float angle) {
  float temp = home1 - 225 + angle * 450 / 180;
  pwm.setPWM(servoChannel1, 0, temp);
}

int SetPosch2(float angle) {
  float temp = home2 + angle * 500 / 180;
  pwm.setPWM(servoChannel2, 0, temp);
}

int SetPosch3(float angle) {
  float temp = home3 + angle * 575 / 180;
  pwm.setPWM(servoChannel3, 0, temp);
}

int SetPosch4(float angle) {
  float temp = home4 + angle * 200 / 90; // 0=open, 90=closed
  pwm.setPWM(servoChannel4, 0, temp);
}

int readsplit() {
  String fullstring = Serial.readStringUntil('\n');

  int comma1 = fullstring.indexOf(',');
  int comma2 = fullstring.indexOf(',', comma1 + 1);
  int comma3 = fullstring.indexOf(',', comma2 + 1);
  int comma4 = fullstring.indexOf(',', comma3 + 1);

  theta0 = fullstring.substring(0, comma1).toFloat();
  theta1 = fullstring.substring(comma1 + 1, comma2).toFloat();
  theta2 = fullstring.substring(comma2 + 1, comma3).toFloat();
  theta3 = fullstring.substring(comma3 + 1, comma4).toFloat();
  theta4 = fullstring.substring(comma4 + 1).toFloat();

  Serial.print("Grabber angle: ");
  Serial.println(theta4); // Debug
}
