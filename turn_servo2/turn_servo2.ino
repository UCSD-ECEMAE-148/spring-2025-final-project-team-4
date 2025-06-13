#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Current angles
float cur0 = 90, cur1 = 120, cur2 = 55, cur3 = 90, cur4 = 90;
float theta0, theta1, theta2, theta3, theta4;

// Servo channels
const int servoChannel0 = 0; // Base
const int servoChannel1 = 1; // Shoulder
const int servoChannel2 = 2; // Elbow
const int servoChannel3 = 3; // Wrist
const int servoChannel4 = 4; // Claw

// PWM limits
const int MIN_PWM = 150;
const int MAX_PWM = 650;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);

  // Initialize all servos to default position
  moveAllTo(90, 120, 55, 90, 90);
}

// Maps angle (0–180°) to PWM (150–650)
int angleToPWM(float angle) {
  return int((angle / 180.0) * (MAX_PWM - MIN_PWM) + MIN_PWM);
}

void moveAllTo(float a0, float a1, float a2, float a3, float a4) {
  pwm.setPWM(servoChannel0, 0, angleToPWM(a0));
  pwm.setPWM(servoChannel1, 0, angleToPWM(a1));
  pwm.setPWM(servoChannel2, 0, angleToPWM(a2));
  pwm.setPWM(servoChannel3, 0, angleToPWM(a3));
  pwm.setPWM(servoChannel4, 0, angleToPWM(a4));
}

void smoothMove(float t0, float t1, float t2, float t3, float t4) {
  const int steps = 30;
  for (int i = 1; i <= steps; i++) {
    moveAllTo(
      cur0 + (t0 - cur0) * i / steps,
      cur1 + (t1 - cur1) * i / steps,
      cur2 + (t2 - cur2) * i / steps,
      cur3 + (t3 - cur3) * i / steps,
      cur4 + (t4 - cur4) * i / steps
    );
    delay(33);
  }

  cur0 = t0;
  cur1 = t1;
  cur2 = t2;
  cur3 = t3;
  cur4 = t4;
}

bool parseSerialAngles() {
  String s = Serial.readStringUntil('\n');

  int c1 = s.indexOf(',');
  int c2 = s.indexOf(',', c1 + 1);
  int c3 = s.indexOf(',', c2 + 1);
  int c4 = s.indexOf(',', c3 + 1);

  if (c1 == -1 || c2 == -1 || c3 == -1 || c4 == -1) {
    Serial.println("❌ Error: Bad input format.");
    return false;
  }

  theta0 = s.substring(0, c1).toFloat();
  theta1 = s.substring(c1 + 1, c2).toFloat();
  theta2 = s.substring(c2 + 1, c3).toFloat();
  theta3 = s.substring(c3 + 1, c4).toFloat();
  theta4 = s.substring(c4 + 1).toFloat();

  Serial.print("✅ Parsed: ");
  Serial.print(theta0); Serial.print(", ");
  Serial.print(theta1); Serial.print(", ");
  Serial.print(theta2); Serial.print(", ");
  Serial.print(theta3); Serial.print(", ");
  Serial.println(theta4);

  return true;
}

void loop() {
  if (Serial.available() > 0) {
    if (parseSerialAngles()) {
      smoothMove(theta0, theta1, theta2, theta3, theta4);
    }
  }
}
