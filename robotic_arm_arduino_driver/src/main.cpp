<<<<<<< HEAD
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// TODO: Winston, Legacy code for servo zero positions
// const int home0=650;
// const int home1=485;
// const int home2=285;
// const int home3=378;

#define NUM_JOINTS 4 // Change this if you add more joints

#define servoChannel0 0
#define servoChannel1 1
#define servoChannel2 2
#define servoChannel3 3

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
String incomingByte = ""; 

float jointAngles[NUM_JOINTS];
float zeroJointAngles[NUM_JOINTS] = {180.0, 134.0, 61.875, 89.25}; // Zero position angles in degrees for each servo


int16_t servoMin[NUM_JOINTS] = {175, 150, 120, 140}; // Minimum PWM values for each servo
int16_t servoMax[NUM_JOINTS] = {650, 600, 600, 620}; // Maximum PWM values for each servo
float servoRangeDeg[NUM_JOINTS] = {180.0, 180.0, 180.0, 180.0}; // Range of motion for each servo in degrees
float servoScale[NUM_JOINTS]; // Scale factors for each servo


// Compute the servo scale factors
// These are used to convert the desired angles to PWM values
// The scale factor is calculated as (max - min) / (maxAngle - minAngle)
// where max and min are the PWM limits for the servo
// and maxAngle and minAngle are the limits of the servo's range of motion
// For example, if a servo can move from 0 to 180 degrees
// and the PWM limits are 175 to 650, the scale factor would be:
// (650 - 175) / (180 - 0) = 475 / 180 = 2.64
void initializeServoScales() {
    for (int i = 0; i < NUM_JOINTS; i++) {
      // Compute the servo scale factors
      servoScale[i] = float(servoMax[i] - servoMin[i]) / servoRangeDeg[i];
    }
}

void setServoAngleDeg(float angleDeg, int16_t servoChannel) {
  int16_t cmd = int16_t((angleDeg + zeroJointAngles[servoChannel]) * servoScale[servoChannel]) + servoMin[servoChannel];
  // Serial.print("cmd: ");
  // Serial.println(cmd);
  pwm.setPWM(servoChannel, 0, cmd);
}


void printJointAngles() {
  Serial.print("Joint Angles: ");
  for (int i = 0; i < NUM_JOINTS-1; i++) {
    Serial.print(jointAngles[i]);
    Serial.print(", ");
  }
  Serial.print(jointAngles[NUM_JOINTS-1]);
  Serial.println();
}

void readsplit() {
  String fullstring = Serial.readStringUntil('\n');
  int lastIndex = 0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    int commaIndex = fullstring.indexOf(',', lastIndex);
    if (commaIndex == -1 && i < NUM_JOINTS - 1) {
      // Not enough values
      jointAngles[i] = 0;
    } else {
      String value = (i == NUM_JOINTS - 1) ? 
        fullstring.substring(lastIndex) : 
        fullstring.substring(lastIndex, commaIndex);
      jointAngles[i] = value.toFloat();
      lastIndex = commaIndex + 1;
    }
  }

  // Print the joint angles for debugging
  printJointAngles();
}


void setup() {
  initializeServoScales();
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pwm.begin();
  pwm.setPWMFreq(50);
  for (int i = 0; i < NUM_JOINTS; i++) {
    setServoAngleDeg(0.0, i);
  }
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    readsplit();
    for (int i = 0; i < NUM_JOINTS; i++) {
      setServoAngleDeg(jointAngles[i], i);
    }

    // Delay to allow the servos to move
    // This delay can be adjusted based on the speed of your servos
    delay(100);
  }
  
=======
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// TODO: Winston, Legacy code for servo zero positions
// const int home0=650;
// const int home1=485;
// const int home2=285;
// const int home3=378;

#define NUM_JOINTS 4 // Change this if you add more joints

#define servoChannel0 0
#define servoChannel1 1
#define servoChannel2 2
#define servoChannel3 3

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
String incomingByte = ""; 

float jointAngles[NUM_JOINTS];
float zeroJointAngles[NUM_JOINTS] = {180.0, 134.0, 61.875, 89.25}; // Zero position angles in degrees for each servo


int16_t servoMin[NUM_JOINTS] = {175, 150, 120, 140}; // Minimum PWM values for each servo
int16_t servoMax[NUM_JOINTS] = {650, 600, 600, 620}; // Maximum PWM values for each servo
float servoRangeDeg[NUM_JOINTS] = {180.0, 180.0, 180.0, 180.0}; // Range of motion for each servo in degrees
float servoScale[NUM_JOINTS]; // Scale factors for each servo


// Compute the servo scale factors
// These are used to convert the desired angles to PWM values
// The scale factor is calculated as (max - min) / (maxAngle - minAngle)
// where max and min are the PWM limits for the servo
// and maxAngle and minAngle are the limits of the servo's range of motion
// For example, if a servo can move from 0 to 180 degrees
// and the PWM limits are 175 to 650, the scale factor would be:
// (650 - 175) / (180 - 0) = 475 / 180 = 2.64
void initializeServoScales() {
    for (int i = 0; i < NUM_JOINTS; i++) {
      // Compute the servo scale factors
      servoScale[i] = float(servoMax[i] - servoMin[i]) / servoRangeDeg[i];
    }
}

void setServoAngleDeg(float angleDeg, int16_t servoChannel) {
  int16_t cmd = int16_t((angleDeg + zeroJointAngles[servoChannel]) * servoScale[servoChannel]) + servoMin[servoChannel];
  // Serial.print("cmd: ");
  // Serial.println(cmd);
  pwm.setPWM(servoChannel, 0, cmd);
}


void printJointAngles() {
  Serial.print("Joint Angles: ");
  for (int i = 0; i < NUM_JOINTS-1; i++) {
    Serial.print(jointAngles[i]);
    Serial.print(", ");
  }
  Serial.print(jointAngles[NUM_JOINTS-1]);
  Serial.println();
}

void readsplit() {
  String fullstring = Serial.readStringUntil('\n');
  int lastIndex = 0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    int commaIndex = fullstring.indexOf(',', lastIndex);
    if (commaIndex == -1 && i < NUM_JOINTS - 1) {
      // Not enough values
      jointAngles[i] = 0;
    } else {
      String value = (i == NUM_JOINTS - 1) ? 
        fullstring.substring(lastIndex) : 
        fullstring.substring(lastIndex, commaIndex);
      jointAngles[i] = value.toFloat();
      lastIndex = commaIndex + 1;
    }
  }

  // Print the joint angles for debugging
  printJointAngles();
}


void setup() {
  initializeServoScales();
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pwm.begin();
  pwm.setPWMFreq(50);
  for (int i = 0; i < NUM_JOINTS; i++) {
    setServoAngleDeg(0.0, i);
  }
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    readsplit();
    for (int i = 0; i < NUM_JOINTS; i++) {
      setServoAngleDeg(jointAngles[i], i);
    }

    // Delay to allow the servos to move
    // This delay can be adjusted based on the speed of your servos
    delay(100);
  }
  
>>>>>>> 513ff3647da496ab852bf29d7a10630878201a34
}