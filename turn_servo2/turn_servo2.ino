#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
String incomingByte = ""; 
float I=0;
const int STOP = 375;
const int FORWARD = 650;
const int REVERSE = 175;
const int ROTATE_360_TIME = 1200;  // Adjust this for your servos

const int servoChannel0 = 0;//PWM signals min 175 max(forward) 650 180 degrees 
const int home0=650;
const int servoChannel1 = 1;//PWM min 150 max 600 UP/home:485 lower signals move arm down
const int home1=485;
const int servoChannel2 = 2;//PWM min 150 max 650 UP/home:285 higher signals move arm down
const int home2=285;
const int servoChannel3 = 3;//PWM min 140 615 up 378. higher moves up I think havnt checked
const int home3=378;
float theta0;
float theta1;
float theta2;
float theta3;
float cur0=90;
float cur1=90;
float cur2=0;
float cur3=0;
//float angles[4];
   void spinOnce(int channel, int directionPWM, const char* message);

    void setup() {
      Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
      pwm.begin();
      pwm.setPWMFreq(50);
      pwm.setPWM(servoChannel0, 0, home0);
      pwm.setPWM(servoChannel1, 0, home1);
      pwm.setPWM(servoChannel2, 0, home2);
      pwm.setPWM(servoChannel3, 0, home3);
    }

    void loop() {

      // send data only when you receive data:
      if (Serial.available() > 0) {
        // read the incoming byte:
        
      readsplit();//changes theta 0 1 2 3 to be the ones in the string should be formated "float0,float1,float2,float3"
      Serial.println(theta1);
        for(int i=1; i<=30; i++){
       //SetPosch0(cur0+(theta0-cur0)*i/30);
       SetPosch1(theta1);//(cur1+(theta1-cur1)*i/30);
       //SetPosch2(cur2+(theta2-cur2)*i/30);
       //SetPosch3(cur3+(theta3-cur3)*i/30);
       delay(33);
        }
        cur0=theta0;
        cur1=theta1;
        cur2=theta2;
        cur3=theta3;
      }
       // I=Serial.read();
        // say what you got:
    }
    
    
void spinOnce(int channel, int directionPWM, const char message) {
  Serial.println(message);
  pwm.setPWM(channel, 0, directionPWM);
  delay(ROTATE_360_TIME);
  // Uncomment the line below to stop the motor after each command
  // pwm.setPWM(channel, 0, STOP);
}

int SetPosch0(float angle) {
float temp;
    temp=(angle/90)*237.5+412.5;//theta0=(atanf(y/x)/(PI*.5))*237.5+412.5;
    //theta1=260+((atan(z/sqrt(x*x+y*y))+acos((r*r+L1*L1-L2*L2)/(2*L1*r))))*450/PI;
    // theta2=285-(acos((L1*L1+L2*L2-r*r)/(2*L1*L2))-PI)*500/PI;
   // pwm.setPWM(servoChannel0, 0, theta0);
    //pwm.setPWM(servoChannel1, 0, theta1);
pwm.setPWM(servoChannel0, 0, temp);
}

int SetPosch1(float angle) {
float temp=0;
    //theta0=(atanf(y/x)/(PI*.5))*237.5+412.5;
   temp=home1-225+angle*450/180; //theta1=260+((atan(z/sqrt(x*x+y*y))+acos((r*r+L1*L1-L2*L2)/(2*L1*r))))*450/PI;
    // theta2=285-(acos((L1*L1+L2*L2-r*r)/(2*L1*L2))-PI)*500/PI;
   // pwm.setPWM(servoChannel0, 0, theta0);
    //pwm.setPWM(servoChannel1, 0, theta1);
pwm.setPWM(servoChannel1, 0, temp);
}

int SetPosch2(float angle) {
float temp;
    //theta0=(atanf(y/x)/(PI*.5))*237.5+412.5;
    //theta1=260+((atan(z/sqrt(x*x+y*y))+acos((r*r+L1*L1-L2*L2)/(2*L1*r))))*450/PI;
temp=home2+(angle)*500/180;// theta2=285-(acos((L1*L1+L2*L2-r*r)/(2*L1*L2))-PI)*500/PI;
   // pwm.setPWM(servoChannel0, 0, theta0);
    //pwm.setPWM(servoChannel1, 0, theta1);
pwm.setPWM(servoChannel2, 0, temp);
}

int SetPosch3(float angle) {
float temp;
    //theta0=(atanf(y/x)/(PI*.5))*237.5+412.5;
    //theta1=260+((atan(z/sqrt(x*x+y*y))+acos((r*r+L1*L1-L2*L2)/(2*L1*r))))*450/PI;
temp=home3+angle*575/180;// theta2=285-(acos((L1*L1+L2*L2-r*r)/(2*L1*L2))-PI)*500/PI;
   // pwm.setPWM(servoChannel0, 0, theta0);
    //pwm.setPWM(servoChannel1, 0, theta1);
pwm.setPWM(servoChannel3, 0, temp);
}

int readsplit(){
 String fullstring="";
 fullstring=Serial.readStringUntil("/n");
 int comma1=fullstring.indexOf(',');
 int comma2 = fullstring.indexOf(',', comma1 + 1);
 int comma3 = fullstring.indexOf(',', comma2 + 1);
theta0 = fullstring.substring(0,comma1).toFloat();
theta1 = fullstring.substring(comma1+1).toFloat();
theta2 = fullstring.substring(comma2+1).toFloat();
theta3 = fullstring.substring(comma3+1).toFloat();
Serial.println(theta1);
 //Serial.println(theta0);
 
 //Serial.println(theta2);
 //Serial.println(theta3);
 }
