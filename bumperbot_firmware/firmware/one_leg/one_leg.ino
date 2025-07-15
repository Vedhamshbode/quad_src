#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create a servo driver object
Adafruit_PWMServoDriver pwm= Adafruit_PWMServoDriver();

// Define the minimum and maximum pulse width for your servo (adjust according to your servo's specification)
#define SERVOMIN  125 // Minimum pulse length (microseconds)
#define SERVOMAX  600 // Maximum pulse length (microseconds)

// Define the number of the servo channel you want to control
#define SERVO_0 3 // Example channel
#define SERVO_1 4
#define SERVO_2 5
#define SERVO_3 6
#define SERVO_4 7
#define SERVO_5 8
#define SERVO_6 0
#define SERVO_7 1
#define SERVO_8 2
#define SERVO_9 9
#define SERVO_10 10
#define SERVO_11 11

int val(int deg){
  int value=map(deg,-90,90,SERVOMIN,SERVOMAX);
  Serial.println(value);
  return value;
}
int val2(int deg){
  int value=map(deg,-90,90,val(-90)+80,val(90)-80);
  Serial.println(value);
  return value;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial.setTimeout(1);

  pwm.begin();
  pwm.setPWMFreq(60);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
   String msg = Serial.readStringUntil('\r');
    msg.trim();  // Remove newlines and spaces

   
   if (msg[0] == 'r' && msg[1] == 'b'){
    int th6, th7, th8;
    msg.remove(0,3);
    int spaceindex = msg.indexOf('_');
    String val1s = msg.substring(0, spaceindex); //th6 in string
    int next_space = msg.indexOf('_', spaceindex+1); 
    String val2s = msg.substring(spaceindex+1,next_space); //th7 in string
    String val3s = msg.substring(next_space+1,msg.end());
    th6 = val1s.toInt();
    th7 = val2s.toInt();
    th8 = val3s.toInt();

    // sscanf only works with c-strings, so convert String to char array
    // sscanf(msg.c_str(), "rb_%d_%d_%d", &th6, &th7, &th8);

    pwm.setPWM(SERVO_6,0,val(-th6));
    //send angle of servo motor 1 of right back leg as negative and then give!**********
    pwm.setPWM(SERVO_7,0,val(th7)+4);
    pwm.setPWM(SERVO_8,0,val(th8)+10);
   }
  }
  // delay(0.1);
}
