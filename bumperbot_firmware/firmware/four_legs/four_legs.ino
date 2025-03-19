#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create a servo driver object
Adafruit_PWMServoDriver pwm= Adafruit_PWMServoDriver();

// Define the minimum and maximum pulse width for your servo (adjust according to your servo's specification)
#define SERVOMIN  125 // Minimum pulse length (microseconds)
#define SERVOMAX  600 // Maximum pulse length (microseconds)

// Define the number of the servo channel you want to control
#define SERVO_0 12 // Example channel
#define SERVO_1 13
#define SERVO_2 14
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

   
   if (msg[0] == 'm' && msg[1] == 'o'){
    int th9, th10, th11, th3, th4, th5, th6, th7, th8, th0, th1, th2;
    msg.remove(0,3);
    int spaceindex = msg.indexOf('_');
    String str9 = msg.substring(0, spaceindex); //th6 in string
    int next_space = msg.indexOf('_', spaceindex+1); 
    String str10 = msg.substring(spaceindex+1,next_space); //th7 in string
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str11 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str3 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str4 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str5 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str6 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str7 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str8 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str0 = msg.substring(spaceindex+1,next_space);
    spaceindex = next_space;
    next_space = msg.indexOf('_', spaceindex+1);
    String str1 = msg.substring(spaceindex+1,next_space);
    String str2 = msg.substring(next_space+1,msg.end());

    th9 = str9.toInt();
    th10 = str10.toInt();
    th11 = str11.toInt();
    th3 = str3.toInt();
    th4 = str4.toInt();
    th5 = str5.toInt();
    th6 = str6.toInt();
    th7 = str7.toInt();
    th8= str8.toInt();
    th0 = str0.toInt();
    th1 = str1.toInt();
    th2 = str2.toInt();

    // sscanf only works with c-strings, so convert String to char array
    // sscanf(msg.c_str(), "rb_%d_%d_%d", &th6, &th7, &th8);
    //************th11 negative diyo********

    pwm.setPWM(SERVO_9,0,val(-th9));
    pwm.setPWM(SERVO_10,0,val(th10)-20);
    pwm.setPWM(SERVO_11,0,val(th11)-15);

    pwm.setPWM(SERVO_6,0,val(-th6));
    //send angle of servo motor 1 of right back leg as negative and then give!**********
    pwm.setPWM(SERVO_7,0,val(th7)-20);
    pwm.setPWM(SERVO_8,0,val(th8)+20);

    pwm.setPWM(SERVO_3,0,val(th3)-10);
    pwm.setPWM(SERVO_4,0,val2(th4)+50);
    pwm.setPWM(SERVO_5,0,val(th5)+20);

    pwm.setPWM(SERVO_0,0,val(th0)-45);
    pwm.setPWM(SERVO_1,0,val(th1));
    pwm.setPWM(SERVO_2,0,val(-th2)+40);

   }
  }

  //   pwm.setPWM(SERVO_0,0,val(0)-20);
  //   delay(800);
  //   pwm.setPWM(SERVO_1,0,val(0)+25);
  //   pwm.setPWM(SERVO_2,0,val(90)+30);
  // // right forward done *********
  // pwm.setPWM(SERVO_6,0,val(0));
  // //send angle of servo motor 1 of right back leg as negative and then give!**********

  // pwm.setPWM(SERVO_7,0,val(0)+4);
  // pwm.setPWM(SERVO_8,0,val(90)+20);

  // pwm.setPWM(SERVO_9,0,val(0));
  // pwm.setPWM(SERVO_10,0,val(0));
  // pwm.setPWM(SERVO_11,0,val(90));

  // pwm.setPWM(SERVO_3,0,val(0));
  // pwm.setPWM(SERVO_4,0,val2(0));
  // pwm.setPWM(SERVO_5,0,val(90));

  // delay(0.1);
}
