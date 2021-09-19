#include <Servo.h> 
#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0X04
#define SPEED_REG 32

volatile boolean receiveFlag = false;
char temp[32];
String command;
Servo left_grabber;
Servo right_grabber; 

const int right_yawID = 1;
const int right_pitchID = 2;
const int left_yawID = 3;
const int left_pitchID = 4; 

const float unit_degree = 0.29;
int right_grabber_pos = 2000;
int left_grabber_pos = 2000;
int fully_open = 2000;
int fully_close = 1000;
int left_fully_open = 2000;
int left_fully_close = 1000;

int delta=500;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  right_grabber.attach(3);
//  left_grabber.attach(15); 
//  left_grabber.writeMicroseconds(left_fully_open);
  right_grabber.writeMicroseconds(fully_open);
  //set_servo_position_angle(left_yawID, 150);
  delay(2000);
   
  
}
void loop()
 {
   
 if (receiveFlag == true) {
    if (temp[1] == '1'){
      String ang = temp;
      char buffer[10];
      ang.substring(2).toCharArray(buffer,10);
      float angle = atof(buffer);
      if (angle!=999)
        {
          if (temp[0] == '0')
          {
              if (angle==0)
              { 
                right_grabber.writeMicroseconds(fully_open);
                delay(100); 
              } 
              else if (angle ==1)
              { 
                right_grabber.writeMicroseconds(fully_close);
                delay(100); 
              } 
          } 
      }
    }
    receiveFlag = false;
  }
 }
 
 
 int degree_to_pwm(float degree)
 {
   return (int)round(degree/unit_degree);
 }
 
 
 float pwm_to_degree(int pwm)
 {
   return pwm*unit_degree;
 }
 
 
 void set_servo_position_angle(int servoID, float angle)
 {
   int pwm = degree_to_pwm(angle);
   //SetPosition(servoID, pwm);
 }
 
 
 float get_servo_position_angle(int servoID)
 {
   //int pwm = GetPosition(servoID);
   //return pwm_to_degree(pwm);
 }
 
 void receiveEvent(int howMany) {

  for (int i = 0; i < howMany; i++) {
    temp[i] = Wire.read();
    temp[i + 1] = '\0'; //add null after ea. char
  }

  //RPi first byte is cmd byte so shift everything to the left 1 pos so temp contains our string
  for (int i = 0; i < howMany; ++i)
    temp[i] = temp[i + 1];

  receiveFlag = true;
}
