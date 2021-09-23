#include <Servo.h> 
#include <Wire.h>
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif


#define SLAVE_ADDRESS 0X04
#define SPEED_REG 32
const float DXL_PROTOCOL_VERSION = 1.0;

DynamixelShield dxl;
using namespace ControlTableItem;

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
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(right_yawID);
  dxl.ping(right_pitchID);
  dxl.ping(left_yawID);
  dxl.ping(left_pitchID);
 
  dxl.torqueOff(right_yawID);
  dxl.torqueOff(right_pitchID);
  dxl.torqueOff(left_yawID);
  dxl.torqueOff(left_pitchID);
  
  dxl.setOperatingMode(right_yawID, OP_POSITION);
  dxl.setOperatingMode(right_pitchID, OP_POSITION);
  dxl.setOperatingMode(left_yawID, OP_POSITION);
  dxl.setOperatingMode(left_pitchID, OP_POSITION);
  
  dxl.torqueOn(right_yawID);
  dxl.torqueOn(right_pitchID);
  dxl.torqueOn(left_yawID);
  dxl.torqueOn(left_pitchID);

  dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, right_yawID, 100);
  dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, right_pitchID, 100);
  dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, left_yawID, 100);
  dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, left_pitchID, 100);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  right_grabber.attach(3);
  left_grabber.attach(4); 
  left_grabber.writeMicroseconds(fully_open);
  delay(3000);
  left_grabber.writeMicroseconds(fully_close); 
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
          else {
            if (angle==0.0)
              {
                left_grabber.writeMicroseconds(left_fully_open);
                delay(100);
              } 
              else if (angle ==1.0)
              {
                left_grabber.writeMicroseconds(left_fully_close);
                delay(100);
              } 
          }
      }
    }
    else if (temp[1] == '2'){
      String ang = temp;
      char buffer[10];
      ang.substring(2).toCharArray(buffer,10);
      float angle = atof(buffer);
      if (angle!=999)
      {
        if (temp[0]=='0')
          dxl.setGoalPosition(right_yawID, angle, UNIT_DEGREE);
        else
          dxl.setGoalPosition(left_yawID, angle, UNIT_DEGREE);
      }
    }
    else if (temp[1] == '3'){
      String ang = temp;
      char buffer[10];
      ang.substring(2).toCharArray(buffer,10);
      float angle = atof(buffer);
      if (angle!=999)
       {
         if (temp[0]=='0')
            dxl.setGoalPosition(right_pitchID, angle, UNIT_DEGREE);
         else
            dxl.setGoalPosition(left_pitchID, angle, UNIT_DEGREE);
       }
    }
    
    receiveFlag = false;
  }
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
