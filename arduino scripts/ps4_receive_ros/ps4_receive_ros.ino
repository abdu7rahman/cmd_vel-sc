/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
int l, r, f, b;
ros::NodeHandle  nh;

int motor1pin1 = 34;
int motor1pin2 = 36;
int ENA = 10;
int motor2pin1 = 38;
int motor2pin2 = 40;
int ENB = 11;
int motor3pin1 = 26;
int motor3pin2 = 28;
int ENC = 8;
int motor4pin1 = 30;
int motor4pin2 = 32;
int END = 9;

void messageCb( const std_msgs::Int64MultiArray& cmd_msg) {

  l = cmd_msg.data[0];
  r = cmd_msg.data[1];
  f = cmd_msg.data[2];
  b = cmd_msg.data[3];

  if (l >= 40) {
    M1(0, 1, l);
    M2(0, 1, l);
    M3(1, 0, l);
    M4(1, 0, l);
    nh.loginfo("left");
  }
  else if (r >= 40) {
    M1(1, 0, r);
    M2(1, 0, r);
    M3(0, 1, r);
    M4(0, 1, r);
    nh.loginfo("right");
  }
  else if (f >= 40) {
    M1(1, 0, f);
    M2(0, 1, f);
    M3(0, 1, f);
    M4(1, 0, f);
    nh.loginfo("front");
  }
  else if (b >= 40) {
    M1(0, 1, b);
    M2(1, 0, b);
    M3(1, 0, b);
    M4(0, 1, b);
    nh.loginfo("back");
  }
  else {
    M1(1, 1, 255);
    M2(1, 1, 255);
    M3(1, 1, 255);
    M4(1, 1, 255);
    nh.loginfo("stop");
  }
}

ros::Subscriber<std_msgs::Int64MultiArray> sub("cmd_vel", &messageCb );

void setup()
{

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(motor3pin1, OUTPUT);
  pinMode(motor3pin2, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(motor4pin1, OUTPUT);
  pinMode(motor4pin2, OUTPUT);
  pinMode(END, OUTPUT);

  M1(1, 1, 255);
  M2(1, 1, 255);
  M3(1, 1, 255);
  M4(1, 1, 255);

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);

}
int M1(int x, int y, int z)
{
  digitalWrite(34, x);
  digitalWrite(36, y);
  analogWrite(10, z);
}
int M2(int x, int y, int z)
{
  digitalWrite(38, x);
  digitalWrite(40, y);
  analogWrite(11, z);
}
int M3(int x, int y, int z)
{
  digitalWrite(26, x);
  digitalWrite(28, y);
  analogWrite(8, z);
}
int M4(int x, int y, int z)
{
  digitalWrite(30, x);
  digitalWrite(32, y);
  analogWrite(9, z);
}
