#define USE_USBCON

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

const int encoderPin1A = 31;
const int encoderPin1B = 33;
const int encoderPin2A = 35;
const int encoderPin2B = 37;
const int encoderPin3A = 39;
const int encoderPin3B = 41;
const int encoderPin4A = 43;
const int encoderPin4B = 45;

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

// Encoder variables
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
volatile long encoderValue4 = 0;

// Robot parameters
const float wheelRadius = 0.05; // Radius of the wheel in meters
const float L = 0.26; // Distance from the center of the robot to the wheel in meters
const float encoderTicksPerRevolution = 1300; // Number of encoder ticks per wheel revolution

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

// Position and orientation
float x = 0.0;
float y = 0.0;
float theta = 0.0;

void updateEncoder1();
void updateEncoder2();
void updateEncoder3();
void updateEncoder4();
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(cmd_vel_sub);

  pinMode(encoderPin1A, INPUT);
  pinMode(encoderPin1B, INPUT);
  pinMode(encoderPin2A, INPUT);
  pinMode(encoderPin2B, INPUT);
  pinMode(encoderPin3A, INPUT);
  pinMode(encoderPin3B, INPUT);
  pinMode(encoderPin4A, INPUT);
  pinMode(encoderPin4B, INPUT);

  digitalWrite(encoderPin1A, HIGH);
  digitalWrite(encoderPin1B, HIGH);
  digitalWrite(encoderPin2A, HIGH);
  digitalWrite(encoderPin2B, HIGH);
  digitalWrite(encoderPin3A, HIGH);
  digitalWrite(encoderPin3B, HIGH);
  digitalWrite(encoderPin4A, HIGH);
  digitalWrite(encoderPin4B, HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderPin1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3B), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4A), updateEncoder4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4B), updateEncoder4, CHANGE);

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
}

void loop() {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  if (dt <= 0) {
    dt = 0.0001;
  }

  float v1 = (encoderValue1 / encoderTicksPerRevolution) * 2 * PI * wheelRadius / dt;
  float v2 = (encoderValue2 / encoderTicksPerRevolution) * 2 * PI * wheelRadius / dt;
  float v3 = (encoderValue3 / encoderTicksPerRevolution) * 2 * PI * wheelRadius / dt;
  float v4 = (encoderValue4 / encoderTicksPerRevolution) * 2 * PI * wheelRadius / dt;

  encoderValue1 = 0;
  encoderValue2 = 0;
  encoderValue3 = 0;
  encoderValue4 = 0;

  float vx = (v1 + v2 + v3 + v4) / 4.0;
  float vy = (-v1 + v2 + v3 - v4) / 4.0;
  float omega = (-v1 + v2 - v3 + v4) / (4 * L);

  x += vx * dt;
  y += vy * dt;
  theta += omega * dt;

  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;

  odom_pub.publish(&odom_msg);
  nh.spinOnce();

  delay(10);
}

void updateEncoder1() {
  static int lastEncoded1 = 0;
  int MSB = digitalRead(encoderPin1A);
  int LSB = digitalRead(encoderPin1B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded1 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1--;

  lastEncoded1 = encoded;
}

void updateEncoder2() {
  static int lastEncoded2 = 0;
  int MSB = digitalRead(encoderPin2A);
  int LSB = digitalRead(encoderPin2B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded2 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2--;

  lastEncoded2 = encoded;
}

void updateEncoder3() {
  static int lastEncoded3 = 0;
  int MSB = digitalRead(encoderPin3A);
  int LSB = digitalRead(encoderPin3B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded3 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue3++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue3--;

  lastEncoded3 = encoded;
}

void updateEncoder4() {
  static int lastEncoded4 = 0;
  int MSB = digitalRead(encoderPin4A);
  int LSB = digitalRead(encoderPin4B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded4 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue4++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue4--;

  lastEncoded4 = encoded;
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  float vx = cmd_vel_msg.linear.x;
  float vy = cmd_vel_msg.linear.y;
  float omega = cmd_vel_msg.angular.z;

  float v1 = (vx - vy - L * omega) / wheelRadius;
  float v2 = (vx + vy + L * omega) / wheelRadius;
  float v3 = (vx - vy + L * omega) / wheelRadius;
  float v4 = (vx + vy - L * omega) / wheelRadius;

  // You would send v1, v2, v3, v4 to the motor controllers here
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
