#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// ROS NodeHandle
ros::NodeHandle nh;

// Encoder pin definitions
const int encoderPin1A = 31;
const int encoderPin1B = 33;
const int encoderPin2A = 37;
const int encoderPin2B = 35;
const int encoderPin3A = 39;
const int encoderPin3B = 41;
const int encoderPin4A = 45;
const int encoderPin4B = 43;

// Motor pin definitions
const int motor1pin1 = 34;
const int motor1pin2 = 36;
const int ENA = 10;
const int motor2pin1 = 38;
const int motor2pin2 = 40;
const int ENB = 11;
const int motor3pin1 = 26;
const int motor3pin2 = 28;
const int ENC = 8;
const int motor4pin1 = 30;
const int motor4pin2 = 32;
const int END = 9;

// Wheel and robot dimensions
const float wheelRadius = 0.05; // meters
const float robotRadius = 0.266; // meters
const int pulsesPerRevolution = 1300; // Encoder pulses per wheel revolution

// Encoder variables
volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
volatile long encoderValue4 = 0;

long lastEncoderValue1 = 0;
long lastEncoderValue2 = 0;
long lastEncoderValue3 = 0;
long lastEncoderValue4 = 0;

// Odometry variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;

// Velocity commands
float linearX = 0.0;
float linearY = 0.0;
float angularZ = 0.0;

// Define functions to set motor direction and speed
void M1(int dir1, int dir2, int speed) {
  digitalWrite(motor1pin1, dir1);
  digitalWrite(motor1pin2, dir2);
  analogWrite(ENA, speed);
}

void M2(int dir1, int dir2, int speed) {
  digitalWrite(motor2pin1, dir1);
  digitalWrite(motor2pin2, dir2);
  analogWrite(ENB, speed);
}

void M3(int dir1, int dir2, int speed) {
  digitalWrite(motor3pin1, dir1);
  digitalWrite(motor3pin2, dir2);
  analogWrite(ENC, speed);
}

void M4(int dir1, int dir2, int speed) {
  digitalWrite(motor4pin1, dir1);
  digitalWrite(motor4pin2, dir2);
  analogWrite(END, speed);
}

// ROS Publishers and Subscribers
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  float linear_x = cmd_msg.linear.x;
  float linear_y = cmd_msg.linear.y;
  float angular_z = cmd_msg.angular.z;

  int speed1 = constrain(abs(linear_x * 255), 0, 255);
  int speed2 = constrain(abs(linear_y * 255), 0, 255);
  int speed3 = constrain(abs(angular_z * 255), 0, 255);

  if (linear_x > 0) {
    // Move forward
    M1(HIGH, LOW, speed1);
    M2(LOW, HIGH, speed1);
    M3(LOW, HIGH, speed1);
    M4(HIGH, LOW, speed1);
  } else if (linear_x < 0) {
    // Move backward
    M1(LOW, HIGH, speed1);
    M2(HIGH, LOW, speed1);
    M3(HIGH, LOW, speed1);
    M4(LOW, HIGH, speed1);
  } else if (linear_y > 0) {
    // Move left
    M1(LOW, HIGH, speed2);
    M2(LOW, HIGH, speed2);
    M3(HIGH, LOW, speed2);
    M4(HIGH, LOW, speed2);
  } else if (linear_y < 0) {
    // Move right
    M1(HIGH, LOW, speed2);
    M2(HIGH, LOW, speed2);
    M3(LOW, HIGH, speed2);
    M4(LOW, HIGH, speed2);
  } else if (angular_z > 0) {
    // Rotate clockwise
    M1(HIGH, LOW, speed3);
    M2(HIGH, LOW, speed3);
    M3(HIGH, LOW, speed3);
    M4(HIGH, LOW, speed3);
  } else if (angular_z < 0) {
    // Rotate counterclockwise
    M1(LOW, HIGH, speed3);
    M2(LOW, HIGH, speed3);
    M3(LOW, HIGH, speed3);
    M4(LOW, HIGH, speed3);
  } else {
    // Stop
    M1(LOW, LOW, 0);
    M2(LOW, LOW, 0);
    M3(LOW, LOW, 0);
    M4(LOW, LOW, 0);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

void setup() {
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
  nh.advertise(odom_pub);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2B), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3A), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3B), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4A), updateEncoder4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4B), updateEncoder4, CHANGE);
}

void loop() {
  // Calculate odometry
  long dEncoder1 = encoderValue1 - lastEncoderValue1;
  long dEncoder2 = encoderValue2 - lastEncoderValue2;
  long dEncoder3 = encoderValue3 - lastEncoderValue3;
  long dEncoder4 = encoderValue4 - lastEncoderValue4;

  lastEncoderValue1 = encoderValue1;
  lastEncoderValue2 = encoderValue2;
  lastEncoderValue3 = encoderValue3;
  lastEncoderValue4 = encoderValue4;

  float dS1 = (dEncoder1 * 2 * PI * wheelRadius) / pulsesPerRevolution;
  float dS2 = (dEncoder2 * 2 * PI * wheelRadius) / pulsesPerRevolution;
  float dS3 = (dEncoder3 * 2 * PI * wheelRadius) / pulsesPerRevolution;
  float dS4 = (dEncoder4 * 2 * PI * wheelRadius) / pulsesPerRevolution;

  // Compute robot movement
  float dX = (dS1 + dS2 + dS3 + dS4) / 4.0;
  float dY = (-dS1 + dS2 - dS3 + dS4) / 4.0;
  float dTheta = (-dS1 + dS2 + dS3 - dS4) / (4.0 * robotRadius);

  // Update robot position
  x += dX * cos(theta) - dY * sin(theta);
  y += dX * sin(theta) + dY * cos(theta);
  theta += dTheta;

  // Publish odometry message
  odom.header.stamp = nh.now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
  odom.twist.twist.linear.x = dX;
  odom.twist.twist.linear.y = dY;
  odom.twist.twist.angular.z = dTheta;
  odom_pub.publish(&odom);

  nh.spinOnce();
  delay(1);
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
