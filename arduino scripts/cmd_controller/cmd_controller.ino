#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

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
}

void loop() {
  nh.spinOnce();
  delay(1);
}

int M1(int x, int y, int z) {
  digitalWrite(34, x);
  digitalWrite(36, y);
  analogWrite(10, z);
}

int M2(int x, int y, int z) {
  digitalWrite(38, x);
  digitalWrite(40, y);
  analogWrite(11, z);
}

int M3(int x, int y, int z) {
  digitalWrite(26, x);
  digitalWrite(28, y);
  analogWrite(8, z);
}

int M4(int x, int y, int z) {
  digitalWrite(30, x);
  digitalWrite(32, y);
  analogWrite(9, z);
}
