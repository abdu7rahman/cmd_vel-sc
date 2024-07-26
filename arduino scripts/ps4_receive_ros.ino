/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
int l,r,f,b;
ros::NodeHandle  nh;

void messageCb( const std_msgs::Int64MultiArray& cmd_msg){
  
  l=cmd_msg.data[0];
  r=cmd_msg.data[1];
  f=cmd_msg.data[2];
  b=cmd_msg.data[3];

  if (l>=40){
    nh.loginfo("left");
  }
  else if (r>=40){
    nh.loginfo("right");
  }
  else if (f>=40){
    nh.loginfo("front");
  }
  else if (b>=40){
    nh.loginfo("back");
  }
  else {
    nh.loginfo("stop");
  }
}

ros::Subscriber<std_msgs::Int64MultiArray> sub("cmd_vel", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
