#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy


class joystick:

    def __init__(self):
        rospy.init_node('swerve_node', anonymous=False)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy)

    def joy(self, data):
        a = data.axes
        b = data.buttons

        LX = a[0]
        LY = a[1]
        RX = a[3]

        f = int(translate(self, LY, 0.1, 1, 0, 255))
        b = int(translate(self, LY, -0.1, -1, 0, 255))
        l = int(translate(self, LX, 0.1, 1, 0, 255))
        r = int(translate(self, LX, -0.1, -1, 0, 255))

        f = sorted([0, f, 255])[1]
        b = sorted([0, b, 255])[1]
        l = sorted([0, l, 255])[1]
        r = sorted([0, r, 255])[1]

        if  (l>0):
            print("LX-LEFT", l)
        if (r>0):                  #(LX < 0.2):
            print("LX-RIGHT", r)
        if (f>0):                  #(LY > 0.1):
            print("LY-FORW", f)
        if (b>0):                  #(LY < 0.2):
            print("LY-BACK", b)

        print(f,b,r,l)

if __name__ == "__main__":

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

    enjoy = joystick()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
	rate.sleep()


































































rate.sleep()
