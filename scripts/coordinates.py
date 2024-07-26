import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(msg):
    # Extract position and orientation from the Odometry message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Orientation as quaternion
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)

    # Print or use the coordinates and angles
    print(f"Position (x, y): ({x}, {y})")
    #print(f"Orientation (roll, pitch, yaw): ({roll}, {pitch}, {yaw})")

def odom_listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass

