#!/usr/bin/env python
import cv2
import rospy
from geometry_msgs.msg import Twist

MAX = 100
MID = 50
PI = 3.1416
LINEAR_FACTOR = 2 / 100
ANGULAR_FACTOR = (2 * PI) / 100
 
def linear_x_change(val):
    vel.linear.x = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    pub.publish(vel)

def linear_y_change(val):
    vel.linear.y = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    pub.publish(vel)

def linear_z_change(val):
    vel.linear.z = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    pub.publish(vel)

def angular_z_change(val):
    vel.angular.z = (val * ANGULAR_FACTOR) - (MID * ANGULAR_FACTOR)
    pub.publish(vel)

if __name__ == '__main__':
    try:
        vel = Twist()
        pub = rospy.Publisher('rc_vel', Twist, queue_size=10)
        rospy.init_node('rc_node', anonymous=True)

        windowName = 'Radio Control'
        cv2.namedWindow(windowName)

        cv2.createTrackbar('Vx', windowName, MID, MAX, linear_x_change)
        cv2.createTrackbar('Vy', windowName, MID, MAX, linear_y_change)
        cv2.createTrackbar('Vz', windowName, MID, MAX, linear_z_change)
        cv2.createTrackbar('Wz', windowName, MID, MAX, angular_z_change)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass