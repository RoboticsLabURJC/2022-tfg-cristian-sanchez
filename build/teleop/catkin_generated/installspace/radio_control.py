#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
 
def linear_x_change(val):
    vel.linear.x = val/100
    pub.publish(vel)

def linear_y_change(val):
    vel.linear.y = val/100
    pub.publish(vel)

def linear_z_change(val):
    vel.linear.z = val/100
    pub.publish(vel)

def angular_x_change(val):
    vel.angular.x = val/100
    pub.publish(vel)

def angular_y_change(val):
    vel.angular.y = val/100
    pub.publish(vel)

def angular_z_change(val):
    vel.angular.z = val/100
    pub.publish(vel)

if __name__ == '__main__':
    try:
        vel = Twist()
        pub = rospy.Publisher('rc_vel', Twist, queue_size=10)
        rospy.init_node('rc_node', anonymous=True)

        blank_image = np.zeros(shape=[512, 512, 3], dtype=np.uint8) 
        windowName = 'Radio Control'

        cv2.imshow(windowName, blank_image)

        cv2.createTrackbar('Vx', windowName, 0, 100, linear_x_change)
        cv2.createTrackbar('Vy', windowName, 0, 100, linear_y_change)
        cv2.createTrackbar('Vz', windowName, 0, 100, linear_z_change)
        cv2.createTrackbar('Wx', windowName, 0, 100, angular_x_change)
        cv2.createTrackbar('Wy', windowName, 0, 100, angular_y_change)
        cv2.createTrackbar('Wz', windowName, 0, 100, angular_z_change)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass