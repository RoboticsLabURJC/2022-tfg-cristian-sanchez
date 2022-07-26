#!/usr/bin/env python
import cv2
import rospy
from geometry_msgs.msg import Twist

MAX = 100
MID = 50
PI = 3.1416
LINEAR_FACTOR = 2 / 100
ANGULAR_FACTOR = (2 * PI) / 100
WINDOWNAME = 'Radio Control'
 
def linear_x_change(val):
    vel.linear.x = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    vel_pub.publish(vel)

def linear_y_change(val):
    vel.linear.y = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    vel_pub.publish(vel)

def linear_z_change(val):
    vel.linear.z = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    vel_pub.publish(vel)

def angular_z_change(val):
    vel.angular.z = (val * ANGULAR_FACTOR) - (MID * ANGULAR_FACTOR)
    vel_pub.publish(vel)

def launch_switch(button):
    print(button)

def land_switch(button):
    print(button)

def turn_cw_switch(button):
    print(button)

def turn_acw_switch(button):
    print(button)

def set_sliders_to_default():
    sliderNames = ('Vx', 'Vy', 'Vz', 'Wz')
    
    for name in sliderNames:
        cv2.setTrackbarPos(name, WINDOWNAME, MID)

if __name__ == '__main__':
    try:
        vel = Twist()
        vel_pub = rospy.Publisher('rc_vel', Twist, queue_size=10)
        rospy.init_node('rc_node', anonymous=True)

        cv2.namedWindow(WINDOWNAME)

        # Sliders
        cv2.createTrackbar('Vx', WINDOWNAME, MID, MAX, linear_x_change)
        cv2.createTrackbar('Vy', WINDOWNAME, MID, MAX, linear_y_change)
        cv2.createTrackbar('Vz', WINDOWNAME, MID, MAX, linear_z_change)
        cv2.createTrackbar('Wz', WINDOWNAME, MID, MAX, angular_z_change)

        # Switches
        cv2.createTrackbar('Launch', WINDOWNAME, 0, 1, launch_switch)
        cv2.createTrackbar('Land', WINDOWNAME, 0, 1, land_switch)
        cv2.createTrackbar('Turn 90º', WINDOWNAME, 0, 1, turn_cw_switch)
        cv2.createTrackbar('Turn -90º', WINDOWNAME, 0, 1, turn_acw_switch)


        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass