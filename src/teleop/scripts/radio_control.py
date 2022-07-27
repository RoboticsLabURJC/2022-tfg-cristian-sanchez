#!/usr/bin/env python
import cv2
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped

MAX = 100
MID = 50
PI = 3.1416
LINEAR_FACTOR = 2 / 100
ANGULAR_FACTOR = (2 * PI) / 100
WINDOWNAME = 'Radio Control'

yaw = 0

# -- SLIDERS -- # 
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

# -- BUTTONS -- #
def launch_button(*args):
    set_sliders_to_default()
    pos.pose.position.z = 2
    pos_pub.publish(pos)

def land_button(*args):
    # Not working...
    set_sliders_to_default()
    pos.pose.position.z = 0
    pos_pub.publish(pos)

def turnCW_button(*args):
    global yaw
    set_sliders_to_default()

    yaw += PI/2

    if yaw == 2*PI:
        yaw = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

def turnACW_button(*args):
    global yaw
    set_sliders_to_default()

    yaw -= PI/2

    if yaw == -2*PI:
        yaw = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

def FW_button(*args):
    global yaw
    set_sliders_to_default()

    if yaw == 0:
        pos.pose.position.x += 2
    elif yaw == PI/2 or yaw == -(3*PI)/2:
        pos.pose.position.y += 2
    elif yaw == PI or yaw == -PI:
        pos.pose.position.x -= 2
    elif yaw == (3*PI)/2 or yaw == -PI/2:
        pos.pose.position.y -= 2

    pos_pub.publish(pos)

def BW_button(*args):
    global yaw
    set_sliders_to_default()

    if yaw == 0:
        pos.pose.position.x -= 2
    elif yaw == PI/2 or yaw == -(3*PI)/2:
        pos.pose.position.y -= 2
    elif yaw == PI or yaw == -PI:
        pos.pose.position.x += 2
    elif yaw == (3*PI)/2 or yaw == -PI/2:
        pos.pose.position.y += 2

    pos_pub.publish(pos)

# -- OTHER -- #
def set_sliders_to_default():
    sliderNames = ('Vx', 'Vy', 'Vz', 'Wz')
    
    for name in sliderNames:
        cv2.setTrackbarPos(name, WINDOWNAME, MID)

if __name__ == '__main__':
    try:
        vel = Twist()
        pos = PoseStamped()
        

        vel_pub = rospy.Publisher('rc/vel', Twist, queue_size=10)
        pos_pub = rospy.Publisher('rc/pos', PoseStamped, queue_size=10)

        rospy.init_node('rc_node', anonymous=True)

        cv2.namedWindow(WINDOWNAME)

        # Sliders
        cv2.createTrackbar('Vx', WINDOWNAME, MID, MAX, linear_x_change)
        cv2.createTrackbar('Vy', WINDOWNAME, MID, MAX, linear_y_change)
        cv2.createTrackbar('Vz', WINDOWNAME, MID, MAX, linear_z_change)
        cv2.createTrackbar('Wz', WINDOWNAME, MID, MAX, angular_z_change)

        # Switches
        cv2.createButton('Launch', launch_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Land', land_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('90 deg', turnCW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('-90 deg', turnACW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Fordward 2m', FW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Backward 2m', BW_button, None, cv2.QT_PUSH_BUTTON, 1)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass