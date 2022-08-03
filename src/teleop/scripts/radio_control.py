#!/usr/bin/env python
import cv2
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from teleop.msg import Px4Cmd

# -- CTE -- #
# Topics
IMAGE_TOPIC = '/iris/usb_cam/image_raw'
LOCAL_POSE_TOPIC = '/mavros/local_position/pose'
RADIO_CONTROL_VEL_TOPIC = 'radio_control/vel'
RADIO_CONTROL_POS_TOPIC = 'radio_control/pos'
RADIO_CONTROL_CMD_TOPIC = 'radio_control/cmd'

# Other
MAX = 100
MID = 50
PI = 3.1416
LINEAR_FACTOR = 2 / 100
ANGULAR_FACTOR = (2 * PI) / 100
WINDOWNAME = 'Radio Control'
NODENAME = 'rc_node'

# Px4Cmd
IDLE = 0
TAKEOFF = 1
LAND = 2
POSITION = 3
VELOCITY = 4

# -- SLIDERS -- # 
def linear_FB_change(val):
    orientation_list = [current_pos.pose.orientation.x, 
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    r, p, y = tf.transformations.euler_from_quaternion(orientation_list)
    
    vel.linear.x = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.cos(y) 
    vel.linear.y = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.sin(y) 

    vel_pub.publish(vel)

def linear_LR_change(val):
    orientation_list = [current_pos.pose.orientation.x, 
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    r, p, y = tf.transformations.euler_from_quaternion(orientation_list)
    
    vel.linear.x = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.cos(y - PI/2) 
    vel.linear.y = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.sin(y - PI/2) 

    vel_pub.publish(vel)

def linear_z_change(val):
    vel.linear.z = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    vel_pub.publish(vel)

def angular_z_change(val):
    vel.angular.z = -((val * ANGULAR_FACTOR) - (MID * ANGULAR_FACTOR))
    vel_pub.publish(vel)

# -- BUTTONS -- #
def launch_button(*args):
    reset_sliders_update_pose()
    pos.pose.position.z = 2
    pos_pub.publish(pos)

def land_button(*args):
    reset_sliders_update_pose()

    cmd = Px4Cmd()
    cmd.cmd = LAND
    cmd_pub.publish(cmd)

def turnCW_button(*args):
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x, 
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    r, p, y = tf.transformations.euler_from_quaternion(orientation_list)

    quaternion = tf.transformations.quaternion_from_euler(r, p, y + PI/2)

    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

def turnACW_button(*args):
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x, 
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    r, p, y = tf.transformations.euler_from_quaternion(orientation_list)

    quaternion = tf.transformations.quaternion_from_euler(r, p, y - PI/2)

    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

def FW_button(*args):
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x, 
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    r, p, y = tf.transformations.euler_from_quaternion(orientation_list)
    
    pos.pose.position.x += 2 * math.cos(y) 
    pos.pose.position.y += 2 * math.sin(y)

    pos_pub.publish(pos)

def BW_button(*args):
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x, 
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    r, p, y = tf.transformations.euler_from_quaternion(orientation_list)
    
    pos.pose.position.x -= 2 * math.cos(y) 
    pos.pose.position.y -= 2 * math.sin(y)

    pos_pub.publish(pos)

def stop_button(*args):
    reset_sliders_update_pose()
    pos_pub.publish(current_pos)

# -- OTHER -- #
def reset_sliders_update_pose():
    global pos
    pos = current_pos
    
    sliderNames = ('Back|Front',
                   'Left|Right', 
                   'Down|Up', 
                   'ACW|CW')
    
    for name in sliderNames:
        cv2.setTrackbarPos(name, WINDOWNAME, MID)

# Callback
def current_pos_cb(pose):
    global current_pos
    current_pos = pose

def image_cb(img_msg):
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    cv2.imshow(WINDOWNAME, cv_image)

# -- MAIN -- #
if __name__ == '__main__':
    try:
        rospy.init_node(NODENAME, anonymous=True)

        vel = Twist()
        pos = PoseStamped() 
        bridge = CvBridge()
        current_pos = PoseStamped()
        
        # Msgs
        ## Subscribers
        image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, callback = image_cb)
        current_pos_sub = rospy.Subscriber(LOCAL_POSE_TOPIC, PoseStamped, callback = current_pos_cb)

        ## Publishers
        vel_pub = rospy.Publisher(RADIO_CONTROL_VEL_TOPIC, Twist, queue_size=10)
        pos_pub = rospy.Publisher(RADIO_CONTROL_POS_TOPIC, PoseStamped, queue_size=10)
        cmd_pub = rospy.Publisher(RADIO_CONTROL_CMD_TOPIC, Px4Cmd, queue_size=10)

        # -- OPENCV -- #
        cv2.namedWindow(WINDOWNAME)

        # Sliders
        cv2.createTrackbar('Back|Front', WINDOWNAME, MID, MAX, linear_FB_change)
        cv2.createTrackbar('Left|Right', WINDOWNAME, MID, MAX, linear_LR_change)
        cv2.createTrackbar('Down|Up', WINDOWNAME, MID, MAX, linear_z_change)
        cv2.createTrackbar('ACW|CW', WINDOWNAME, MID, MAX, angular_z_change)

        # Buttons
        cv2.createButton('Launch', launch_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Land', land_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('-90 deg', turnCW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('90 deg', turnACW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Fordward 2m', FW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Backward 2m', BW_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('STOP', stop_button, None, cv2.QT_PUSH_BUTTON, 1)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass