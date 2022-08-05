#!/usr/bin/env python
'''
RADIO CONTROL

This module uses openCV to make a radio control GUI. The goal is to operate remotely
the Iris drone simulated inside Gazebo 11.

It provides the following:
    Sliders
        Back|Front
        Left|Right
        Down|Up
        ACW|CW

    Buttons
        Launch
        Land
        -90 deg
        90 deg
        Forward 2m
        Backward 2m
        STOP

    Display
        Which shows a front view of the drone
'''
import math
import cv2
import rospy
import tf
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

# -- GLOBAL VARIABLES -- #
pos = PoseStamped()
current_pos = PoseStamped()

# -- SLIDERS -- #
def linear_back_front_change(val):
    '''
    Register value changes inside Back|Front slider, and publish it transformated into linear
    velocities [-1, 1] relative to the drone reference system.

        Parameter:
            val (int): integer between 0 and 100
    '''
    orientation_list = [current_pos.pose.orientation.x,
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)

    vel = Twist()
    vel.linear.x = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.cos(yaw)
    vel.linear.y = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.sin(yaw)

    vel_pub.publish(vel)

def linear_left_right_change(val):
    '''
    Register value changes inside Left|Right slider, and publish it transformated into linear
    velocities [-1, 1] relative to the drone reference system.

        Parameter:
            val (int): integer between 0 and 100
    '''
    orientation_list = [current_pos.pose.orientation.x,
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)

    vel = Twist()
    vel.linear.x = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.cos(yaw - PI/2)
    vel.linear.y = ((val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR))* math.sin(yaw - PI/2)

    vel_pub.publish(vel)

def linear_z_change(val):
    '''
    Register value changes inside Down|Up slider, and publish it transformated into linear
    velocities [-1, 1] relative to the drone reference system.

        Parameter:
            val (int): integer between 0 and 100
    '''
    vel = Twist()
    vel.linear.z = (val * LINEAR_FACTOR) - (MID * LINEAR_FACTOR)
    vel_pub.publish(vel)

def angular_z_change(val):
    '''
    Register value changes inside ACW|CW slider, and publish it transformated into angular
    velocities [3.14, -3.14] relative to the drone reference system.

        Parameter:
            val (int): integer between 0 and 100
    '''
    vel = Twist()
    vel.angular.z = -((val * ANGULAR_FACTOR) - (MID * ANGULAR_FACTOR))
    vel_pub.publish(vel)

# -- BUTTONS -- #
def launch_button(*args):
    '''
    Register if the launch button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case elevates 2 m.
    '''
    _ = args
    reset_sliders_update_pose()
    pos.pose.position.z = 2
    pos_pub.publish(pos)

def land_button(*args):
    '''
    Register if the land button is pressed. Then publish the command land.

    In this case land at the current position.
    '''
    _ = args
    reset_sliders_update_pose()
    cmd = Px4Cmd()
    cmd.cmd = LAND
    cmd_pub.publish(cmd)

def turn_clockwise_button(*args):
    '''
    Register if the -90 deg button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case turns 90 degrees clockwise.
    '''
    _ = args
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x,
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw + PI/2)

    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

def turn_anticlockwise_button(*args):
    '''
    Register if the 90 deg button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case turns 90 degrees anticlockwise.
    '''
    _ = args
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x,
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw - PI/2)

    pos.pose.orientation.x = quaternion[0]
    pos.pose.orientation.y = quaternion[1]
    pos.pose.orientation.z = quaternion[2]
    pos.pose.orientation.w = quaternion[3]

    pos_pub.publish(pos)

def forward_button(*args):
    '''
    Register if the Forward 2m button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case move 2 m forward.
    '''
    _ = args
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x,
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)

    pos.pose.position.x += 2 * math.cos(yaw)
    pos.pose.position.y += 2 * math.sin(yaw)

    pos_pub.publish(pos)

def backward_button(*args):
    '''
    Register if the Backward 2m button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case move 2 m backward.
    '''
    _ = args
    reset_sliders_update_pose()

    orientation_list = [current_pos.pose.orientation.x,
                        current_pos.pose.orientation.y,
                        current_pos.pose.orientation.z,
                        current_pos.pose.orientation.w]

    _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)

    pos.pose.position.x -= 2 * math.cos(yaw)
    pos.pose.position.y -= 2 * math.sin(yaw)

    pos_pub.publish(pos)

def stop_button(*args):
    '''
    Register if the STOP button is pressed. Then stops the drone in that position.
    '''
    _ = args
    reset_sliders_update_pose()
    pos_pub.publish(current_pos)

# -- OTHER -- #
def reset_sliders_update_pose():
    '''
    Update local pose and set all the sliders to default.
    '''
    global pos
    pos = current_pos

    slider_names = ('Back|Front',
                   'Left|Right',
                   'Down|Up',
                   'ACW|CW')

    for name in slider_names:
        # pylint: disable=E1101
        cv2.setTrackbarPos(name, WINDOWNAME, MID)
        # pylint: enable=E1101

# Callback
def current_pos_cb(pose):
    '''
    Local pose callback.
    '''
    global current_pos
    current_pos = pose

def image_cb(img_msg):
    '''
    Image callback.
    '''
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    # pylint: disable=E1101
    cv2.imshow(WINDOWNAME, cv_image)
    # pylint: enable=E1101

# -- MAIN -- #
if __name__ == '__main__':
    try:
        rospy.init_node(NODENAME, anonymous=True)

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
        cv2.createTrackbar('Back|Front', WINDOWNAME, MID, MAX, linear_back_front_change)
        cv2.createTrackbar('Left|Right', WINDOWNAME, MID, MAX, linear_left_right_change)
        cv2.createTrackbar('Down|Up', WINDOWNAME, MID, MAX, linear_z_change)
        cv2.createTrackbar('ACW|CW', WINDOWNAME, MID, MAX, angular_z_change)

        # Buttons
        cv2.createButton('Launch', launch_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Land', land_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('-90 deg', turn_clockwise_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('90 deg', turn_anticlockwise_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Fordward 2m', forward_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Backward 2m', backward_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('STOP', stop_button, None, cv2.QT_PUSH_BUTTON, 1)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
