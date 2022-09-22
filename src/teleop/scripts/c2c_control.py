#!/usr/bin/env python
'''
RADIO CONTROL

This module uses openCV to make a radio control GUI. The goal is to operate remotely
the Iris drone simulated inside Gazebo 11.

It provides the following:
    Buttons
        Launch
        Land
        Left 2m
        Front 2m
        Back 2m
        Right 2m
        STOP

    Display
        Which shows a front view of the drone
'''
import cv2
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from teleop.msg import Px4Cmd

# -- CTE -- #
# Topics
IMAGE_TOPIC = '/iris/usb_cam/image_raw'
LOCAL_POSE_TOPIC = '/mavros/local_position/pose'
RADIO_CONTROL_POS_TOPIC = 'radio_control/pos'
RADIO_CONTROL_CMD_TOPIC = 'radio_control/cmd'

# Other
MAX = 100
MID = 50
PI = 3.1416
LINEAR_FACTOR = 2 / 100
ANGULAR_FACTOR = (2 * PI) / 100
WINDOWNAME = 'Radio Control'
NODENAME = 'c2c_node'

# Px4Cmd
IDLE = 0
TAKEOFF = 1
LAND = 2
POSITION = 3
VELOCITY = 4

# -- GLOBAL VARIABLES -- #
pos = PoseStamped()
current_pos = PoseStamped()

# -- BUTTONS -- #
def launch_button(*args):
    '''
    Register if the launch button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case elevates 2 m.
    '''
    _ = args
    update_pose()
    pos.pose.position.z = 2
    pos_pub.publish(pos)

def land_button(*args):
    '''
    Register if the land button is pressed. Then publish the command land.

    In this case land at the current position.
    '''
    _ = args
    update_pose()
    cmd = Px4Cmd()
    cmd.cmd = LAND
    cmd_pub.publish(cmd)

def front_button(*args):
    '''
    Register if the ↑ button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case move 2 m to the front.
    '''
    _ = args
    update_pose()
    pos.pose.position.x = current_pos.pose.position.x + 2
    pos_pub.publish(pos)

def back_button(*args):
    '''
    Register if the ↓ button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case move 2 m to the back.
    '''
    _ = args
    update_pose()
    pos.pose.position.x = current_pos.pose.position.x - 2
    pos_pub.publish(pos)

def left_button(*args):
    '''
    Register if the ← button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case move 2 m to de left.
    '''
    _ = args
    update_pose()
    pos.pose.position.y = current_pos.pose.position.y + 2
    pos_pub.publish(pos)

def right_button(*args):
    '''
    Register if the → button is pressed. Then publish next pose relative to the drone
    reference system.

    In this case move 2 m to the right.
    '''
    _ = args
    update_pose()
    pos.pose.position.y = current_pos.pose.position.y - 2
    pos_pub.publish(pos)

def stop_button(*args):
    '''
    Register if the STOP button is pressed. Then stops the drone in that position.
    '''
    _ = args
    update_pose()
    pos_pub.publish(current_pos)

# -- OTHER -- #
def update_pose():
    '''
    Update local pose and set all the sliders to default.
    '''
    global pos
    pos = current_pos


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
        pos_pub = rospy.Publisher(RADIO_CONTROL_POS_TOPIC, PoseStamped, queue_size=10)
        cmd_pub = rospy.Publisher(RADIO_CONTROL_CMD_TOPIC, Px4Cmd, queue_size=10)

        # -- OPENCV -- #
        cv2.namedWindow(WINDOWNAME)

        # Buttons
        cv2.createButton('Launch', launch_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('Land', land_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('←', left_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('↑', front_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('↓', back_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('→', right_button, None, cv2.QT_PUSH_BUTTON, 1)
        cv2.createButton('STOP', stop_button, None, cv2.QT_PUSH_BUTTON, 1)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
