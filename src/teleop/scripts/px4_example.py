#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import threading

# ------------------------------------------------------------------------------ #

#######
# GUI #
#######

Z_MIN = 0
Z_MAX = 100
STEP = 1
V_Z = 0.0

class sliderdemo(QWidget):
    def __init__(self, parent = None):
        super(sliderdemo, self).__init__(parent)

        layout = QVBoxLayout()
        
        # Title for slider
        self.l1 = QLabel("linear_z")
        layout.addWidget(self.l1)
        
        # Slider
        self.sl = QSlider(Qt.Vertical)
        self.sl.setMinimum(Z_MIN)
        self.sl.setMaximum(Z_MAX)
        self.sl.setValue(Z_MIN)
        self.sl.setTickPosition(QSlider.TicksBelow)
        self.sl.setTickInterval(STEP)        
        layout.addWidget(self.sl)

        # Set all
        self.sl.valueChanged.connect(self.valuechange)
        self.setLayout(layout)
        self.setWindowTitle("Drone controller")
        self.setGeometry(100, 100, 50, 400)

        self.show()

    def valuechange(self):
        V_Z = self.sl.value()/Z_MAX

def GUI_function(app):
    # Threading tests (not working)
    app.exec_()

# ------------------------------------------------------------------------------ #

#############
# PX4 DRONE #
#############

current_state = State()

# Callback to check the connection --> Drone armed + OFFBOARD
def state_cb(msg):
    global current_state
    current_state = msg



if __name__ == "__main__":







    

    # ---- GUI instances ---- #
    rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    app = QApplication(sys.argv)
    # ex = sliderdemo()
    # ex.show()
    GUI_thread = threading.Thread(target=GUI_function, args=(app,))
    GUI_thread.start()
    rospy.loginfo("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")









    
    # ---- ROS part ---- #
    rospy.init_node("px4_example_py")

    # Mavros name??
    # msg sub/pub
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    
    # services
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Once connected, elevate 2 m
    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    vel = Twist()
    vel.linear.x = 0.0
    vel.linear.y = 0.0
    vel.linear.z = 0.0

    vel.angular.x = 0.0
    vel.angular.x = 0.0
    vel.angular.x = 0.0


    # Send a few setpoints before starting
    # otherwise mode will be rejected ??
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel)
        rate.sleep()

    # Init OFFBOARD request
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Init arm request
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    # 5s between request to avoid flooding the system
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            # OFFBOARD mode
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                # Drone armed
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        # Pub without checking offboard mode and drone armed??
        # local_pos_pub.publish(pose)

        rospy.loginfo("v_z: %s", str(V_Z))
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = V_Z

        vel.angular.x = 0.0
        vel.angular.x = 0.0
        vel.angular.x = 0.0
        local_vel_pub.publish(vel)

        # Faster than 2Hz
        rate.sleep()