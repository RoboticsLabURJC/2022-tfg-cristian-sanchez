#! /usr/bin/env python
'''
PX4 CONTROLLER

This module is made to communicate with the Iris drone, using MavROS.

It allows to Send:
    Poses
    Velocities
    Specific commands (Land request)

And receive:
    States
    Responses
'''
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import (
        CommandBool,
        CommandBoolRequest,
        SetMode,
        SetModeRequest,
        CommandTOL,
        CommandTOLRequest
    )
from teleop.msg import Px4Cmd

#############
# PX4 DRONE #
#############

# -- CTE -- #
# Other
NODENAME = 'px4_controller'

# Topics
STATE_TOPIC = 'mavros/state'
RADIO_CONTROL_VEL_TOPIC = 'radio_control/vel'
RADIO_CONTROL_POS_TOPIC = 'radio_control/pos'
RADIO_CONTROL_CMD_TOPIC = 'radio_control/cmd'
SET_VEL_TOPIC = 'mavros/setpoint_velocity/cmd_vel_unstamped'
SET_POS_TOPIC = 'mavros/setpoint_position/local'

# Services
ARM_SRV = '/mavros/cmd/arming'
SET_MODE_SRV= '/mavros/set_mode'
LAND_CMD_SRV = '/mavros/cmd/land'

# Px4Cmd
IDLE = 0
TAKEOFF = 1
LAND = 2
POSITION = 3
VELOCITY = 4

# -- GLOBAL VARIABLES -- #
current_state = State()
velocities = Twist()
positions = PoseStamped()
commands = Px4Cmd()
mode = VELOCITY

# -- METHODS -- #
def avoid_rejection(vel):
    '''
    Compatibility purposes, necesary to communicate with Mavlink stuff.
    '''
    # To avoid rejections
    for _ in range(100):
        if rospy.is_shutdown():
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel)
        rate.sleep()

def ok_to_fly (mod, arm):
    '''
    Puts the drone in OFFBOARD mode and armed via requests.
    '''
    global last_req
    # 5s between request to avoid flooding the system
    if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        # OFFBOARD mode
        if set_mode_client.call(mod).mode_sent is True:
            rospy.loginfo("OFFBOARD enabled")

        last_req = rospy.Time.now()
    else:
        if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            # Drone armed
            if arming_client.call(arm).success is True:
                rospy.loginfo("Vehicle armed")

            last_req = rospy.Time.now()

# Callback to check the connection --> Drone armed + OFFBOARD
def state_cb(msg):
    '''
    State callback
    '''
    global current_state
    current_state = msg

def rc_vel_cb(vel):
    '''
    Velocity callback
    '''
    global velocities, mode
    mode = VELOCITY
    velocities = vel

def rc_pos_cb(pos):
    '''
    Pose callback
    '''
    global positions, mode
    mode = POSITION
    positions = pos

def rc_cmd_cb(cmd):
    '''
    Command callback
    '''
    global mode
    commands = cmd
    mode = commands.cmd

# -- MAIN -- #
if __name__ == "__main__":
    rospy.init_node(NODENAME)

    # Msgs
    ## Subscribers
    state_sub = rospy.Subscriber(STATE_TOPIC, State, callback = state_cb)
    rc_vel_sub = rospy.Subscriber(RADIO_CONTROL_VEL_TOPIC, Twist, callback = rc_vel_cb)
    rc_pos_sub = rospy.Subscriber(RADIO_CONTROL_POS_TOPIC, PoseStamped, callback = rc_pos_cb)
    rc_cmd_sub = rospy.Subscriber(RADIO_CONTROL_CMD_TOPIC, Px4Cmd, callback = rc_cmd_cb)

    ## Publishers
    local_vel_pub = rospy.Publisher(SET_VEL_TOPIC, Twist, queue_size=10)
    local_pos_pub = rospy.Publisher(SET_POS_TOPIC, PoseStamped, queue_size=10)

    # Services
    rospy.wait_for_service(ARM_SRV)
    arming_client = rospy.ServiceProxy(ARM_SRV, CommandBool)
    rospy.wait_for_service(SET_MODE_SRV)
    set_mode_client = rospy.ServiceProxy(SET_MODE_SRV, SetMode)
    rospy.wait_for_service(LAND_CMD_SRV)
    land_client = rospy.ServiceProxy(LAND_CMD_SRV, CommandTOL)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    avoid_rejection(velocities)

    # Initializations
    # OFFBOARD mode request
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Arm request
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # Land request
    land_cmd = CommandTOLRequest()
    land_cmd.min_pitch = 0
    land_cmd.yaw = 0
    land_cmd.latitude = 0
    land_cmd.longitude = 0
    land_cmd.altitude = 0

    # Timer for requesting
    last_req = rospy.Time.now()

    # Operating loop
    while not rospy.is_shutdown():
        ok_to_fly(offb_set_mode, arm_cmd)

        if mode == POSITION:
            local_pos_pub.publish(positions)
        elif mode == VELOCITY:
            local_vel_pub.publish(velocities)
        elif mode == LAND:
            if(rospy.Time.now() - last_req) > rospy.Duration(5.0):
                response = land_client.call(land_cmd)
                last_req = rospy.Time.now()

        rate.sleep()
