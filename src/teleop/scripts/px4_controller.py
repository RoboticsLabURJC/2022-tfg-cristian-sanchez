#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from teleop.msg import Px4Cmd

#############
# PX4 DRONE #
#############

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
    # To avoid rejections
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel)
        rate.sleep()

def ok_to_fly (mode, arm):
    global last_req
    # 5s between request to avoid flooding the system
    if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        # OFFBOARD mode
        if(set_mode_client.call(mode).mode_sent == True):
            rospy.loginfo("OFFBOARD enabled")
        
        last_req = rospy.Time.now()
    else:
        if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            # Drone armed
            if(arming_client.call(arm).success == True):
                rospy.loginfo("Vehicle armed")
        
            last_req = rospy.Time.now()

# Callback to check the connection --> Drone armed + OFFBOARD
def state_cb(msg):
    global current_state
    current_state = msg

def rc_vel_cb(vel):
    global velocities, mode
    mode = VELOCITY
    velocities = vel

def rc_pos_cb(pos):
    global positions, mode
    mode = POSITION
    positions = pos

def rc_cmd_cb(cmd):
    global mode
    commands = cmd
    mode = commands.cmd

# -- MAIN -- #
if __name__ == "__main__":
    rospy.init_node("px4_controller")

    # Msgs
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rc_vel_sub = rospy.Subscriber("radio_control/vel", Twist, callback = rc_vel_cb)
    rc_pos_sub = rospy.Subscriber("radio_control/pos", PoseStamped, callback = rc_pos_cb)
    rc_cmd_sub = rospy.Subscriber("radio_control/cmd", Px4Cmd, callback = rc_cmd_cb)

    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # Services
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.wait_for_service("/mavros/cmd/land")
    land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
    
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
    while(not rospy.is_shutdown()):
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
