#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

#############
# PX4 DRONE #
#############

# -- GLOBAL VARIABLES -- #
current_state = State()
velocities = Twist()

# -- METHODS -- #
def avoid_rejection(vel):
    # To avoid rejections
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel)
        rate.sleep()

def system_check(mode, arm):
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

def rc_cb(vel):
    global velocities
    velocities = vel


# -- MAIN -- #
if __name__ == "__main__":
    rospy.init_node("px4_controller")

    # Msgs
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rc_sub = rospy.Subscriber("rc_vel", Twist, callback = rc_cb)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    
    # Services
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
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

    # Timer for requesting
    last_req = rospy.Time.now()

    # Operating loop
    while(not rospy.is_shutdown()):
        system_check(offb_set_mode, arm_cmd)
        local_vel_pub.publish(velocities)
        rate.sleep()
