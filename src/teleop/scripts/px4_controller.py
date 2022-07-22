#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

#############
# PX4 DRONE #
#############

current_state = State()
velocities = Twist()

# Callback to check the connection --> Drone armed + OFFBOARD
def state_cb(msg):
    global current_state
    current_state = msg

def rc_cb(vel):
    global velocities
    velocities = vel

if __name__ == "__main__":
    rospy.init_node("px4_controller")

    # Mavros name??
    # msg sub/pub
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rc_sub = rospy.Subscriber("rc_vel", Twist, callback = rc_cb)
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

    # Once connected    
    # Send a few setpoints before starting
    # otherwise mode will be rejected ??
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(velocities)
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
        local_vel_pub.publish(velocities)

        # Faster than 2Hz
        rate.sleep()
