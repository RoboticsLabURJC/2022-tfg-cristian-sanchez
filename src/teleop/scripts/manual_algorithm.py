#!/usr/bin/env python
'''
ToDo...
'''
import rospy
from geometry_msgs.msg import PoseStamped
from teleop.msg import Px4Cmd
import actionlib
from heatmap_util.msg import GetPowerFrissAction, GetPowerFrissGoal

# -- CTE -- #
# Topics
LOCAL_POSE_TOPIC = '/mavros/local_position/pose'
RADIO_CONTROL_POS_TOPIC = 'radio_control/pos'
RADIO_CONTROL_CMD_TOPIC = 'radio_control/cmd'

# Other
NODENAME = 'manual_algorithm_node'
TOLERANCE = 0.1
CELLSIZE = 1.0
TIMEOUT = 0.1
H = 2.0

# Px4Cmd
IDLE = 0
TAKEOFF = 1
LAND = 2
POSITION = 3
VELOCITY = 4

# -- GLOBAL VARIABLES -- #
target_pos = PoseStamped()
current_pos = PoseStamped()

class Drone:
    def __init__(self):
        rospy.init_node(NODENAME)

        self.pos_pub = rospy.Publisher(RADIO_CONTROL_POS_TOPIC, PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher(RADIO_CONTROL_CMD_TOPIC, Px4Cmd, queue_size=10)

        self.pwr_client = actionlib.SimpleActionClient('drone_friss_action', GetPowerFrissAction)
        self.pwr_client.wait_for_server()

        self.pwr_goal = GetPowerFrissGoal()

        self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        self.target_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        self.takeoff()

    def takeoff(self):
        rospy.loginfo("Takeoff detected!")
        self.target_pos.pose.position.z = H
        while not self.h_reached(self.current_pos):
            rospy.loginfo("Taking off...")            
            self.pos_pub.publish(self.target_pos)
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Takeoff OK!")
        
    def h_reached(self, current_pose):
        current_z = current_pose.pose.position.z
        target_z = self.target_pos.pose.position.z

        reached_z = current_z - TOLERANCE < target_z < current_z + TOLERANCE

        return reached_z

    def centered(self, current_pose):
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        target_x = self.target_pos.pose.position.x
        target_y = self.target_pos.pose.position.y

        reached_x = current_x - TOLERANCE < target_x < current_x + TOLERANCE
        reached_y = current_y - TOLERANCE < target_y < current_y + TOLERANCE

        return reached_x and reached_y

    def move_to(self, cmd="", pose=PoseStamped()):
        rospy.loginfo("Move " + cmd + " detected!")
        if cmd == "FRONT":
            self.target_pos.pose.position.x = self.current_pos.pose.position.x + CELLSIZE
        elif cmd == "BACK":
            self.target_pos.pose.position.x = self.current_pos.pose.position.x - CELLSIZE
        elif cmd == "LEFT":
            self.target_pos.pose.position.y = self.current_pos.pose.position.y + CELLSIZE
        elif cmd == "RIGHT":
            self.target_pos.pose.position.y = self.current_pos.pose.position.y - CELLSIZE
        else:
            self.target_pos = pose
        
        while not self.centered(self.current_pos):
            rospy.loginfo("Moving...")
            self.pos_pub.publish(self.target_pos)
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Reached!")

    def gzcoords_to_heatmapcoords(self, gzcoords):
        gz_x, gz_y = gzcoords
        
        heat_x = round(4 - gz_x)
        heat_y = round(4 - gz_y)

        return [heat_x, heat_y]

    
    def read_pwr(self):
        current_pose = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        current_coords = (current_pose.pose.position.x, current_pose.pose.position.y)
        self.pwr_goal.index = self.gzcoords_to_heatmapcoords(current_coords)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        return (self.pwr_client.get_result().data, current_coords)

    def manual_algorithm(self):
        readings = []
        readings_coords = []
        origin = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        while True:
            self.move_to("FRONT")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("RIGHT")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("BACK")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("BACK")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("LEFT")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("LEFT")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("FRONT")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("FRONT")
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            self.move_to("INITIAL POSE", pose=origin)
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            origin.pose.position.x = readings_coords[readings.index(max(readings))][0]
            origin.pose.position.y = readings_coords[readings.index(max(readings))][1]
            self.move_to("NEXT", pose=origin)

# -- MAIN -- #
if __name__ == '__main__':
    iris = Drone()
    iris.manual_algorithm()
