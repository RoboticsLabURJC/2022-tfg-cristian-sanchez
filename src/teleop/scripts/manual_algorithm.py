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
TOLERANCE = 0.0675
CELLSIZE = 1.0
TIMEOUT = 0.1
H = 1.0

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
        self.pwr_goal.index = [0, 0]

        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        self.size = self.pwr_client.get_result().size

        self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        self.target_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

    def takeoff(self):
        rospy.loginfo("Takeoff detected!")
        self.target_pos.pose.position.z = H
        while not self.h_reached(self.current_pos):
            rospy.loginfo("Taking off...")            
            self.pos_pub.publish(self.target_pos)
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Takeoff OK!")

    def land(self):
        cmd = Px4Cmd()
        cmd.cmd = LAND
    
        self.target_pos.pose.position.z = 0.0
        while not self.h_reached(self.current_pos):
            rospy.loginfo("Landing...")   
            self.cmd_pub.publish(cmd)         
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        
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
            # rospy.loginfo("Moving...")
            self.pos_pub.publish(self.target_pos)
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Reached!")

    def gzcoords_to_heatmapcoords(self, gzcoords):
        gz_x, gz_y = gzcoords
        
        heat_x = round((self.size / 2) - 1 - gz_x)
        heat_y = round((self.size / 2) - 1 - gz_y)

        return (heat_x, heat_y)
    
    def heatmapcoords_to_gzcoords(self, hmcoords):
        hm_x, hm_y = hmcoords
        
        gz_x = (self.size / 2) - 1 - hm_x
        gz_y = (self.size / 2) - 1 - hm_y

        return (gz_x, gz_y)

    
    def read_pwr(self):
        current_pose = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        current_coords = (current_pose.pose.position.x, current_pose.pose.position.y)
        self.pwr_goal.index = self.gzcoords_to_heatmapcoords(current_coords)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        return (self.pwr_client.get_result().data, current_coords)

    def manual_algorithm(self):
        readings = []
        readings_prev = []  
        readings_coords = []
        goal_pose = PoseStamped()

        signal_found = False
        goal_pose.pose.position.z = H
        neighbors = ("FRONT", "RIGHT", "BACK", "BACK", "LEFT", "LEFT", "FRONT", "FRONT")

        self.takeoff()
        start_time = rospy.Time.now()
        while not signal_found:
            for neigh in neighbors:
                read, coord = self.read_pwr()
                readings.append(read)
                readings_coords.append(coord)
                self.move_to(neigh)

            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)

            goal_pose.pose.position.x = readings_coords[readings.index(max(readings))][0]
            goal_pose.pose.position.y = readings_coords[readings.index(max(readings))][1]  

            self.move_to("GOAL", pose=goal_pose)

            for i in range(len(readings)):
                if len(readings_prev) == 0:
                    readings_prev = readings.copy()
                    break
                elif readings[i] != readings_prev[i]:
                    signal_found = False
                    readings_prev = readings.copy()
                    break
                else:
                    signal_found = True

            readings.clear()
            readings_coords.clear()

        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()


    def manual_algorithm_optimized(self):
        readings = []
        visited = set() 
        readings_coords = []
        goal_pose = PoseStamped()
        next_pose = PoseStamped()
        goal_pose.pose.position.z = H
        next_pose.pose.position.z = H
        last_goal = 0

        self.takeoff()
        start_time = rospy.Time.now()
        while True:
            read, coord = self.read_pwr()
            hm_coords = self.gzcoords_to_heatmapcoords(coord)

            readings.append(read)
            readings_coords.append(coord)
            visited.add(hm_coords)
            
            path = self.get_next_positions(hm_coords, visited)

            for x, y in path:
                next_pose.pose.position.x = x
                next_pose.pose.position.y = y

                self.move_to("NEXT", pose=next_pose)
                read, coord = self.read_pwr()
                readings.append(read)
                readings_coords.append(coord)
                visited.add(self.gzcoords_to_heatmapcoords(coord))

            goal_pose.pose.position.x = readings_coords[readings.index(max(readings))][0]
            goal_pose.pose.position.y = readings_coords[readings.index(max(readings))][1]
            current_goal = self.gzcoords_to_heatmapcoords((goal_pose.pose.position.x, goal_pose.pose.position.y))

            self.move_to("GOAL", pose=goal_pose)

            if last_goal == current_goal:
                break
            else:
                last_goal = current_goal

            readings.clear()
            readings_coords.clear()

        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()

            
    def get_next_positions(self, current_cell, visited_cells):
        x, y = current_cell
        next_poses = []

        if (x - CELLSIZE, y) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x - CELLSIZE, y)))
        if (x - CELLSIZE, y + CELLSIZE) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x - CELLSIZE, y + CELLSIZE)))
        if (x, y + CELLSIZE) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x, y + CELLSIZE)))
        if (x + CELLSIZE, y + CELLSIZE) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x + CELLSIZE, y + CELLSIZE)))
        if (x + CELLSIZE, y) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x + CELLSIZE, y)))
        if (x + CELLSIZE, y - CELLSIZE) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x + CELLSIZE, y - CELLSIZE)))
        if (x, y - CELLSIZE) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x, y - CELLSIZE)))
        if (x - CELLSIZE, y - CELLSIZE) not in visited_cells:
            next_poses.append(self.heatmapcoords_to_gzcoords((x - CELLSIZE, y - CELLSIZE)))

        return next_poses
        

# -- MAIN -- #
if __name__ == '__main__':
    iris = Drone()
    # iris.manual_algorithm()
    iris.manual_algorithm_optimized()
    rospy.spin()
