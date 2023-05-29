#!/usr/bin/env python
'''
This script let us test different algorithms to solve the RF signal seeking:

    1. Manual algorithm (1st approach)
        It takes readings from current position and it's neighbors,
        then moves to the highest reading.
    
    2. Manual algorithm optimized
        It takes readings from current position and it's non visited neighbors,
        then moves to the highest reading.
'''
import rospy
from geometry_msgs.msg import PoseStamped
from teleop.msg import Px4Cmd
import actionlib
from heatmap_util.msg import GetPowerFrissAction, GetPowerFrissGoal
import numpy as np
import random

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


class Drone:
    '''
    Drone class:
        - It connects to the data server to receive power readings.
        - It commands the drone to find the RF signal using different approaches:
            1. Manual algorithm (1st approach)
            2. Manual algorithm optimized
            3. Q-Learning
    '''
    def __init__(self):
        '''
        Constructor, it initialize the drone and it's attributes.
        '''
        rospy.init_node(NODENAME)
        # ROS Publishers
        self.pos_pub = rospy.Publisher(RADIO_CONTROL_POS_TOPIC, PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher(RADIO_CONTROL_CMD_TOPIC, Px4Cmd, queue_size=10)

        # ROS action client --> RF Data server
        self.pwr_client = actionlib.SimpleActionClient('drone_friss_action', GetPowerFrissAction)
        self.pwr_client.wait_for_server()

        # Attributes (power, heatmap size, current position and target position)
        self.pwr_goal = GetPowerFrissGoal()
        self.pwr_goal.index = [0, 0]
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()

        self.size = self.pwr_client.get_result().size
        self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        self.target_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)


    def takeoff(self):
        '''
        Takeoff the drone.
        '''
        rospy.loginfo("Takeoff detected!")
        self.target_pos.pose.position.z = H
        while not self.h_reached(self.current_pos):
            rospy.loginfo("Taking off...")            
            self.pos_pub.publish(self.target_pos)
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Takeoff OK!")


    def land(self):
        '''
        Land the drone.
        '''
        cmd = Px4Cmd()
        cmd.cmd = LAND    
        self.target_pos.pose.position.z = 0.0
        while not self.h_reached(self.current_pos):
            rospy.loginfo("Landing...")   
            self.cmd_pub.publish(cmd)         
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)


    def h_reached(self, current_pose):
        '''
        Condition of height in gz reached.
        '''
        current_z = current_pose.pose.position.z
        target_z = self.target_pos.pose.position.z
        reached_z = current_z - TOLERANCE < target_z < current_z + TOLERANCE

        return reached_z


    def centered(self, current_pose):
        '''
        Condition of coords in gz reached.
        '''
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        target_x = self.target_pos.pose.position.x
        target_y = self.target_pos.pose.position.y

        reached_x = current_x - TOLERANCE < target_x < current_x + TOLERANCE
        reached_y = current_y - TOLERANCE < target_y < current_y + TOLERANCE

        return reached_x and reached_y


    def move_to(self, cmd="", pose=PoseStamped()):
        '''
        Move command, it allows the drone to navigate cell to cell in the x, y axis.
        '''
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
            self.pos_pub.publish(self.target_pos)
            self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)

        rospy.loginfo("Reached!")


    def gzcoords_to_heatmapcoords(self, gzcoords):
        '''
        Transform gazebo coords to heatmap coords.
        '''
        gz_x, gz_y = gzcoords        
        heat_x = round((self.size / 2) - 1 - gz_x)
        heat_y = round((self.size / 2) - 1 - gz_y)
        return (heat_x, heat_y)


    def heatmapcoords_to_gzcoords(self, hmcoords):
        '''
        Transform heatmap coords to gazebo coords.
        '''
        hm_x, hm_y = hmcoords        
        gz_x = (self.size / 2) - 1 - hm_x
        gz_y = (self.size / 2) - 1 - hm_y
        return (gz_x, gz_y)

    
    def read_pwr(self):
        '''
        Returns the power reading and the coords where it was taken.
        '''
        current_pose = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        current_coords = (current_pose.pose.position.x, current_pose.pose.position.y)
        self.pwr_goal.index = self.gzcoords_to_heatmapcoords(current_coords)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        return (self.pwr_client.get_result().data, current_coords)


    def manual_algorithm(self):
        '''
        Navigates through the signal origin.
        '''
        readings = []               # Power reads per iteration
        readings_coords = []        # Power coords in gz
        hm_coords = []              # Power coords in heatmap
        hm_coords_prev = []         # Previous power coords in heatmap
        goal_pose = PoseStamped()   # Goal per iteration

        # Initializations
        signal_found = False
        goal_pose.pose.position.z = H
        neighbors = ("FRONT", "RIGHT", "BACK", "BACK", "LEFT", "LEFT", "FRONT", "FRONT")

        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()

        while not signal_found:
            # Read data in current cell and move to next position
            for neigh in neighbors:
                read, coord = self.read_pwr()
                readings.append(read)
                readings_coords.append(coord)
                hm_coords.append(self.gzcoords_to_heatmapcoords(coord))
                self.move_to(neigh)

            # Read data in last cell
            read, coord = self.read_pwr()
            readings.append(read)
            readings_coords.append(coord)
            hm_coords.append(self.gzcoords_to_heatmapcoords(coord))

            # Look for the max power value readed position and move there
            goal_pose.pose.position.x = readings_coords[readings.index(max(readings))][0]
            goal_pose.pose.position.y = readings_coords[readings.index(max(readings))][1]
            self.move_to("GOAL", pose=goal_pose)

            # End condition (1st approach), if drone repeats movement patron --> land
            if len(hm_coords_prev) == 0:
                hm_coords_prev = hm_coords.copy()
            else:
                for i in range(len(hm_coords)):
                    if hm_coords[i] != hm_coords_prev[i]:
                        signal_found = False
                        hm_coords_prev = hm_coords.copy()
                        break
                    else:
                        signal_found = True

            # Clear arrays
            readings.clear()
            readings_coords.clear()
            hm_coords.clear()

        # Calcule times and land
        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()


    def manual_algorithm_optimized(self):
        '''
        Navigates through the signal origin, avoiding visited cells.
        '''
        readings = []               # Power reads per iteration
        readings_coords = []        # Power coords in gz
        goal_pose = PoseStamped()   # Target pose to move
        next_pose = PoseStamped()   # Next pose to move
        visited = set()             # Visited cells, CAREFUL! no size limit stablish, so in big maps drone will store a lot of data.

        # Initializations
        goal_pose.pose.position.z = H
        next_pose.pose.position.z = H
        last_goal = 0               # Stores last goal in heatmap coords, initialize with whatever different to (x, y)

        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()

        while True:
            # Take readings in current cell and add to visited
            read, coord = self.read_pwr()
            hm_coords = self.gzcoords_to_heatmapcoords(coord)
            readings.append(read)
            readings_coords.append(coord)
            visited.add(hm_coords)
            
            # Obtain next path to avoid visited cells (gz coords)
            path = self.get_next_positions(hm_coords, visited)

            # Moves to the next position in the path and take readings, adding new cells to visited
            for x, y in path:
                next_pose.pose.position.x = x
                next_pose.pose.position.y = y
                self.move_to("NEXT", pose=next_pose)
                read, coord = self.read_pwr()
                readings.append(read)
                readings_coords.append(coord)
                visited.add(self.gzcoords_to_heatmapcoords(coord))

            # Look for the max power value readed position and move there
            goal_pose.pose.position.x = readings_coords[readings.index(max(readings))][0]
            goal_pose.pose.position.y = readings_coords[readings.index(max(readings))][1]
            self.move_to("GOAL", pose=goal_pose)

            # End condition, if previous goal it's the same than current goal --> land
            # We look in the drones heatmap coords to avoid decimals problems.
            current_goal = self.gzcoords_to_heatmapcoords((goal_pose.pose.position.x, goal_pose.pose.position.y))
            if last_goal == current_goal:
                break
            else:
                last_goal = current_goal

            # Clear arrays
            readings.clear()
            readings_coords.clear()

        # Calcule times and land
        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()


    def get_next_positions(self, current_cell, visited_cells):
        '''
        Check if neighbors of current_cell are visited or not, returns a list of non visited.
        '''
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
    
    
    
    def q_learning_algorithm (self):
        # actions = ["N", "E", "S", "W"]
        actions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        states = ["HIGH", "MID", "LOW"]

        steps = 2000
        alpha = 0.05
        gamma = 0.7

        # Epsilon
        eps = 1.0
        eps_end = 0.05
        eps_increment = (eps - eps_end) / steps

        q_table = np.zeros((len(states), len(actions)))
        initial_gz_pose = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        initial_coords_gz = (initial_gz_pose.pose.position.x, initial_gz_pose.pose.position.y)

        current_coords_hm = self.gzcoords_to_heatmapcoords(initial_coords_gz)
        
        # self.pwr_goal.index = [0,0]
        # self.pwr_client.send_goal(self.pwr_goal)
        # self.pwr_client.wait_for_result()

        # print("--------------------------------")
        # print(self.pwr_client.get_result().data)
        # print("--------------------------------")

        # self.pwr_goal.index = [1,1]
        # self.pwr_client.send_goal(self.pwr_goal)
        # self.pwr_client.wait_for_result()

        # print("--------------------------------")
        # print(self.pwr_client.get_result().data)
        # print("--------------------------------")

        # self.pwr_goal.index = [2,2]
        # self.pwr_client.send_goal(self.pwr_goal)
        # self.pwr_client.wait_for_result()

        # print("--------------------------------")
        # print(self.pwr_client.get_result().data)
        # print("--------------------------------")

        # self.pwr_goal.index = [0,2]
        # self.pwr_client.send_goal(self.pwr_goal)
        # self.pwr_client.wait_for_result()

        # print("--------------------------------")
        # print(self.pwr_client.get_result().data)
        # print("--------------------------------")

        # self.pwr_goal.index = [2,0]
        # self.pwr_client.send_goal(self.pwr_goal)
        # self.pwr_client.wait_for_result()

        # print("--------------------------------")
        # print(self.pwr_client.get_result().data)
        # print("--------------------------------")

        # self.pwr_goal.index = [-1,3]
        # self.pwr_client.send_goal(self.pwr_goal)
        # self.pwr_client.wait_for_result()

        # print("--------------------------------")
        # print(self.pwr_client.get_result().data)
        # print("--------------------------------")

        print(q_table)
        rospy.loginfo("Training...")
        # Training
        end_condition = False
        not_valid_action_idxs = []
        for i in range(steps):
            ## Starting position
            if end_condition or (i + 1) % (100) == 0:
                end_condition = False
                current_coords_hm = self.gzcoords_to_heatmapcoords(initial_coords_gz)

            ## Sensor data extraction   
            self.pwr_goal.index = current_coords_hm
            self.pwr_client.send_goal(self.pwr_goal)
            self.pwr_client.wait_for_result()
            
            pwr_current = self.pwr_client.get_result().data

            ## State and action definition
            current_state_idx = self.get_state_idx(pwr_current, states)
            current_action_idx = self.get_action_idx(eps, q_table[current_state_idx])

            ## Future state data extraction
            next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[current_action_idx])

            self.pwr_goal.index = next_coords_hm
            self.pwr_client.send_goal(self.pwr_goal)
            self.pwr_client.wait_for_result()
            
            pwr_next = self.pwr_client.get_result().data

            ## End condition (if power == 1 means is out of limits, see rf_data_server.py)
            while pwr_next == 1:
                # print(current_action_idx)
                not_valid_action_idxs.append(current_action_idx)
                current_action_idx = self.get_action_idx(eps, q_table[current_state_idx], not_valid_action_idxs)
                
                next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[current_action_idx])

                self.pwr_goal.index = next_coords_hm
                self.pwr_client.send_goal(self.pwr_goal)
                self.pwr_client.wait_for_result()

                pwr_next = self.pwr_client.get_result().data
                # print(current_coords_hm, not_valid_action_idxs, current_action_idx, pwr_next)

            next_state_idx = self.get_state_idx(pwr_next, states)
            
            # (np.round(pwr_current, 3) == np.round(pwr_next, 3)

            # if next_state_idx > current_state_idx: # Switch to a lower signal state --> end
            #     # end_condition = True
            #     reward = -10
            # elif next_state_idx < current_state_idx:
            #     reward = 10
            # else:
            #     reward = pwr_next - pwr_current
                
                # if (pwr_next - pwr_current) > 0: # Power increase (negative scale)
                #     reward = 1
                # else:
                #     reward = -1

            ## Bellman
            reward = pwr_next - pwr_current
            error = (reward + gamma * np.max(q_table[next_state_idx])) - q_table[current_state_idx, current_action_idx]
            q_table[current_state_idx, current_action_idx] += alpha * error

            # print(((current_coords_hm, states[current_state_idx], actions[current_action_idx]),(next_coords_hm, states[next_state_idx]), (pwr_current, pwr_next), reward))
            # rospy.sleep(1)

            ## Update next state and epsilon
            current_coords_hm = next_coords_hm
            eps = max((eps + eps_increment, eps_end))
            not_valid_action_idxs.clear()

            ## Debuggin progress bar
            if (i + 1) % (steps / 100) == 0:
                progress = (i + 1) / steps
                progress_percentage = int(progress * 100)                
                progress_bar = '[' + '#' * (progress_percentage // 10) + ' ' * ((100 - progress_percentage) // 10) + ']'
                print(f'Progress: {progress_bar} {progress_percentage}% ', end='\r')

        rospy.loginfo("Training OK!")
        print(q_table)

        self.test_q(q_table, actions, states)


    def get_state_idx(self, power, states):
        if power >= -20:
            state = "HIGH"
        elif -20 > power >= -25:
            state = "MID"
        else:
            state = "LOW"

        return states.index(state)
        
    def get_action_idx(self, epsilon, q_values, not_valid=[]):
        random_idx = list(range(q_values.size))

        for action_idx in not_valid:
            random_idx.remove(action_idx)

        if np.random.random() < epsilon:
            return random.choice(random_idx)
        else:
            return np.argmax(q_values)
        
    def get_next_coords_heatmap(self, coords, action):
        x, y = coords # Heatmap coords

        if action == "N":
            new_coords = (x, y + 1)
        elif action == "E":
            new_coords = (x + 1, y)
        elif action == "S":
            new_coords = (x, y - 1)
        elif action == "W":
            new_coords = (x - 1, y)

        elif action == "NE":
            new_coords = (x + 1, y + 1)
        elif action == "SE":
            new_coords = (x + 1, y - 1)
        elif action == "NW":
            new_coords = (x - 1, y + 1)
        elif action == "SW":
            new_coords = (x - 1, y - 1)

        return new_coords


    def test_q(self, q_table, actions, states):
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H
        
        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()

        while True:
            ## Take readings
            pwr, current_coords_gz = self.read_pwr()
            current_coords_hm = self.gzcoords_to_heatmapcoords(current_coords_gz)

            ## Look for best action inside Q table
            state_idx = self.get_state_idx(pwr, states)

            if states[state_idx] == "HIGH":
                break

            action_idx = np.argmax(q_table[state_idx])
            
            ## Set new goal
            next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[action_idx])
            next_coords_gz = self.heatmapcoords_to_gzcoords(next_coords_hm)

            goal_pose.pose.position.x = next_coords_gz[0]
            goal_pose.pose.position.y = next_coords_gz[1]

            self.move_to(pose=goal_pose)

        # Calcule times and land
        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()


# -- MAIN -- #
if __name__ == '__main__':
    iris = Drone()
    iris.q_learning_algorithm()
    # iris.manual_algorithm()
    # iris.manual_algorithm_optimized()
    rospy.spin()
