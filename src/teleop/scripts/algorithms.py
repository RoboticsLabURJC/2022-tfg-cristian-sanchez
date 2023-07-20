#!/usr/bin/env python
'''
This script let us test different algorithms to solve the RF signal seeking:

    1. Manual algorithm (1st approach)
        It takes readings from current position and it's neighbors,
        then moves to the highest reading.

    2. Manual algorithm optimized
        It takes readings from current position and it's non visited neighbors,
        then moves to the highest reading.

    3. Q-Learning algorithm
        We train a model using a Q table (coords x cardinal movement),
        then we use that table to navigate to the signal source.
'''
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from teleop.msg import Px4Cmd
import actionlib
from heatmap_util.msg import GetPowerFrissAction, GetPowerFrissGoal, RvizFrissAction, RvizFrissGoal
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd

# -- CTE -- #
# Topics
LOCAL_POSE_TOPIC = '/mavros/local_position/pose'
RADIO_CONTROL_POS_TOPIC = 'radio_control/pos'
RADIO_CONTROL_CMD_TOPIC = 'radio_control/cmd'
RVIZ_HEATMAP_TOPIC = '/heatmap/map'

# Other
NODENAME = 'algorithms_node'
TOLERANCE = 0.0675
CELLSIZE = 1.0
TIMEOUT = 0.1
H = 2.0
SIGNAL_ORIGIN = (5, 3)

# Px4Cmd
IDLE = 0
TAKEOFF = 1
LAND = 2
POSITION = 3
VELOCITY = 4

# Heatmap parameters
POWER_TRANSMITTER = 1.0
FREQUENCY = 2.4 * (10**9)

# Q Learning parameters
## State generator
PWR_MAX = 10
PWR_MIN = -100
PWR_STEP = -1

## Training
MAX_EPISODES = 5000
ALPHA = 0.4
GAMMA = 0.7
EPSILON = 0.99
EPSILON_END = 0.1
OFFSET_FACTOR_A = 0.1
OFFSET_FACTOR_B = 0.2
OFFSET_CENTER = 1

## End conditions
NOT_VALID_POWER = -100
NEGATIVE_REWARD_FACTOR = 1.0
POSITIVE_REWARD_FACTOR = 1.0
OUT_OF_MAP_REWARD = -10
EXPLORATION_PERCENT = 0.2

# Testing poses
TESTING_POSES_12 = ((2,4),
                    (2,8),
                    (7,9),
                    (9,3),
                    (5,7))

TESTING_POSES_30 = ((8,5),
                    (4,22),
                    (19,23),
                    (24,6),
                    (14,15))

TESTING_POSES_Q_30 = ((14,15),)


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
        self.rvz_pub = rospy.Publisher(RVIZ_HEATMAP_TOPIC, Float64MultiArray, queue_size=10)

        # ROS action client --> RF Data server
        self.pwr_client = actionlib.SimpleActionClient('drone_friss_action', GetPowerFrissAction)
        self.pwr_client.wait_for_server()

        self.rvz_client = actionlib.SimpleActionClient('rviz_friss_action', RvizFrissAction)
        self.rvz_client.wait_for_server()

        # Goal objects to make requests
        self.rvz_goal = RvizFrissGoal()
        self.pwr_goal = GetPowerFrissGoal()

        # Requesting heatmap for Rviz and data extraction
        self.rvz_goal.origin = SIGNAL_ORIGIN
        self.rvz_goal.heatmap_config.extend([POWER_TRANSMITTER, FREQUENCY])
        self.rvz_client.send_goal(self.rvz_goal)
        self.rvz_client.wait_for_result()

        # Definition of the rviz msg
        self.rvz_msg = Float64MultiArray()
        self.rvz_msg.data = self.rvz_client.get_result().data

        # Waits the rviz subscriber to be available
        while self.rvz_pub.get_num_connections() == 0:
            rospy.sleep(1.0)

        # Publish heatmap to perform rviz representation
        self.rvz_pub.publish(self.rvz_msg)

        # Size extraction
        self.size = int(np.sqrt(len(self.rvz_msg.data))) # Square maps!

        # Setting initial poses (with the correct PoseStamped format to edit later)
        self.current_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        self.target_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        self.origin_pos = PoseStamped()

        offset_a = int(np.round(self.size * OFFSET_FACTOR_A))
        offset_b = int(np.round(self.size * OFFSET_FACTOR_B))

        # Defining training points for Q-Learning
        self.training_poses_hm = ((offset_a, offset_b),
                                  (offset_b, self.size - offset_b),
                                  (self.size - offset_b, self.size - offset_a - offset_b),
                                  (self.size - offset_a - offset_b, offset_a),
                                  (int(np.round((self.size - 1) / 2) + OFFSET_CENTER), int(np.round((self.size - 1) / 2) - OFFSET_CENTER)))
        
        
        # For plotting purposes
        self.labels = ('Time (s)', 'Iterations', 'Bad moves (%)')
        self.labels_exp = []
        self.data = []
        self.paths = []

        # Q-Learning training
        self.actions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        self.states = self.generate_coord_states(1)
        self.q_table = np.zeros((len(self.states), len(self.actions)))

        # Debugging obstacles
        self.pwr_goal.index = (22,3)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        rospy.logerr(self.pwr_client.get_result().data)

        self.pwr_goal.index = (22,4)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        rospy.logerr(self.pwr_client.get_result().data)

        self.pwr_goal.index = (22,5)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        rospy.logerr(self.pwr_client.get_result().data)

        self.pwr_goal.index = (22,6)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        rospy.logerr(self.pwr_client.get_result().data)

        self.pwr_goal.index = (21,5)
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()
        rospy.logerr(self.pwr_client.get_result().data)

        # self.show_points(TESTING_POSES_Q_30)
        # self.train_q(self.q_table, self.actions, self.states)
        
        # # Start in random pose
        # self.go_to_random_pose()


    def takeoff(self):
        '''
        Takeoff the drone.
        '''
        rospy.loginfo("Takeoff detected!")
        self.target_pos.pose.position.z = H
        while not self.h_reached(self.current_pos):
            # rospy.loginfo("Taking off...")
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
            # rospy.loginfo("Landing...")
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


    def read_only_pwr(self, heatmap_coords):
        '''
        Returns power for a heatmap coords.
        '''
        self.pwr_goal.index = heatmap_coords
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()

        return self.pwr_client.get_result().data


    def manual_algorithm(self):
        '''
        Navigates through the signal origin.
        '''
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H

        previous_goal_hm = 0
        total_it = 0
        bad_moves_it = 0

        readings = []
        readings_coords = []
        path = []

        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()

        while True:
            # Initializations            
            prev_read, current_coord_gz = self.read_pwr()
            readings.append(prev_read)
            readings_coords.append(current_coord_gz)

            # Get possible neighbors
            current_coords_hm = self.gzcoords_to_heatmapcoords(current_coord_gz)
            neighbors = self.get_valid_neighbors(current_coords_hm)
            neighbors_coords = self.get_neighbor_coords_hm(current_coords_hm, neighbors)

            path.append(current_coords_hm)
            
            # Reading power from the neighbors
            for coord_hm in neighbors_coords:
                x_gz, y_gz = self.heatmapcoords_to_gzcoords(coord_hm)
                goal_pose.pose.position.x = x_gz
                goal_pose.pose.position.y = y_gz
                self.move_to(pose=goal_pose)

                # Take readings and it's position
                current_read, current_coord_gz = self.read_pwr()
                readings.append(current_read)
                readings_coords.append(current_coord_gz)

                if current_read < prev_read:
                    bad_moves_it += 1

                total_it += 1
                prev_read = current_read
                path.append(coord_hm)

            # Get coords to make decisions
            x_goal_gz = readings_coords[readings.index(max(readings))][0]
            y_goal_gz = readings_coords[readings.index(max(readings))][1]
            current_goal_hm = self.gzcoords_to_heatmapcoords((x_goal_gz, y_goal_gz))
            
            # Move to the best position
            goal_pose.pose.position.x = x_goal_gz
            goal_pose.pose.position.y = y_goal_gz
            self.move_to(pose=goal_pose)

            # Clear arrays
            readings.clear()
            readings_coords.clear()
            
            # Update iteration counter
            total_it += 1

            # End condition, when last goal is current one and power is max
            if previous_goal_hm == current_goal_hm and self.is_goal():
                self.paths.append((path))
                break
            else:
                previous_goal_hm = current_goal_hm

        # Calcule times and land
        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()

        # To plot later
        self.data.append((elapsed_time.to_sec(), total_it, (bad_moves_it/total_it)*100))

        # Return to initial position
        rospy.loginfo("Going home...")
        self.go_home()
        self.labels_exp.append('Manual')


    def manual_algorithm_optimized(self):
        '''
        Navigates through the signal origin, avoiding visited cells.
        '''
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H

        previous_goal_hm = 0
        total_it = 0
        bad_moves_it = 0
        max_visited_size = 18

        readings = []
        readings_coords = []
        visited = []
        path = []

        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()

        while True:
            # Initializations            
            prev_read, current_coord_gz = self.read_pwr()
            readings.append(prev_read)
            readings_coords.append(current_coord_gz)

            # Store visited
            current_coords_hm = self.gzcoords_to_heatmapcoords(current_coord_gz)
            if len(visited) >= max_visited_size:
                visited.pop(0)            
            visited.append(current_coords_hm)

            path.append(current_coords_hm)

            # Get possible neighbors            
            neighbors = self.get_valid_neighbors(current_coords_hm)
            neighbors_coords = self.get_neighbor_coords_hm(current_coords_hm, neighbors, visited)
            
            # Reading power from the neighbors
            for coord_hm in neighbors_coords:
                x_gz, y_gz = self.heatmapcoords_to_gzcoords(coord_hm)
                goal_pose.pose.position.x = x_gz
                goal_pose.pose.position.y = y_gz
                self.move_to(pose=goal_pose)

                # Take readings and it's position
                current_read, current_coord_gz = self.read_pwr()
                readings.append(current_read)
                readings_coords.append(current_coord_gz)

                if current_read < prev_read:
                    bad_moves_it += 1

                total_it += 1
                prev_read = current_read

                # Store visited
                if len(visited) >= max_visited_size:
                    visited.pop(0)            
                visited.append(coord_hm)

                path.append(coord_hm)

            # Get coords to make decisions
            x_goal_gz = readings_coords[readings.index(max(readings))][0]
            y_goal_gz = readings_coords[readings.index(max(readings))][1]
            current_goal_hm = self.gzcoords_to_heatmapcoords((x_goal_gz, y_goal_gz))
            
            # Move to the best position
            goal_pose.pose.position.x = x_goal_gz
            goal_pose.pose.position.y = y_goal_gz
            self.move_to(pose=goal_pose)

            # Clear arrays
            readings.clear()
            readings_coords.clear()
            
            # Update iteration counter
            total_it += 1

            # End condition, when last goal is current one and power is max
            if previous_goal_hm == current_goal_hm and self.is_goal():
                self.paths.append((path))
                break
            else:
                previous_goal_hm = current_goal_hm

        # Calcule times and land
        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()

        # To plot later
        self.data.append((elapsed_time.to_sec(), total_it, (bad_moves_it/total_it)*100))
        
        # Return to initial position
        rospy.loginfo("Going home...")
        self.go_home()
        self.labels_exp.append('Manual Opt')


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


    def q_learning_algorithm(self, do_training=True):
        '''
        Perform Q learning algorithm, first training (constructor) and then testing in the simulation:

            - Actions are cardinal directions and it's diagonals.
            - States are defined by the power read intensity.
        '''
        self.test_q(self.q_table, self.actions, self.states)


    def get_state_idx(self, power, states):
        '''
        Returns the Q table state index for a certain power read:
        '''
        for state in states:
            max_pwr, min_pwr = state
            if max_pwr >= power > min_pwr:
                return states.index(state)

        return -1


    def generate_states(self, max_val, min_val, step):
        '''
        Returns a tuple of power intervals that represent states.
        '''
        state_limits = list(range(max_val, min_val - 1, step))
        states = []

        while len(state_limits) > 1:
            states.append((state_limits[0], state_limits[1]))
            state_limits.pop(0)

        return tuple(states)


    def generate_coord_states(self, cell_step):
        '''
        Creates all states based on the coords of the map. The step
        value, represents the resolution changes.
        '''
        states = []
        for x in range(0, self.size, cell_step):
            for y in range(0, self.size, cell_step):
                states.append((x, y))

        return tuple(states)


    def get_coord_state_idx(self, coord, states):
        '''
        Finds and returns the index if present, if not returns None.
        '''
        index = None
        for i, item in enumerate(states):
            if item == coord and index is None:
                index = i
                break

        return index


    def get_action_idx(self, epsilon, q_values):
        '''
        Returns the Q table action index depending on the epsilon value and
        the non-valid actions registered.
        '''
        random_idx = tuple(range(q_values.size))

        if np.random.random() < epsilon:
            return np.random.choice(random_idx)

        return np.argmax(q_values)


    def get_next_coords_heatmap(self, coords, action):
        '''
        Returns the resulting heatmap coords, after aplying an action.
        '''
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


    def request_new_scenario(self):
        '''
        Request new scenario to data server, putting antenna in a random pose.
        '''
        self.rvz_goal.origin = (np.random.randint(self.size), np.random.randint(self.size))
        self.rvz_client.send_goal(self.rvz_goal)
        self.rvz_client.wait_for_result()

        self.rvz_msg.data = self.rvz_client.get_result().data
        self.rvz_pub.publish(self.rvz_msg)


    def train_q(self, q_table, actions, states, alpha=ALPHA, gamma=GAMMA, eps_end=EPSILON_END):
        '''
        Fills the Q table
        '''
        # Training parameters
        ## Epsilon (percentual decrement)
        eps = EPSILON
        eps_increment = (eps - eps_end) / (MAX_EPISODES * EXPLORATION_PERCENT)

        ## Initializations
        target_coords_hm = self.rvz_goal.origin

        cumulative_reward = 0.0
        it_per_ep = 0
        episode_counter = 0
        negative_reward_counter = 0

        eps_to_plot = []
        reward_to_plot = []
        it_to_plot = []

        # Training
        ## Initial conditions        
        current_coords_hm = self.training_poses_hm[np.random.randint(len(self.training_poses_hm))]
        end_condition = False
        start_time = rospy.Time.now()
        while episode_counter < MAX_EPISODES:
            ## Sensor data extraction
            pwr_current = self.read_only_pwr(current_coords_hm)

            ## State and action indexes for Q table
            current_state_idx = self.get_coord_state_idx(current_coords_hm, states)
            current_action_idx = self.get_action_idx(eps, q_table[current_state_idx])

            ## Future state data extraction
            next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[current_action_idx])
            pwr_next = self.read_only_pwr(next_coords_hm)

            ## Gets next state index with valid power value
            ### (END CONDITION) Not possible state after performing selected action
            if pwr_next == NOT_VALID_POWER:
                end_condition = True
                reward = OUT_OF_MAP_REWARD
                error = reward - q_table[current_state_idx, current_action_idx]

            else:
                reward = pwr_next - pwr_current

                ### (END CONDITION) Target reached
                if target_coords_hm == next_coords_hm:
                    end_condition = True
                    reward *= POSITIVE_REWARD_FACTOR

                ### Otherwise
                else:
                    if reward < 0:
                        negative_reward_counter += 1
                        reward *= NEGATIVE_REWARD_FACTOR
                    else:
                        negative_reward_counter = 0
                        reward *= POSITIVE_REWARD_FACTOR

                    ### (END CONDITION) If agent does n consecutive bad actions --> end
                    if negative_reward_counter >= self.size:
                        end_condition = True
                
                reward *= 10

                ## Get next state index and calculate error
                next_state_idx = self.get_coord_state_idx(next_coords_hm, states)
                error = (reward + gamma * np.max(q_table[next_state_idx])) - q_table[current_state_idx, current_action_idx]

            ## Bellman eq to update Q table
            q_table[current_state_idx, current_action_idx] += alpha * error

            ## Update episode variables
            cumulative_reward += reward
            current_coords_hm = next_coords_hm

            ## Update iterations to plot
            it_per_ep += 1

            ## When an end condition is detected
            if end_condition:
                ### Linear epsilon
                eps = max(eps - eps_increment, eps_end)

                ### Reset position
                current_coords_hm = self.training_poses_hm[np.random.randint(len(self.training_poses_hm))]

                ### Update all plots info
                eps_to_plot.append(eps)
                reward_to_plot.append(cumulative_reward)
                it_to_plot.append(it_per_ep)

                ### Reset variables
                it_per_ep = 0
                cumulative_reward = 0
                negative_reward_counter = 0
                end_condition = False

                ### Update episode counter
                episode_counter += 1

            ## Print training percent
            percent = (episode_counter + 1) * 100 // MAX_EPISODES
            print(f"Training: {percent}%", end='\r')

        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time training: {:.6f} seconds".format(elapsed_time.to_sec()))

        ## End normal plots
        plt.subplot(3, 1, 1)
        plt.plot(eps_to_plot)
        plt.xlabel('Episodes')
        plt.ylabel('Epsilon')

        ### Moving avg technique to clean the plot
        series = pd.Series(reward_to_plot)
        moving_avg_reward = series.rolling(window=10).mean()
        moving_avg_reward_filled = moving_avg_reward.fillna(series)
        reward_to_plot = moving_avg_reward_filled.to_list()

        plt.subplot(3, 1, 2)
        plt.plot(reward_to_plot)
        plt.xlabel('Episodes')
        plt.ylabel('Cumulative reward')

        plt.subplot(3, 1, 3)
        plt.plot(it_to_plot)
        plt.xlabel('Episodes')
        plt.ylabel('Iterations')

        plt.tight_layout()
        plt.show()


    def test_q(self, q_table, actions, states):
        '''
        Test a Q table performance using gazebo drone.
        '''
        # To send positions to the drone
        previous_coords_gz = PoseStamped()
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H

        total_it = 0
        bad_moves_it = 0
        previous_pwr = PWR_MIN
        
        path = []
        coord_pair = []

        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()
        while True:
            ## Take readings
            pwr, current_coords_gz = self.read_pwr()
            current_coords_hm = self.gzcoords_to_heatmapcoords(current_coords_gz)
            path.append(current_coords_hm)

            if previous_pwr > pwr:
                bad_moves_it += 1

            ## Look for state in Q table
            state_idx = self.get_coord_state_idx(current_coords_hm, states)

            ## Get best action using Q table
            action_idx = np.argmax(q_table[state_idx])

            ## Set new goal and move
            next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[action_idx])
            next_coords_gz = self.heatmapcoords_to_gzcoords(next_coords_hm)
            
            # If first iteration store the pair of coords
            if not coord_pair:
                coord_pair.extend((current_coords_hm, next_coords_hm))
            else:
                # If next coords are revisited
                if next_coords_hm in coord_pair:
                    # If not in the highest power position candidate
                    if previous_pwr > pwr:
                        path.append(self.gzcoords_to_heatmapcoords(previous_coords_gz))
                        goal_pose.pose.position.x = previous_coords_gz[0]
                        goal_pose.pose.position.y = previous_coords_gz[1]
                        self.move_to(pose=goal_pose)

                    # (END CONDITION)
                    if self.is_goal():
                        self.paths.append((path))
                        break
                    # If not end --> reset and continue in the highest power detected.
                    else:
                        coord_pair.clear()
                        pwr, current_coords_gz = self.read_pwr()
                        current_coords_hm = self.gzcoords_to_heatmapcoords(current_coords_gz)
                        path.append(current_coords_hm)
                        state_idx = self.get_coord_state_idx(current_coords_hm, states)
                        action_idx = np.argmax(q_table[state_idx])
                        next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[action_idx])
                        next_coords_gz = self.heatmapcoords_to_gzcoords(next_coords_hm)
                        coord_pair.extend((current_coords_hm, next_coords_hm))
                else:
                    coord_pair.pop(0)
                    coord_pair.append(next_coords_hm)

            print((pwr, (current_coords_gz, current_coords_hm), (next_coords_gz, next_coords_hm), states[state_idx], actions[action_idx]))

            goal_pose.pose.position.x = next_coords_gz[0]
            goal_pose.pose.position.y = next_coords_gz[1]
            self.move_to(pose=goal_pose)

            ## Update values
            previous_pwr = pwr
            previous_coords_gz = current_coords_gz
            total_it += 1

        # Calcule times and land
        elapsed_time = rospy.Time.now() - start_time
        rospy.loginfo("Time: {:.6f} seconds".format(elapsed_time.to_sec()))
        self.land()

        # To plot later
        self.data.append((elapsed_time.to_sec(), total_it, (bad_moves_it/total_it)*100))

        # Return to initial position
        rospy.loginfo("Going home...")
        self.go_home()

        if 'Q-Learning' not in self.labels_exp:
            self.labels_exp.append('Q-Learning')
        else:
            self.labels_exp.append('Q-Learning 2')


    def go_to_random_pose(self, testing_pose=-1):
        '''
        Send the drone to a random position inside heatmap, that isn't the training ones.
        '''
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H

        if testing_pose == -1:
            while True:
                random_spawn_hm = (np.random.randint(0, self.size), np.random.randint(0, self.size))
                if random_spawn_hm not in self.training_poses_hm:
                    break
        else:
            random_spawn_hm = testing_pose

        self.takeoff()
        random_spawn_gz = self.heatmapcoords_to_gzcoords(random_spawn_hm)
        goal_pose.pose.position.x = random_spawn_gz[0]
        goal_pose.pose.position.y = random_spawn_gz[1]
        self.move_to(pose=goal_pose)
        self.origin_pos = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        self.land()


    def go_home(self):
        '''
        Send the drone to the origin position.
        '''
        self.takeoff()
        self.move_to(pose=self.origin_pos)
        self.land()


    def show_results(self):
        '''
        Plot bar graphs to see performance after running the algorithms.
        '''
        self.generate_trajectory_plot(self.paths)

        rainbow_colors = ('blue', 'orange', 'green', 'red', 'indigo', 'yellow', 'violet')
        k = 0
        for i in range(len(self.labels)):
            variable = [item[i] for item in self.data]
            ax = plt.subplot(1, len(self.labels), i + 1)
            bars = plt.bar(self.labels_exp, variable)

            ax.bar_label(bars)
            ax.set_ylabel(self.labels[i])

            # Ticks for percent plot
            if i == len(self.labels) - 1:
                ax.set_yticks(range(0, 101, 10))

            for j in range(len(bars)):
                try:
                    bars[j].set_color(rainbow_colors[k%len(self.labels_exp)])
                except IndexError:
                    k = 0
                    bars[j].set_color(rainbow_colors[k%len(self.labels_exp)])

                k += 1

        plt.tight_layout()
        plt.show()

        # self.export_to_csv()


    def is_goal(self):
        '''
        Check if all posible neighbor power are lower than current one.
        '''
        # Initializations
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H
        current_read, current_coord_gz = self.read_pwr()
        current_coords_hm = self.gzcoords_to_heatmapcoords(current_coord_gz)
        neighbors = self.get_valid_neighbors(current_coords_hm)
        neighbors_coords = self.get_neighbor_coords_hm(current_coords_hm, neighbors)
        
        # Analyzing only valid neighbors
        for coord_hm in neighbors_coords:
            # Move to the pose
            x_gz, y_gz = self.heatmapcoords_to_gzcoords(coord_hm)
            goal_pose.pose.position.x = x_gz
            goal_pose.pose.position.y = y_gz
            self.move_to(pose=goal_pose)

            # Store reading
            read = self.read_only_pwr(coord_hm)
            
            if read > current_read:
                return False

        # Return to original position
        goal_pose.pose.position.x = current_coord_gz[0]
        goal_pose.pose.position.y = current_coord_gz[1]
        self.move_to(pose=goal_pose)

        return True


    def get_valid_neighbors(self, current_coords_hm):
        '''
        Return a list of valid neighbors.
        '''
        x, y = current_coords_hm

        neighbors = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        out = []

        if x - 1 < 0:
            out.extend(('NW', 'W', 'SW'))

        if y - 1 < 0:
            out.extend(('SE', 'S', 'SW'))

        if x + 1 >= self.size:
            out.extend(('NE', 'E', 'SE'))

        if y + 1 >= self.size:
            out.extend(('NW', 'N', 'NE'))

        out = tuple(set(out))

        for item in out:
            neighbors.remove(item)

        rospy.logerr(neighbors)

        return tuple(neighbors)


    def get_neighbor_coords_hm(self, current_coords_hm, neighbors, visited=[]):
        '''
        Transform cardinal axis into heatmap coords.
        '''
        x, y = current_coords_hm
        coords_hm = []

        for neigh in neighbors:
            if neigh == 'W':
                coords_hm.append((x - 1, y))
            elif neigh == 'NW':
                coords_hm.append((x - 1, y + 1))
            elif neigh == 'N':
                coords_hm.append((x, y + 1))
            elif neigh == 'NE':
                coords_hm.append((x + 1, y + 1))
            elif neigh == 'E':
                coords_hm.append((x + 1, y))
            elif neigh == 'SE':
                coords_hm.append((x + 1, y - 1))
            elif neigh == 'S':
                coords_hm.append((x, y - 1))
            elif neigh == 'SW':
                coords_hm.append((x - 1, y - 1))

        for visited_coords in visited:
            if visited_coords in coords_hm:
                coords_hm.remove(visited_coords)

        rospy.logerr(coords_hm)
        return tuple(coords_hm)


    def generate_trajectory_plot(self, paths):
        '''
        Creates a plot that describes the trajectories behavior, 
        mixing all the paths received.

        paths must be a list of lists that include points (x, y).

        path = [((x1, y1), ...), ..., ((xn, yn), ...)]
        '''
        # Plotting the paths received
        rainbow_colors = ('blue', 'orange', 'green', 'red', 'indigo', 'yellow', 'violet')

        # Create subplots
        _, axs = plt.subplots(1, len(paths), figsize=(12, 4))

        for k, ax in enumerate(axs):
            path = paths[k]
            ax.plot(*zip(*path),
                    linestyle='dotted',
                    marker='o',
                    linewidth=1,
                    color=rainbow_colors[k % len(self.labels_exp)],
                    label=self.labels_exp[k],
                    markersize=0.5)

            for i in range(len(path) - 1):
                x, y = path[i]
                dx, dy = path[i + 1][0] - x, path[i + 1][1] - y
                ax.arrow(x, y, dx, dy,
                        head_width=0.2,
                        head_length=0.25,
                        fc=rainbow_colors[k % len(self.labels_exp)],
                        ec=rainbow_colors[k % len(self.labels_exp)])

            # Set parameters
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title(self.labels_exp[k])
            ax.set_xlim(0, self.size - 1)
            ax.set_ylim(0, self.size - 1)
            ax.set_xticks(range(0, self.size, 1))
            ax.set_yticks(range(0, self.size, 1))
            ax.grid()
            ax.legend()
            ax.set_aspect('equal')

        plt.tight_layout()
        plt.show()


    def reset_plots(self):
        '''
        Resets all variables used for plotting.
        '''
        self.labels_exp.clear()
        self.data.clear()
        self.paths.clear()


    def show_points(self, testing_points):
        '''
        Displays all points used.
        '''
        point_size = 10
        limit_size = 5

        # Training
        x_values = [point[0] for point in self.training_poses_hm]
        y_values = [point[1] for point in self.training_poses_hm]
        plt.plot(x_values, y_values, 'rx', label='Training', markersize=point_size)

        # Training
        x_values = [point[0] for point in testing_points]
        y_values = [point[1] for point in testing_points]
        plt.plot(x_values, y_values, 'go', label='Inference', markersize=point_size)

        # Signal
        plt.plot(SIGNAL_ORIGIN[0], SIGNAL_ORIGIN[1], 'b^', label='Signal', markersize=point_size)

        # Limits of the map
        plt.plot([0, 0], [0, self.size - 1], 'k-', linewidth=limit_size)
        plt.plot([0, self.size - 1], [self.size - 1, self.size - 1], 'k-', linewidth=limit_size)
        plt.plot([self.size - 1, self.size - 1], [self.size - 1, 0], 'k-', linewidth=limit_size)
        plt.plot([self.size - 1, 0], [0, 0], 'k-', linewidth=limit_size)

        # Set parameters
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Scenario map (points)')
        plt.xlim(-1, self.size)
        plt.ylim(-1, self.size)
        plt.xticks(range(0, self.size, 1))
        plt.yticks(range(0, self.size, 1))
        plt.grid()
        plt.legend(loc='upper right')

        # Display the plot
        plt.show()


    def change_pwr_q_learning(self, new_power, new_f):
        '''
        Change power values in heatmap server.
        '''
        # Requesting heatmap for Rviz and data extraction
        self.rvz_goal.heatmap_config.clear()
        self.rvz_goal.heatmap_config.extend([new_power, new_f])
        self.rvz_client.send_goal(self.rvz_goal)
        self.rvz_client.wait_for_result()

        # Definition of the rviz msg
        self.rvz_msg.data = self.rvz_client.get_result().data

        # Waits the rviz subscriber to be available
        while self.rvz_pub.get_num_connections() == 0:
            rospy.sleep(1.0)

        # Publish heatmap to perform rviz representation
        self.rvz_pub.publish(self.rvz_msg)

    
    def export_to_csv(self):
        '''
        Generates 3 csv files with n-experiment columns.
        '''
        # Separate the data into three lists        
        times = [entry[0] for entry in self.data]
        iterations = [entry[1] for entry in self.data]
        bad_moves = [entry[2] for entry in self.data]

        # Define the file paths for each CSV file
        time_csv_file = '/tmp/times.csv'
        iterations_csv_file = '/tmp/iterations.csv'
        bad_moves_csv_file = '/tmp/bad_moves.csv'

        # Write time data to the time CSV file
        with open(time_csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(times)

        # Write iterations data to the iterations CSV file
        with open(iterations_csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(iterations)

        # Write bad_moves data to the bad_moves CSV file
        with open(bad_moves_csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(bad_moves)


# -- MAIN -- #
if __name__ == '__main__':
    iris = Drone()

    # for test_pose in TESTING_POSES_Q_30:
    #     iris.go_to_random_pose(test_pose)
        
    #     # iris.manual_algorithm()
    #     # iris.manual_algorithm_optimized()

    #     iris.q_learning_algorithm()
    #     iris.change_pwr_q_learning(2, 5 * (10**9))
    #     iris.q_learning_algorithm()

    #     iris.show_results()
    #     iris.reset_plots()

    rospy.spin()
