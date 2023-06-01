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
        We train a model using a Q table (power signal x cardinal movement),
        then we use that table to navigate to the signal source.
'''
import roslib
roslib.load_manifest('heatmap_util')
import friss as fr

import random
import rospy
from geometry_msgs.msg import PoseStamped
from teleop.msg import Px4Cmd
import actionlib
from heatmap_util.msg import GetPowerFrissAction, GetPowerFrissGoal
import numpy as np
import matplotlib.pyplot as plt

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


    def get_source_coords(self):
        '''
        Returns antenna coords, only for trainning in simulation.
        '''
        self.pwr_goal.index = [0,0]
        self.pwr_client.send_goal(self.pwr_goal)
        self.pwr_client.wait_for_result()

        return self.pwr_client.get_result().source_coords


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
        visited = set()             # Visited cells, CAREFUL! no size limit.

        # Initializations
        goal_pose.pose.position.z = H
        next_pose.pose.position.z = H
        last_goal = 0               # Stores last goal in heatmap coords.

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
            goal_coords = (goal_pose.pose.position.x, goal_pose.pose.position.y)
            current_goal = self.gzcoords_to_heatmapcoords(goal_coords)
            if last_goal == current_goal:
                break
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


    def q_learning_algorithm(self):
        '''
        Perform Q learning algorithm, first training and then testing in the simulation:

            - Actions are cardinal directions and it's diagonals.
            - States are defined by the power read intensity.
        '''
        actions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        states = self.generate_states(10, -100, -1)
        q_table = np.zeros((len(states), len(actions)))

        self.train_q(q_table, actions, states)
        self.test_q(q_table, actions, states)


    def get_state_idx(self, power, states):
        '''
        Returns the Q table state index for a certain power read:
        '''
        for state in states:
            max, min = state
            if max >= power > min:
                return states.index(state)


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


    def get_action_idx(self, epsilon, q_values, not_valid=[]):
        '''
        Returns the Q table action index depending on the epsilon value and
        the non-valid actions registered.
        '''
        random_idx = list(range(q_values.size))

        for action_idx in not_valid:
            random_idx.remove(action_idx)

        if np.random.random() < epsilon:
            return random.choice(random_idx)

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


    def train_q(self, q_table, actions, states, steps=2000, alpha=0.1, gamma=0.7, eps_end=0.05):
        '''
        Fills the Q table
        '''
        # Training parameters
        ## Epsilon
        eps = 0.99
        # eps_increment = (eps - eps_end) / steps
        eps_increment = 0.01

        ## Initializations
        initial_gz_pose = rospy.wait_for_message(LOCAL_POSE_TOPIC, PoseStamped)
        initial_coords_gz = (initial_gz_pose.pose.position.x, initial_gz_pose.pose.position.y)
        target_coords_hm = self.get_source_coords()

        cumulative_reward = 0.0
        episode_counter = -1
        eps_to_plot = []
        reward_to_plot = []

        # Training
        ## Initial conditions
        end_condition = True

        ## Start interactive plots
        # plt.ion()
        # fig, (ax1, ax2) = plt.subplots(2, 1)
        # eps_line, = ax1.plot(eps_to_plot)
        # rew_line, = ax2.plot(reward_to_plot)
        # ax1.set_xlabel('Iteration')
        # ax1.set_ylabel('Epsilon')
        # ax2.set_xlabel('Iteration')
        # ax2.set_ylabel('Cumulative reward')

        for i in range(steps):
            ## Starting position
            if end_condition:
                episode_counter += 1
                negative_reward_counter = 0
                end_condition = False
                current_coords_hm = self.gzcoords_to_heatmapcoords(initial_coords_gz)

            ## Sensor data extraction
            pwr_current = self.read_only_pwr(current_coords_hm)

            ## State and action indexes for Q table
            current_state_idx = self.get_state_idx(pwr_current, states)
            current_action_idx = self.get_action_idx(eps, q_table[current_state_idx])

            ## Future state data extraction
            next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[current_action_idx])
            pwr_next = self.read_only_pwr(next_coords_hm)

            ## Gets next state index with valid power value
            ### (END CONDITION) Not possible state after performing selected action
            if pwr_next == 1:
                end_condition = True
                reward = -10
                error = reward - q_table[current_state_idx, current_action_idx]
            ### (END CONDITION) Target reached
            elif target_coords_hm == next_coords_hm:
                end_condition = True
                next_state_idx = self.get_state_idx(pwr_next, states)
                reward = 100
                error = (reward + gamma * np.max(q_table[next_state_idx])) - q_table[current_state_idx, current_action_idx]
            else:
                next_state_idx = self.get_state_idx(pwr_next, states)            
                reward = pwr_next - pwr_current
                error = (reward + gamma * np.max(q_table[next_state_idx])) - q_table[current_state_idx, current_action_idx]

                if reward < 0:
                    negative_reward_counter += 1
                else:
                    negative_reward_counter = 0

                ### (END CONDITION) If agent does n consecutive bad actions --> end
                if negative_reward_counter >= 5:
                    end_condition = True

            ## Bellman eq to update Q table
            # rospy.logerr("----------------------------------------")
            # print (q_table)
            # rospy.logwarn("***************************************")
            # print("Q state:", q_table[current_state_idx])
            # print("Previous Q value:", q_table[current_state_idx, current_action_idx])
            q_table[current_state_idx, current_action_idx] += alpha * error
            # print (q_table)
            # rospy.logerr("----------------------------------------")
            # print("Updated Q value:", q_table[current_state_idx, current_action_idx])

            # print((reward, (pwr_current, pwr_next), states[current_state_idx], actions[current_action_idx], (current_coords_hm, next_coords_hm)))
            # rospy.sleep(1)

            ## Update episode variables
            cumulative_reward += reward
            current_coords_hm = next_coords_hm

            ## Epsilon update
            ### Linear epsilon decrement
            # eps = max((eps - eps_increment, eps_end))

            ### Every 5 completed episodes, update epsilon
            if episode_counter % 5 == 0:
                eps = max(eps - eps_increment, eps_end)

            ## Update all plots info
            eps_to_plot.append(eps)
            reward_to_plot.append(cumulative_reward)

            ### Update interactive plots
            # eps_line.set_data(range(i + 1), eps_to_plot)
            # rew_line.set_data(range(i + 1), reward_to_plot)
            # ax1.relim()
            # ax1.autoscale_view()
            # ax2.relim()
            # ax2.autoscale_view()
            # fig.canvas.draw()
            # plt.pause(0.01)

            ## Print training percent
            percent = (i + 1) * 100 // steps
            print(f"Training: {percent}%", end='\r')

        ## End interactive plots
        # plt.ioff()
        # plt.show()

        ## End normal plots
        plt.subplot(2, 1, 1)
        plt.plot(eps_to_plot)
        plt.xlabel('Iteration')
        plt.ylabel('Epsilon')

        plt.subplot(2, 1, 2)
        plt.plot(reward_to_plot)
        plt.xlabel('Iteration')
        plt.ylabel('Cumulative reward')

        plt.tight_layout()
        plt.show()


    def test_q(self, q_table, actions, states):
        '''
        Test a Q table performance using gazebo drone.
        '''
        # To send positions to the drone
        goal_pose = PoseStamped()
        goal_pose.pose.position.z = H

        # Start algorithm
        self.takeoff()
        start_time = rospy.Time.now()
        while True:
            ## Take readings
            pwr, current_coords_gz = self.read_pwr()
            current_coords_hm = self.gzcoords_to_heatmapcoords(current_coords_gz)

            ## Look for state in Q table
            state_idx = self.get_state_idx(pwr, states)

            ## End condition (when the signal is HIGH --> end)
            if states[state_idx] == (-15, -20):
                break

            ## Get best action using Q table
            action_idx = np.argmax(q_table[state_idx])
            
            ## Set new goal and move
            next_coords_hm = self.get_next_coords_heatmap(current_coords_hm, actions[action_idx])
            next_coords_gz = self.heatmapcoords_to_gzcoords(next_coords_hm)

            print((pwr, (current_coords_gz, current_coords_hm), (next_coords_gz, next_coords_hm), states[state_idx], actions[action_idx]))

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

    # iris.manual_algorithm()
    # iris.manual_algorithm_optimized()
    iris.q_learning_algorithm()

    rospy.spin()
