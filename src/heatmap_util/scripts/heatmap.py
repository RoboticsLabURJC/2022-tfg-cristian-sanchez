#! /usr/bin/env python
'''
HEATMAP

This app is made for debbuging purposes, it draws
markers inside rviz, following px4 trayectory.

Is made to work with center 2 center control script.
'''
import random
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

# Importing friss
import friss as fr
import matplotlib as mpl
import numpy as np

# -- CTE -- #
NODENAME = 'heatmap_node'
MARKER_TOPIC = 'heatmap_markers'
FRAME_ID = 'map'
LOCAL_POSE_TOPIC = '/mavros/local_position/pose'

# -- CONSTANTS -- #
DEFAULT_WORLD_SIZE = (10, 10)   # size range --> 10 <= (a x a) <= 100
DEFAULT_ORIGIN = (0, 0)         # Origin of the signal

# -- GLOBAL VARIABLES -- #
unique_marker_id = 0
current_pos = PoseStamped()
markers = MarkerArray()
i = 0
poses = []

def init_marker():
    '''
    Init default rviz marker parameters
    '''
    global unique_marker_id

    marker = Marker()

    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.id = unique_marker_id
    marker.ns = 'Cube'
    marker.action = marker.ADD
    marker.type = marker.CUBE
    marker.lifetime = rospy.Duration(0.0)
    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    # marker pose
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    # marker scale
    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 0.001
    # marker color
    # a (alpha) must be non-zero
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    unique_marker_id += 1

    return marker

def set_marker_random_rgb(marker):
    '''
    Set rviz marker color, in this case random
    '''
    marker.color.r = random.random()
    marker.color.g = random.random()
    marker.color.b = random.random()

def set_marker_rgb(marker, red, green, blue):
    '''
    Set rviz marker color, in this case random
    '''
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue


def set_marker_pose(marker, x_pos, y_pos, z_pos):
    '''
    Set rviz marker position
    '''
    marker.pose.position.x = x_pos
    marker.pose.position.y = y_pos
    marker.pose.position.z = z_pos

def scan_cell():
    '''
    Scans current cell, and returns markers list:

        - marker, store rviz marker to add it to markers list.
        - marker_pose, store the x, y coords of the rviz marker previously defined.
          Then is added to the poses list.
    '''
    global markers, unique_marker_id, current_pos, i, poses

    if unique_marker_id == 0:
        # Origin marker placement
        marker = init_marker()
        set_marker_random_rgb(marker)
        set_marker_pose(marker, 0, 0, 0)

        markers.markers.append(marker)
        poses.append((0, 0))
    else:
        # Last x, y visited
        last_visited_x = poses[i][0]
        last_visited_y = poses[i][1]

        # Inside last cell marked conditions
        inside_cell_x = (last_visited_x - 1) <= current_pos.pose.position.x <= (last_visited_x + 1)
        inside_cell_y = (last_visited_y - 1) <= current_pos.pose.position.y <= (last_visited_y + 1)

        if not inside_cell_x or not inside_cell_y:
            if not inside_cell_x:
                # y CTE if x is out of range
                new_y = last_visited_y
                # Condition to evalue if drone is inside right cell or not (left cell).
                inside_right_cell = current_pos.pose.position.x > (last_visited_x + 1)
                if inside_right_cell:
                    new_x = last_visited_x + 2
                else:
                    new_x = last_visited_x - 2

            elif not inside_cell_y:
                # x CTE if y is out of range
                new_x = last_visited_x
                # Condition to evalue if drone is inside upper cell or not (bottom cell).
                inside_upper_cell = current_pos.pose.position.y > (last_visited_y + 1)
                if inside_upper_cell:
                    new_y = last_visited_y + 2
                else:
                    new_y = last_visited_y - 2

            if (new_x, new_y) not in poses:
                marker = init_marker()
                set_marker_random_rgb(marker)
                set_marker_pose(marker, new_x, new_y, 0)

                markers.markers.append(marker)
                poses.append((new_x, new_y))

            i = poses.index((new_x, new_y))

    return markers

def current_pos_cb(pose):
    '''
    Local pose callback.
    '''
    global current_pos
    current_pos = pose

def expand_rf_signal(origin):
    global markers

    my_model = fr.Friss(world_sz=DEFAULT_WORLD_SIZE, power_tras=0.05, gain_tras=100, gain_recv=1000)
    #data = normalize_2d(my_model.model_power_signal(origin)) # [0-1]
    data = my_model.model_power_signal(origin)
    print(data)

    afmhot = mpl.colormaps['afmhot'].resampled(8)

    rows, cols = data.shape

    for x in range(rows):
        for y in range(cols):
            r, g, b, _ = afmhot(data[x, y])
            marker = init_marker()
            set_marker_rgb(marker, r, g, b)
            set_marker_pose(marker, x, y, 0)

            markers.markers.append(marker)
            poses.append((x, y))

            
def normalize_2d(matrix):
    norm = np.linalg.norm(matrix)
    matrix = matrix/norm
    return matrix

# -- MAIN -- #
if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)

    # ROS pub/sub
    markers_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=10)
    # current_pos_sub = rospy.Subscriber(LOCAL_POSE_TOPIC, PoseStamped, callback = current_pos_cb)

    expand_rf_signal(DEFAULT_ORIGIN)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # markers_pub.publish(scan_cell())
        markers_pub.publish(markers)
        # print(".")
        rate = rospy.Rate(10)
        rate.sleep()
