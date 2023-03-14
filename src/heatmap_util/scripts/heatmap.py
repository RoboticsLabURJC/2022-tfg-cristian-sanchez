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
DEFAULT_ORIGIN = (5, 5)         # Origin of the signal
WORLD_SIZE = (100, 100)

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

def set_marker_rgb(marker, red, green, blue, alpha):
    '''
    Set rviz marker color, in this case random
    '''
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha


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

    my_model = fr.Friss(world_sz=WORLD_SIZE)
    # data = normalize_2d(my_model.model_power_signal(origin)) # [0-1]
    data = my_model.model_power_signal(origin)
    # print(data)

    data_min = np.min(data)
    data_max = np.max(data)

    # print(translate(data[0,0], data_min, data_max, 0, 765))

    # afmhot = mpl.colormaps['afmhot'].resampled(1024)
    # afmhot = mpl.colormaps['afmhot']

    rows, cols = data.shape

    for x in range(rows):
        for y in range(cols):
            # r, g, b, a = afmhot(data[x, y])

            # r, g, b = extract_rgb(translate(data[x, y], data_min, data_max, 0, 765))
            # r, g, b = extract_rgb(translate(data[x, y], data_min, data_max, 0, WORLD_SIZE[0]*WORLD_SIZE[1]))
            r, g, b = extract_rgb(translate(data[x, y], -40, 40, 0, WORLD_SIZE[0]*WORLD_SIZE[1]))
            marker = init_marker()
            set_marker_rgb(marker, r, g, b, 0.5)
            set_marker_pose(marker, x, y, 0)

            markers.markers.append(marker)
            poses.append((x, y))

            
def normalize_2d(matrix):
    norm = np.linalg.norm(matrix)
    matrix = matrix/norm
    return matrix

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return int(np.round(rightMin + (valueScaled * rightSpan)))

def extract_rgb(value):
    surf = WORLD_SIZE[0]*WORLD_SIZE[1]

    it = 12
    while (it > 0):
        if surf*((it-1)/12) <= value <= surf*(it/12):
            break
        it -= 1

    r, g, b = (0,0,0)
    if it == 12:
        r = translate(value, 0, surf, 230, 242)
        g = translate(value, 0, surf, 218, 242)
        b = translate(value, 0, surf, 138, 159)

    elif it == 11:
        r = translate(value, 0, surf, 217, 230)
        g = translate(value, 0, surf, 194, 218)
        b = translate(value, 0, surf, 120, 138)

    elif it == 10:
        r = translate(value, 0, surf, 204, 217)
        g = translate(value, 0, surf, 171, 194)
        b = translate(value, 0, surf, 102, 120)

    elif it == 9:
        r = translate(value, 0, surf, 189, 204)
        g = translate(value, 0, surf, 149, 171)
        b = translate(value, 0, surf, 87, 102)

    elif it == 8:
        r = translate(value, 0, surf, 189, 174)
        g = translate(value, 0, surf, 149, 127)
        b = translate(value, 0, surf, 87, 72)

    elif it == 7:
        r = translate(value, 0, surf, 158, 174)
        g = translate(value, 0, surf, 106, 149)
        b = translate(value, 0, surf, 59, 72)

    elif it == 6:
        r = translate(value, 0, surf, 141, 158)
        g = translate(value, 0, surf, 85, 106)
        b = translate(value, 0, surf, 48, 59)

    elif it == 5:
        r = translate(value, 0, surf, 124, 141)
        g = translate(value, 0, surf, 65, 85)
        b = translate(value, 0, surf, 37, 48)

    elif it == 4:
        r = translate(value, 0, surf, 106, 124)
        g = translate(value, 0, surf, 46, 65)
        b = translate(value, 0, surf, 27, 37)

    elif it == 3:
        r = translate(value, 0, surf, 88, 106)
        g = translate(value, 0, surf, 26, 46)
        b = translate(value, 0, surf, 18, 27)

    elif it == 2:
        r = translate(value, 0, surf, 70, 88)
        g = translate(value, 0, surf, 5, 26)
        b = translate(value, 0, surf, 5, 18)

    else:
        r = translate(value, 0, surf, 0, 70)
        g = translate(value, 0, surf, 0, 5)
        b = translate(value, 0, surf, 0, 5)

    # print(it, (r,g,b), value)   

    return (r, g, b)


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
