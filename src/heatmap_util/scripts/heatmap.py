#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import random

NODENAME = 'heatmap_node'
MARKER_TOPIC = 'heatmap_markers'
FRAME_ID = 'map'
LOCAL_POSE_TOPIC = '/mavros/local_position/pose'

unique_marker_id = 0
current_pos = PoseStamped()
markers = MarkerArray()
markers_pose = []
i = 0

def init_marker():
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

def set_marker_rgb(marker):
    marker.color.r = random.random()
    marker.color.g = random.random()
    marker.color.b = random.random()


def set_marker_pose(marker, x, y, z):
    # Same pose fix ??
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

def del_marker(marker, publisher):
    # Not working...
    marker.action = marker.DELETEALL
    publisher.publish(marker)

def scan_cell():
    '''
    Scans current cell, and returns markers list:

        - marker, store rviz marker to add it to markers list.
        - marker_pose, store the pose of the rviz marker previously defined.
          Then is added to the markers_pose list.
    '''
    global markers, markers_pose, unique_marker_id, current_pos, i

    if unique_marker_id == 0:
        # Origin marker placement
        marker_pose = PoseStamped()

        marker_pose.pose.position.x = 0.0
        marker_pose.pose.position.y = 0.0
        marker_pose.pose.position.z = 0.0

        marker = init_marker() 
        set_marker_rgb(marker)
        set_marker_pose(marker, 0.0, 0.0, 0.0)

        markers.markers.append(marker)
        markers_pose.append(marker_pose)
    else:
        # Inside last cell marked conditions
        inside_cell_x = (markers_pose[i].pose.position.x - 1) <= current_pos.pose.position.x <= (markers_pose[i].pose.position.x + 1)
        inside_cell_y = (markers_pose[i].pose.position.y - 1) <= current_pos.pose.position.y <= (markers_pose[i].pose.position.y + 1)

        if not inside_cell_x or not inside_cell_y:           
            marker_pose = PoseStamped()

            # z CTE
            marker_pose.pose.position.z = 0.0

            if not inside_cell_x:
                # y CTE if it's out of x range
                marker_pose.pose.position.y = markers_pose[i].pose.position.y

                # Condition to evalue if drone is inside right cell or not (left cell).
                inside_right_cell = current_pos.pose.position.x > (markers_pose[i].pose.position.x + 1)                
                if inside_right_cell:
                    marker_pose.pose.position.x = markers_pose[i].pose.position.x + 2
                else:
                    marker_pose.pose.position.x = markers_pose[i].pose.position.x - 2

            elif not inside_cell_y:
                # x CTE if it's out of y range
                marker_pose.pose.position.x = markers_pose[i].pose.position.x

                # Condition to evalue if drone is inside upper cell or not (bottom cell).
                inside_upper_cell = current_pos.pose.position.y > (markers_pose[i].pose.position.y + 1)
                if inside_upper_cell:
                    marker_pose.pose.position.y = markers_pose[i].pose.position.y + 2
                else:
                    marker_pose.pose.position.y = markers_pose[i].pose.position.y - 2

            # Not efficient!!
            if marker_pose not in markers_pose:
                i = unique_marker_id

                marker = init_marker() 
                set_marker_rgb(marker)
                set_marker_pose(marker, 
                                marker_pose.pose.position.x, 
                                marker_pose.pose.position.y, 
                                marker_pose.pose.position.z)

                markers.markers.append(marker)
                markers_pose.append(marker_pose)
            else:
                i -= 1

    return markers

# Callback
def current_pos_cb(pose):
    '''
    Local pose callback.
    '''
    global current_pos
    current_pos = pose

if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)

    markers_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=10)
    current_pos_sub = rospy.Subscriber(LOCAL_POSE_TOPIC, PoseStamped, callback = current_pos_cb)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        markers_pub.publish(scan_cell())
        rate = rospy.Rate(10)
        rate.sleep()
