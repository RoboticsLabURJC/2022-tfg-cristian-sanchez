#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray

NODENAME = 'heatmap_node'
MARKER_TOPIC = 'heatmap_markers'
FRAME_ID = 'map'

unique_marker_id = 0

def init_marker():
    global unique_marker_id
    unique_marker_id += 1

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

    return marker

def set_marker_rgb(marker, R, G, B):  
    marker.color.r = R
    marker.color.g = G
    marker.color.b = B

def set_marker_pose(marker, x, y, z):
    # Same pose fix ??
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

def del_marker(marker, publisher):
    # Not working...
    marker.action = marker.DELETEALL
    publisher.publish(marker)

if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)
    markers_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=10)
    rate = rospy.Rate(10)

    markers = MarkerArray()

    marker = init_marker()
    set_marker_rgb(marker, 0.0, 1.0, 0.0)
    set_marker_pose(marker, 2.0, 0.0, 0.0)

    marker_2 = init_marker()
    set_marker_rgb(marker_2, 1.0, 0.0, 0.0)
    set_marker_pose(marker_2, 0.0, 0.0, 0.0)

    markers.markers.append(marker)
    markers.markers.append(marker_2)

    while not rospy.is_shutdown():
        markers_pub.publish(markers)

        rate = rospy.Rate(10)
        rate.sleep()

    # del_marker(marker, marker_pub)

    rospy.loginfo('rospy shutdown')
