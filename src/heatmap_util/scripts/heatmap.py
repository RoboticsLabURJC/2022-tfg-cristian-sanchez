#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

NODENAME = 'heatmap_node'
MARKER_TOPIC = 'heatmap_markers'
FRAME_ID = 'map'

def init_marker():
    marker = Marker()

    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
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
    # a (alpha) must be non-zero
    marker.color.a = 1.0

    return marker

def set_marker_rgb(marker, R, G, B):
    # marker colors   
    marker.color.r = R
    marker.color.g = G
    marker.color.b = B


if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)
    marker_pub = rospy.Publisher(MARKER_TOPIC, Marker, queue_size=10)
    rate = rospy.Rate(10)

    marker = init_marker()
    set_marker_rgb(marker, 0.0, 1.0, 0.0)

    while not rospy.is_shutdown():
        # publish msg
        marker_pub.publish(marker)
        rate = rospy.Rate(10)
        rate.sleep()

    rospy.loginfo('rospy shutdown')
