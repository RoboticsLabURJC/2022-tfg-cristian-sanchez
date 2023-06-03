#! /usr/bin/env python
'''
PATH MARKER

This app is made for debbuging purposes, it draws
markers inside rviz, following px4 trajectory.

Is made to work with center 2 center control script.
'''
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

# -- CTE -- #
NODENAME = 'path_node'
MARKER_TOPIC = 'path_markers'
FRAME_ID = 'map'
OFFSET_TOPIC = '/rf_data_server/offset_pose'
LINE_LENGTH = 200   # Max line length
FREQ = 10           # Node frequency

# -- GLOBAL VARIABLES -- #
unique_marker_id = 0
current_pos = PoseStamped()

def init_marker(marker):
    '''
    Init default rviz marker parameters
    '''
    global unique_marker_id

    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    marker.id = unique_marker_id
    marker.ns = 'Line Strip'
    marker.action = marker.ADD
    marker.type = marker.LINE_STRIP
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
    marker.scale.x = 0.05
    marker.scale.y = 0.0
    marker.scale.z = 0.0
    # marker color
    # a (alpha) must be non-zero
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    unique_marker_id += 1

def draw_path(marker):
    '''
    This method draws path of max 100 points.
    '''
    line_point = Point()

    # Empty list condition
    if not marker.points:
        line_point.x = 0
        line_point.y = 0
        line_point.z = current_pos.pose.position.z
        marker.points.append(line_point)

    line_point.x = current_pos.pose.position.x
    line_point.y = current_pos.pose.position.y
    line_point.z = current_pos.pose.position.z
    marker.points.append(line_point)

    # Full list condition
    if len(marker.points) > LINE_LENGTH:
        marker.points.pop(0)

    return marker

def current_pos_cb(pose):
    '''
    Local pose callback.
    '''
    global current_pos
    current_pos = pose

# -- MAIN -- #
if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)

    # ROS pub/sub
    markers_pub = rospy.Publisher(MARKER_TOPIC, Marker, queue_size=10)
    current_pos_sub = rospy.Subscriber(OFFSET_TOPIC, PoseStamped, callback = current_pos_cb)

    line_marker = Marker()
    init_marker(line_marker)

    rate = rospy.Rate(FREQ)
    while not rospy.is_shutdown():
        markers_pub.publish(draw_path(line_marker))
        rate = rospy.Rate(10)
        rate.sleep()
