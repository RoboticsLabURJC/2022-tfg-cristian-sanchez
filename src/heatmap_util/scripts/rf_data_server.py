#! /usr/bin/env python
'''
RF HEATMAP

This node manage the friss data simulated,
it works like a server that waits for requests.

It distinguishes between rviz data and drone
requests, because one is for debugging purposes,
and the other is for the algorithms.
'''
import roslib
roslib.load_manifest('heatmap_util')
import rospy
import friss as fr
import actionlib
from heatmap_util.msg import RvizFrissAction, RvizFrissResult, GetPowerFrissAction, GetPowerFrissResult
import numpy as np
from geometry_msgs.msg import PoseStamped

# -- CTE -- #
NODENAME = 'heatmap_data_server_node'

class MyActionServer:
    def __init__(self):
        self._rviz_server = actionlib.SimpleActionServer('rviz_friss_action', RvizFrissAction, self.__response_rviz, False)
        self._power_server = actionlib.SimpleActionServer('drone_friss_action', GetPowerFrissAction, self.__response_drone, False)

        self._size = rospy.get_param('map_size')
        self._rviz_size = [self._size[0], self._size[1]]
        self._origin = rospy.get_param('radio_origin')
        self._res = rospy.get_param('resolution')

        self._rviz_server.start()
        self._power_server.start()

        self._rviz_result = RvizFrissResult()
        self._power_result = GetPowerFrissResult() 

    def __response_rviz(self, goal):
        rospy.loginfo("rviz data requested!")
        if goal.get_data:
            self._rviz_result.data = self.__get_rviz_data()
            self._rviz_result.size = self._rviz_size[0]
            self._rviz_server.set_succeeded(self._rviz_result)
            rospy.loginfo("Response sent to rviz!")
        else:
            rospy.logwarn("get_data set to false, aborting...")
            self._rviz_server.set_aborted()

    def __response_drone(self, goal):
        # rospy.loginfo("drone data requested!")

        # Checks if index is not valid ([x,y] positive integers or 0s)
        if len(goal.index) != 2 and not all(isinstance(x, int) and x>=0 for x in goal):
            rospy.logwarn("wrong format for index, please introduce [x, y]")
            self._power_server.set_aborted()
        else:
            self._power_result.data = self.__get_drone_data(goal.index)
            self._power_result.size = self._size[0]
            self._power_result.source_coords = self._origin
            self._power_server.set_succeeded(self._power_result)
            # rospy.loginfo("Response sent to drone!")

    def __get_rviz_data(self):
        model = fr.Friss(world_sz=self._rviz_size, resolution=self._res)
        data = model.model_power_signal(self._origin)
        flip_data = np.rot90(data, k=-1)
        flip_data = np.flip(flip_data, axis=1)
        
        return list(flip_data.flatten())
    
    def __get_drone_data(self, pose):
        x, y = pose
        if x < 0 or y < 0:
            return 1
        
        model = fr.Friss(world_sz=self._size)
        data = model.model_power_signal(self._origin)
        
        try:
            return data[x, y]
        except IndexError:
            return 1

def pose_callback(msg):
    offset_pose.header = msg.header
    offset_pose.pose.position.x = msg.pose.position.x + 0.5
    offset_pose.pose.position.y = msg.pose.position.y + 0.5
    offset_pose.pose.position.z = msg.pose.position.z
    custom_pub.publish(offset_pose)

# -- MAIN -- #
if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)
    server = MyActionServer()

    custom_pub = rospy.Publisher('/rviz_drone_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)

    offset_pose = PoseStamped()
    rospy.spin()
