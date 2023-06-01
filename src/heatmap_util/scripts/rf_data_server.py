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
        '''
        Constructor, defines all variables needed to act like:

            - Power action server   --> returns power value for given coords (index).
            - Rviz action server    --> returns all power values to display in rviz.
            - Offset publisher      --> Publish drone pose with the offset added.
        '''
        self._rviz_server = actionlib.SimpleActionServer('rviz_friss_action', RvizFrissAction, self.__response_rviz, False)
        self._power_server = actionlib.SimpleActionServer('drone_friss_action', GetPowerFrissAction, self.__response_drone, False)
        self._offset_pub = rospy.Publisher('/rviz_drone_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        self._size = rospy.get_param('map_size')
        self._origin = rospy.get_param('radio_origin')
        self._res = rospy.get_param('resolution')

        self._rviz_server.start()
        self._power_server.start()

        self._rviz_result = RvizFrissResult()
        self._power_result = GetPowerFrissResult()
        self._offset_pose = PoseStamped()
        self._model = fr.Friss(world_sz=self._size, resolution=self._res)
        self._data = self._model.model_power_signal(self._origin)


    def __response_rviz(self, goal):
        '''
        Map request for rviz representation
        '''    
        if goal.get_data:
            flip_data = np.rot90(self._data, k=-1)
            flip_data = np.flip(flip_data, axis=1)
            self._rviz_result.data = list(flip_data.flatten())
            self._rviz_result.size = self._rviz_size[0]
            self._rviz_server.set_succeeded(self._rviz_result)
        else:
            rospy.logwarn("get_data set to false, aborting...")
            self._rviz_server.set_aborted()


    def __response_drone(self, goal): 
        '''
        Data request for a certain index inside friss model.
        '''       
        if len(goal.index) == 2:
            x, y = goal.index
            out_of_index = x < 0 or y < 0 or x > (self._size[0] - 1) or y > (self._size[1] - 1)

            if not out_of_index:
                self._power_result.data = self._data[x, y]
            else:
                self._power_result.data = 1

            self._power_result.size = self._size[0]
            self._power_result.source_coords = self._origin
            self._power_server.set_succeeded(self._power_result)
        else:    
            rospy.logwarn("wrong format for index, please introduce [x, y]")
            self._power_server.set_aborted()


    def pose_callback(self, msg):
        '''
        Publish drone pose centered in the cell
        '''
        self._offset_pose.header = msg.header
        self._offset_pose.pose.position.x = msg.pose.position.x + (self._res / 2)
        self._offset_pose.pose.position.y = msg.pose.position.y + (self._res / 2)
        self._offset_pose.pose.position.z = msg.pose.position.z

        self._offset_pub.publish(self._offset_pose)

# -- MAIN -- #
if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)
    server = MyActionServer()
    rospy.spin()
