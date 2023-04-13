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
from heatmap_util.msg import RvizFrissAction, RvizFrissResult, DroneFrissAction, DroneFrissResult

# -- CTE -- #
NODENAME = 'heatmap_server_node'
DEFAULT_ORIGIN = (0, 0)         # Origin of the signal
WORLD_SIZE = (10, 10)
WORLD_SIZE_RVIZ = (WORLD_SIZE[0] + 1, WORLD_SIZE[1] + 1)

class MyActionServer:
    def __init__(self):
        self._rviz_server = actionlib.SimpleActionServer('rviz_friss_action', RvizFrissAction, self.__response_rviz, False)
        self._drone_server = actionlib.SimpleActionServer('drone_friss_action', DroneFrissAction, self.__response_drone, False)

        self._rviz_server.start()
        self._drone_server.start()

        self._rviz_result = RvizFrissResult()
        self._drone_result = DroneFrissResult()

    def __response_rviz(self, goal):
        rospy.loginfo("rviz data requested!")
        if goal.get_data:
            self._rviz_result.data = self.__get_rviz_data()
            self._rviz_server.set_succeeded(self._rviz_result)
            rospy.loginfo("Response sent to rviz!")
        else:
            rospy.logwarn("get_data set to false, aborting...")
            self._rviz_server.set_aborted()

    def __response_drone(self, goal):
        rospy.loginfo("drone data requested!")

        # Checks if index is not valid ([x,y] positive integers or 0s)
        if len(goal.index) != 2 and not all(isinstance(x, int) and x>=0 for x in goal):
            rospy.logwarn("wrong format for index, please introduce [x, y]")
            self._drone_server.set_aborted()
        else:
            self._drone_result.data = self.__get_drone_data(goal.index)
            self._drone_server.set_succeeded(self._drone_result)
            rospy.loginfo("Response sent to drone!")

    def __get_rviz_data(self):
        model = fr.Friss(world_sz=WORLD_SIZE_RVIZ)
        data = model.model_power_signal(DEFAULT_ORIGIN)
        return list(data.flatten())
    
    def __get_drone_data(self, pose):
        x, y = pose
        model = fr.Friss(world_sz=WORLD_SIZE)
        data = model.model_power_signal(DEFAULT_ORIGIN)
        return data[x, y]

# -- MAIN -- #
if __name__ == '__main__':
    rospy.init_node(NODENAME, anonymous=True)
    server = MyActionServer()
    rospy.spin()
