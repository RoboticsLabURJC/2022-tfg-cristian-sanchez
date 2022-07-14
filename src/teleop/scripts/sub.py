#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('sub_node', anonymous=True) 
    rospy.Subscriber("topic_of_pub_node", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()