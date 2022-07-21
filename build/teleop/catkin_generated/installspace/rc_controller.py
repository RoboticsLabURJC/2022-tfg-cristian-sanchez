#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
 
def linear_x_change(val):
    vel.linear.x = val/100
    pub.publish(vel)

if __name__ == '__main__':
    try:
        vel = Twist()

        # vel.linear.x = 0.0
        # vel.linear.y = 0.0
        # vel.linear.z = 0.0

        # vel.angular.x = 0.0
        # vel.angular.x = 0.0
        # vel.angular.x = 0.0

        pub = rospy.Publisher('rc_controller_vel', Twist, queue_size=10)
        rospy.init_node('rc_controller', anonymous=True)
        pub.publish(vel)

        blank_image = np.zeros(shape=[512, 512, 3], dtype=np.uint8) 
        windowName = 'RC Controller'

        cv2.imshow(windowName, blank_image)
        cv2.createTrackbar('slider', windowName, 0, 100, linear_x_change)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass