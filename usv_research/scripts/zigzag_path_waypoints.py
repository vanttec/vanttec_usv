#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Float32MultiArray

class Test:
    def __init__(self):
        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)

    def desired(self, path):
    	self.path_pub.publish(path)

def main():
    rospy.init_node('zigzag_path_waypoints', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    time.sleep(5)
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 11
    path_array.data = [-1,-1,3.5,-6,-1,-15,2,-23,-1.5,-30,0] # Last should be waypoint mode: 0 for NED, 1 for GPS, 2 for body
    while not rospy.is_shutdown():
        t.desired(path_array)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass