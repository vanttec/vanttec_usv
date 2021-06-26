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
    rospy.init_node('straight_path_waypoints', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    time.sleep(10)
    rospy.logwarn("Start")
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 5
    path_array.data = [3.0,0,3.0,-30.0,0] # Last should be waypoint mode: 0 for NED, 1 for GPS, 2 for body
    while not rospy.is_shutdown():
        t.desired(path_array)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
