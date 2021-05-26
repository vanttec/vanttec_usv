#!/usr/bin/env python

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
    rospy.init_node('uturn_path_waypoints', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    time.sleep(2)
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 9
    path_array.data = [0,-1,0,-15,4,-15,4,-1,0] # Last should be waypoint mode: 0 for NED, 1 for GPS, 2 for body
    while not rospy.is_shutdown():
        t.desired(path_array)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass