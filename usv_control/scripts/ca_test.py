#!/usr/bin/env python

import os
import time
import rospy
from std_msgs.msg import Float32MultiArray
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
import math

#Constant Speed and Constant angular
#0.7 m/s and pi/8 rad/s
class Test:
    def __init__(self):
        self.testing = True

        self.waypoints = Float32MultiArray()
        self.waypoints.layout.data_offset = 3
        self.waypoints.data = [5, 1, 0]

        self.obj_list = obj_detected_list()
        self.len_list = 1
        self.obj = obj_detected()
        #print(p1,p2)
        self.obj.X = 
        self.obj.Y = 1
        self.obj.color = "green"
        self.obj.clase = "bouy"
        self.obj_list.objects.append(self.obj)

        self.obstacles_pub = rospy.Publisher("/usv_perception/lidar_detector/obstacles", obj_detected_list, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)

def main():
    rospy.init_node('ca_test', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    while not rospy.is_shutdown() and t.testing:
        t.obstacles_pub.publish(t.obj_list)
        t.waypoints_pub.publish(t.waypoints)
        #t.testing = False
        #rospy.logwarn("Finished")
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass