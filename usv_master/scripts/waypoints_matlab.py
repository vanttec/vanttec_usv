#!/usr/bin/env python

import os
import time
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Float32MultiArray
import scipy.io as sio
from nav_msgs.msg import Path


class Test:
    def __init__(self):
        self.testing = True

        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)

    def desired(self, path): 
    	self.path_pub.publish(path)

def main():
    rospy.init_node('waypoints_matlab', anonymous=False)
    rate = rospy.Rate(20) # 100hz
    t = Test()
    dir_name = os.path.dirname(__file__)
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 33
    matlabposition = sio.loadmat(dir_name + '/mat/finalposition.mat')
    matlabposition = matlabposition['finalposition']
    positions = []
    for i in range(32):
        positions.extend((matlabposition[i,1],matlabposition[i,2]))
    positions.append(0)
                
    #path_array.data = [0, 4, 45, 4, 0]
    path_array.data = positions
    while not rospy.is_shutdown() and t.testing:
        time.sleep(1)
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 50 and not rospy.is_shutdown():
            t.desired(path_array)
            rate.sleep()
        path_array.layout.data_offset = 3
        path_array.data = [0,0,2]
        t.desired(path_array)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass