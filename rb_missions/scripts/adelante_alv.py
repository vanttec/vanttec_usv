#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
                =====================================
                || NORTHERN PASSAGE - CHALLENGE #4 ||
                =====================================

    @file: panama_canal.py
    @date: Sun Jan 22, 2023
    @modified: Sat May 15, 2021

    @author: Max Pacheco Ramirez
    @e-mail: maxprpxam@icloud.com
    @co-author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @co-author: Daniel Noe Salinas ylc
    @e-mail: droniel@
    @co-author: Rodolfo Cuan Urquizo
    @e-mail: fitocuan@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com

	@brief: Motion planning. Script inspired in the outdated
    "speed_challenge.py", which functionality is to navigate a USV through a 
    first set of two buoys, find a third buoy, circle around it and return to 
    the first set of buoys, all by receiving obstacle positions and sending a 
    desired position for the USV.
	@version: 1.2
    Open source
----------------------------------------------------------
'''

import math
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Int32, String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import include.challenge_functions as challenge_functions

from usv_perception.msg import obj_detected, obj_detected_list

def main():
    rospy.init_node("speed_challenge", anonymous=False)
    rate = rospy.Rate(20)
    autoNav = challenge_functions.AutoNav()
    autoNav.distance = 4
    last_detection = []
    time.sleep(10)
    initTime = rospy.Time.now().secs
    while not rospy.is_shutdown() and autoNav.activated:
        if rospy.Time.now().secs - initTime > 1:
            autoNav.farther(3)
        rate.sleep()    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass