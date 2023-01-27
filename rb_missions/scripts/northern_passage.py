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
    init_x = autoNav.ned_x
    init_y = autoNav.ned_y
    last_detection = []
    time.sleep(10)
    while not rospy.is_shutdown() and autoNav.activated:
        if autoNav.objects_list != last_detection or autoNav < 1:
            # State -1: Runs 'one' time. Changes when the boat detects at least two buoys ahead (the gate)
            if autoNav.state == -1:
                while (not rospy.is_shutdown()) and (len(autoNav.objects_list) < 2):
                    autoNav.test.publish(autoNav.state)
                    rate.sleep()
                autoNav.state = 0
                last_detection = autoNav.objects_list
        # State 0: The boat has detected the gate, and is moving towards 5 meters ahead of it
        if autoNav.state == 0:
            autoNav.test.publish(autoNav.state)
            if len(autoNav.objects_list) >= 2:
                autoNav.calculate_distance_to_boat()
            if len(autoNav.objects_list) >= 2 and autoNav.distance >= 2:
                autoNav.center_point()
            else:
                x_dif = autoNav.target_x - autoNav.ned_x
                y_dif = autoNav.target_y - autoNav.ned_y
                dist = math.pow(x_dif**2 + y_dif**2, 0.5) 
                if dist < 2:
                    autoNav.state = 1

            last_detection = autoNav.objects_list
        # State 1: The boat has reached the 5 m mark. It now starts moving forward until it detects the blue buoy
        if autoNav.state == 1:
            autoNav.test.publish(autoNav.state)
            x_list = []
            y_list = []
            class_list = []
            distance_list = []
            x_list, y_list, class_list, distance_list = challenge_functions.init_lists(autoNav)
            if len(distance_list) > 0:
                ind_0 = np.argsort(distance_list)[0]
            #if (len(autoNav.objects_list) >= 1 and
            #(str(autoNav.objects_list[ind_0]['color']) == 'blue')):
            if (len(autoNav.objects_list) >= 1):
                autoNav.state = 2
            else:
                initTime = rospy.Time.now().secs
                while (not rospy.is_shutdown() and (len(autoNav.objects_list)) < 1):
                    if rospy.Time.now().secs - initTime > 1:
                        autoNav.farther(3)
                        rate.sleep()
                        break
            last_detection = autoNav.objects_list
        if autoNav.objects_list != last_detection or autoNav.state < 4:
            # State 2: Keeps moving forward until the buoy is 5 meters ahead of the boat
            if autoNav.state == 2:
                autoNav.test.publish(autoNav.state)
                x_list = []
                y_list = []
                class_list = []
                distance_list = []
                x_list, y_list, class_list, distance_list = challenge_functions.init_lists(autoNav)
                if len(distance_list) > 0:
                    ind_0 = np.argsort(distance_list)[0]
                
                if ((len(autoNav.objects_list) >= 1) and
                    (autoNav.objects_list[ind_0]['X'] < 7)):
                    buoy_x = autoNav.objects_list[ind_0]['X']
                    buoy_y = autoNav.objects_list[ind_0]['Y']
                    autoNav.state = 3
                else:
                    initTime = rospy.Time.now().secs
                    while not rospy.is_shutdown() and (len(autoNav.objects_list) < 1 or autoNav.objects_list[ind_0]['X'] >= 7):
                        if rospy.Time.now().secs - initTime > 1:
                            autoNav.farther(4)
                            rate.sleep()
                            break
                last_detection = autoNav.objects_list
            # State 3: Calculation of the needed waypoints to go around the blue buoy and go back through the gate
            if autoNav.state == 3:
                autoNav.test.publish(autoNav.state)
                x_final, y_final = autoNav.buoy_waypoints(init_x, init_y, buoy_x,buoy_y)
                autoNav.state = 4
                last_detection = autoNav.objects_list
        # State 4: Publishes challenge status completion when the task starting coordinates point is at most 1 meter ahead
        if autoNav.state == 4:
            autoNav.test.publish(autoNav.state)
            x_squared = math.pow(x_final - autoNav.ned_x, 2)
            y_squared = math.pow(y_final - autoNav.ned_y, 2)
            distance_final = math.pow(x_squared + y_squared, 0.5)
            if distance_final <= 1:
                autoNav.status_pub.publish(2)
            last_detection = autoNav.objects_list
        rate.sleep()    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass