#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: panama_canal.py
    @date: Sun Jan 22, 2023
    @modified: Sat May 15, 2021

    @author: Max Pacheco Ramirez
    @e-mail: maxprpxam@icloud.com
    @co-author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @co-author: Rodolfo Cuan Urquizo
    @e-mail: fitocuan@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com

    @brief: Motion planning. Script inspired in the outdated
    "auto_nav_position.py", which functionality is to navigate
    a USV through two sets of buoys or markers, all by receiving 
    obstacle positions and sending a desired position for the USV.
    @version: 1.0
    Open source
---------------------------------------------------------
'''
import math
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose2D

from usv_perception.msg import obj_detected, obj_detected_list
import include.challenge_functions as challenge_functions


def main():
    rospy.init_node("auto_nav_position", anonymous=False)
    rate = rospy.Rate(10)
    autoNav = challenge_functions.AutoNav()
    autoNav.distance = 4
    last_detection = []
    time.sleep(10)
    while not rospy.is_shutdown() and autoNav.activated:
        # State -1: The boat hasnt detected the two buoys
        if autoNav.objects_list != last_detection or autoNav.state < 1:
            if autoNav.state == -1:
                while (not rospy.is_shutdown()) and (len(autoNav.objects_list) < 2):
                    autoNav.test.publish(autoNav.state)
                    rate.sleep()
                autoNav.state = 0
                last_detection = autoNav.objects_list

            # State 0: The boat is detecting the buoys and moves forward to the GPS target position
            # which takes it 3 meters ahead of the gate created by the first pair of buoys
            if autoNav.state == 0:
                autoNav.test.publish(autoNav.state)
                if len(autoNav.objects_list) >= 2:
                    autoNav.calculate_distance_to_boat()
                if (len(autoNav.objects_list) >= 2) and (autoNav.distance >= 3):
                    autoNav.center_point()
                else:
                    x_dif = autoNav.target_x - autoNav.ned_x
                    y_dif = autoNav.target_y - autoNav.ned_y
                    dist = math.pow(x_dif**2 + y_dif**2, 0.5) 
                    if dist < 3:
                        autoNav.state = 1
                        rate.sleep()
                last_detection = autoNav.objects_list

        # State 1: The boat has reached the 3m mark, and its moving forward until it detects another pair of buoys 0.2 to 5 meters ahead
        if autoNav.state == 1:
            autoNav.test.publish(autoNav.state)
            if len(autoNav.objects_list) >= 2:
                autoNav.state = 2
            else:
                initTime = rospy.Time.now().secs
                while ((not rospy.is_shutdown()) and 
                    (len(autoNav.objects_list) < 2)):
                    if rospy.Time.now().secs - initTime > 1:
                        autoNav.farther()
                        rate.sleep()
                        break
            last_detection = autoNav.objects_list

        if autoNav.objects_list != last_detection or autoNav.state < 3:
            # State 2: Same as state 0
            if autoNav.state == 2:
                autoNav.test.publish(autoNav.state)
                if len(autoNav.objects_list) >= 2:
                    autoNav.calculate_distance_to_boat()
                if len(autoNav.objects_list) >= 2 and autoNav.distance >= 3:
                    autoNav.center_point()
                else:
                    x_dif = autoNav.target_x - autoNav.ned_x
                    y_dif = autoNav.target_y - autoNav.ned_y
                    dist = math.pow(x_dif**2 + y_dif**2, 0.5) 
                    if dist < 3:
                        autoNav.state = 3
                        rate.sleep()
                last_detection = autoNav.objects_list

        # State 3: Reached the end of the task
        elif autoNav.state == 3:
            autoNav.test.publish(autoNav.state)
            time.sleep(1)
            autoNav.status_pub.publish(1)

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
