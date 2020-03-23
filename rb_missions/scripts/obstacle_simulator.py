#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: obstacle_simulator.py
    @date: Sun Mar 22, 2020
	@author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
	@brief: Obstacle simulation for mission testing.
	@version: 1.0
    Open source
----------------------------------------------------------
'''

import math
import time
import os

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list


class ObstacleSimulator:
    def __init__(self):

        self.active = True

        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0

        self.challenge = 1 #0 for AutonomousNavigation, 1 for SpeedChallenge
        self.obstacle_list = []

        self.max_visible_radius = 10

        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)

        self.detector_pub = rospy.Publisher('/usv_perception/yolo_zed/objects_detected', obj_detected_list, queue_size=10)

    def ins_pose_callback(self,pose):
        self.ned_x = pose.x
        self.ned_y = pose.y
        self.yaw = pose.theta

    def simulate(self):
        '''
        @name: simulate
        @brief: Simulates the obstacles the  USV should be looking at any moment in time 
        @param: --
        @return: --
        '''
        object_detected_list = obj_detected_list()
        list_length = 0
        for i in range(len(self.obstacle_list)):
            x = self.obstacle_list[i]['X']
            y = self.obstacle_list[i]['Y']
            delta_x = x - self.ned_x
            delta_y = y - self.ned_y
            distance = math.pow(delta_x*delta_x + delta_y*delta_y, 0.5)
            if (distance < self.max_visible_radius):
                x, y = self.ned_to_body(x, y)
                if x > 1:
                    obstacle = obj_detected()
                    obstacle.X = x
                    obstacle.Y = y
                    obstacle.color = self.obstacle_list[i]['color']
                    obstacle.clase = self.obstacle_list[i]['class']
                    list_length += 1
                    object_detected_list.objects.append(obstacle)
        object_detected_list.len = list_length
        self.detector_pub.publish(object_detected_list)


    def body_to_ned(self, x2, y2):
        '''
        @name: body_to_ned
        @brief: Coordinate transformation between body and NED reference frames.
        @param: x2: target x coordinate in body reference frame
                y2: target y coordinate in body reference frame
        @return: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        '''
        p = np.array([x2, y2])
        J = self.rotation_matrix(self.yaw)
        n = J.dot(p)
        ned_x2 = n[0] + self.ned_x
        ned_y2 = n[1] + self.ned_y
        return (ned_x2, ned_y2)

    def ned_to_body(self, ned_x2, ned_y2):
        '''
        @name: ned_to_ned
        @brief: Coordinate transformation between NED and body reference frames.
        @param: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        @return: body_x2: target x coordinate in body reference frame
                body_y2: target y coordinate in body reference frame
        '''
        n = np.array([ned_x2 - self.ned_x, ned_y2 - self.ned_y])
        J = self.rotation_matrix(self.yaw)
        J = np.linalg.inv(J)
        b = J.dot(n)
        body_x2 = b[0]
        body_y2 = b[1]
        return (body_x2, body_y2)

    def rotation_matrix(self, angle):
        '''
        @name: rotation_matrix
        @brief: Transformation matrix template.
        @param: angle: angle of rotation
        @return: J: transformation matrix
        '''
        J = np.array([[math.cos(angle), -1*math.sin(angle)],
                      [math.sin(angle), math.cos(angle)]])
        return (J)

def main():
    rospy.init_node('obstacle_simulator', anonymous=False)
    rate = rospy.Rate(20) # 100hz
    obstacleSimulator = ObstacleSimulator()
    if obstacleSimulator.challenge == 0:
        obstacleSimulator.obstacle_list.append({'X' : 6,
                                    'Y' : 3,
                                    'color' : 'yellow', 
                                    'class' : 'bouy'})
        obstacleSimulator.obstacle_list.append({'X' : 6,
                                    'Y' : -3,
                                    'color' : 'yellow', 
                                    'class' : 'bouy'})
        obstacleSimulator.obstacle_list.append({'X' : 28,
                                    'Y' : 3,
                                    'color' : 'yellow', 
                                    'class' : 'bouy'})
        obstacleSimulator.obstacle_list.append({'X' : 28,
                                    'Y' : -3,
                                    'color' : 'yellow', 
                                    'class' : 'bouy'})
    elif obstacleSimulator.challenge == 1:
        obstacleSimulator.obstacle_list.append({'X' : 6,
                                    'Y' : 3,
                                    'color' : 'yellow', 
                                    'class' : 'bouy'})
        obstacleSimulator.obstacle_list.append({'X' : 6,
                                    'Y' : -3,
                                    'color' : 'yellow', 
                                    'class' : 'bouy'})
        obstacleSimulator.obstacle_list.append({'X' : 28,
                                    'Y' : 0,
                                    'color' : 'blue', 
                                    'class' : 'bouy'})
    while not rospy.is_shutdown() and obstacleSimulator.active:
        obstacleSimulator.simulate()
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass