#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: obstacle_simulator.py
    @date: Sun Mar 22, 2020
    @modified: Sat May 15, 2021
	@author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
	@brief: Obstacle simulation for mission testing.
	@version: 1.2
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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ObstacleSimulator:
    def __init__(self):

        self.active = True

        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0

        self.challenge = 3 #0 for AutonomousNavigation, 1 for SpeedChallenge, 2 for ObstacleChannel, 3 for ObstacleField
        self.obstacle_list = []

        self.max_visible_radius = 10
        self.sensor_to_usv_offset = 0.55

        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)

        #self.detector_pub = rospy.Publisher('/usv_perception/yolo_zed/objects_detected', obj_detected_list, queue_size=10)
        self.detector_pub = rospy.Publisher('/usv_perception/lidar/objects_detected', obj_detected_list, queue_size=10)
        self.marker_pub = rospy.Publisher("/usv_perception/lidar_detector/markers", MarkerArray, queue_size=10)

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
                    obstacle.X = x - self.sensor_to_usv_offset
                    obstacle.Y = -y
                    obstacle.R = self.obstacle_list[i]['R']
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

    def rviz_markers(self):
        '''
        @name: rviz_markers
        @brief: Publishes obstacles as rviz markers.
        @param: --
        @return: --
        '''
        marker_array = MarkerArray()
        for i in range(len(self.obstacle_list)):
            if self.obstacle_list[i]['class'] == 'buoy':
                x = self.obstacle_list[i]['X']
                y = -self.obstacle_list[i]['Y']
                if self.challenge == 0:
                    diameter = 0.254
                if self.challenge == 1:
                    diameter = 0.39
                if self.challenge == 2:
                    diameter = 0.21
                if self.challenge == 3:
                    diameter = 0.21
                marker = Marker()
                marker.header.frame_id = "/world"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                if self.obstacle_list[i]['color'] == 'yellow':
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    if self.challenge == 2:
                        diameter = 0.39
                elif self.obstacle_list[i]['color'] == 'red':
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif self.obstacle_list[i]['color'] == 'green':
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif self.obstacle_list[i]['color'] == 'blue':
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = diameter
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0
                if self.challenge == 0:
                    marker.type = marker.CYLINDER
                    marker.scale.z = 1
                    marker.pose.position.z = 0.5
                marker.id = i
                marker_array.markers.append(marker)
            elif self.obstacle_list[i]['class'] == 'marker':
                x = self.obstacle_list[i]['X']
                y = -self.obstacle_list[i]['Y']
                diameter = 0.2286
                marker = Marker()
                marker.header.frame_id = "/world"
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = 1.54
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.77
                marker.id = i
                marker_array.markers.append(marker)
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

def main():
    rospy.init_node('obstacle_simulator', anonymous=False)
    rate = rospy.Rate(20) # 100hz
    obstacleSimulator = ObstacleSimulator()
    if obstacleSimulator.challenge == 0:
        obstacleSimulator.obstacle_list.append({'X' : 6.5,
                                    'Y' : -0.5,
                                    'R' : 0.127,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 3.5,
                                    'Y' : 2.5,
                                    'R' : 0.127,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 22.5,
                                    'Y' : 15.5,
                                    'R' : 0.127,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 19.5,
                                    'Y' : 18.5,
                                    'R' : 0.127,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
    elif obstacleSimulator.challenge == 1:
        obstacleSimulator.obstacle_list.append({'X' : 6.5,
                                    'Y' : -0.5,
                                    'R' : 0.195,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 3.5,
                                    'Y' : 2.5,
                                    'R' : 0.195,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 21,
                                    'Y' : 17,
                                    'R' : 0.195,
                                    'color' : 'blue', 
                                    'class' : 'buoy'})
    elif obstacleSimulator.challenge == 2:
        obstacleSimulator.max_visible_radius = 50
        obstacleSimulator.obstacle_list.append({'X' : 7,
                                    'Y' : 2,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 7,
                                    'Y' : 5,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' :14,
                                    'Y' : 4.7,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 14,
                                    'Y' : 7.7,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 16,
                                    'Y' : 5.5,
                                    'R' : 0.195,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 21,
                                    'Y' : 5.2,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' :21,
                                    'Y' : 8.2,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 28,
                                    'Y' : 4.3,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 28,
                                    'Y' : 7.3,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 35,
                                    'Y' : 1,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' :35,
                                    'Y' : 4,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 40,
                                    'Y' : 3,
                                    'R' : 0.195,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 42,
                                    'Y' : 0.8,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 42,
                                    'Y' : 3.8,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' :49,
                                    'Y' : 1.5,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 49,
                                    'Y' : 4.5,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 56,
                                    'Y' : 3,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 56,
                                    'Y' : 6,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' :63,
                                    'Y' : 5,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 63,
                                    'Y' : 8,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
    elif obstacleSimulator.challenge == 3:
        obstacleSimulator.max_visible_radius = 50
        obstacleSimulator.obstacle_list.append({'X' : 3.1,
                                    'Y' : 5.1, 
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 3.3,
                                    'Y' : 7.2,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 3.2,
                                    'Y' : 1.3,
                                    'R' : 0.105,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 6.2,
                                    'Y' : 5.2,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 4.1,
                                    'Y' : 0.2,
                                    'R' : 0.105,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 4.4,
                                    'Y' : 3.5,
                                    'R' : 0.105,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 5.8,
                                    'Y' : 1.4,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 6.4,
                                    'Y' : 6.3,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 9.1,
                                    'Y' : 0.4,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 9.6,
                                    'Y' : 1.6,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 12.6,
                                    'Y' : 7.4,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 10.7,
                                    'Y' : 0.6,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 10.3,
                                    'Y' : 10.1,
                                    'R' : 0.105,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 9.3,
                                    'Y' : 8.1,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 12.4,
                                    'Y' : 1.3,
                                    'R' : 0.105,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 13.6,
                                    'Y' : 3.6,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 13.4,
                                    'Y' : 6.4,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 14.2,
                                    'Y' : 4.2,
                                    'R' : 0.105,
                                    'color' : 'green', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 5.1,
                                    'Y' : 8.2,
                                    'R' : 0.105,
                                    'color' : 'yellow', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 7.7,
                                    'Y' : 9.6,
                                    'R' : 0.105,
                                    'color' : 'red', 
                                    'class' : 'buoy'})
        obstacleSimulator.obstacle_list.append({'X' : 10.0,
                                    'Y' : 4.0,
                                    'R' : 0.105,
                                    'color' : 'blue', 
                                    'class' : 'marker'})
                                
    while not rospy.is_shutdown() and obstacleSimulator.active:
        obstacleSimulator.simulate()
        obstacleSimulator.rviz_markers()
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
