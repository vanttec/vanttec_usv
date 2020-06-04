#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: obstacle_simulator.py
    @date: Wed Jun 3, 2020
	@author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
	@brief: Dock and acoustic signal simulator.
	@version: 1.0
    Open source
----------------------------------------------------------
'''

import math
import time
import os

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, PoseArray, Pose
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleSimulator:
    def __init__(self):

        self.active = True

        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0

        self.obstacle_list = []

        self.max_acoustic_radius = 20
        self.radius = 1 #Dock radius

        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)

        self.signal_pub = rospy.Publisher('/usv_perception/hydrophones/acoustic_signal', Float64, queue_size=10)
        self.detector_pub = rospy.Publisher('/usv_perception/lidar_detector/dock', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/usv_perception/dock_marker", MarkerArray, queue_size=10)

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
        pose_array = PoseArray()
        pose1 = Pose()
        pose2 = Pose()
        for i in range(len(self.obstacle_list)):
            x = self.obstacle_list[i]['X']
            y = self.obstacle_list[i]['Y']
            delta_x = x - self.ned_x
            delta_y = y - self.ned_y
            distance = math.pow(delta_x*delta_x + delta_y*delta_y, 0.5)
            x1 = x 
            y1 = y - 2*self.radius
            x2 = x 
            y2 = y + 2*self.radius
            x1b, y1b = self.ned_to_body(x1, y1)
            x2b, y2b = self.ned_to_body(x2, y2)
            pose1.position.x = x1b
            pose1.position.y = y1b
            pose_array.poses.append(pose1)
            pose2.position.x = x2b
            pose2.position.y = y2b
            pose_array.poses.append(pose2)
            self.detector_pub.publish(pose_array)
            if (distance < self.max_acoustic_radius):
                xb, yb = self.ned_to_body(x, y)
                signal = np.math.atan2(yb,xb)
                self.signal_pub.publish(signal)

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
            x = self.obstacle_list[i]['X']
            y = -self.obstacle_list[i]['Y']
            self.radius = 1
            marker = Marker()
            marker.header.frame_id = "/world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = self.radius
            marker.scale.y = self.radius*2
            marker.scale.z = self.radius
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x + (self.radius/2.)
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.id = i
            marker_array.markers.append(marker)
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

def main():
    rospy.init_node('dock_simulator', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    obstacleSimulator = ObstacleSimulator()
    obstacleSimulator.obstacle_list.append({'X' : 8,
                                'Y' : -5})
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
