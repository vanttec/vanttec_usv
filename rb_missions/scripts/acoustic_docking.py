#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: acoustic_docking.py
    @date: Wed Jun 3, 2020
    @author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @brief: Motion planning. ROS node to follow an acoustic
        signal for autonomous docking.
    @version: 1.0
    Open source
---------------------------------------------------------
'''

import math
import time
import os

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from std_msgs.msg import Int32, Float32MultiArray, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

class AcousticDocking:
    def __init__(self):
        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0

        self.activated = True

        self.distance = 0
        self.signal_angle = 0
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.x_body_origin = 0
        self.y_body_origin = 0

        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber("/usv_perception/hydrophones/acoustic_signal", Float64, self.signal_callback)
        rospy.Subscriber("/usv_perception/lidar_detector/dock", PoseArray, self.dock_callback)

        # ROS Publishers
        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

    def ins_pose_callback(self,pose):
        self.ned_x = pose.x
        self.ned_y = pose.y
        self.yaw = pose.theta

    def signal_callback(self, signal):
        self.signal_angle = signal.data

    def dock_callback(self, dock):
        self.x1 = dock.poses[0].position.x
        self.y1 = dock.poses[0].position.y
        self.x2 = dock.poses[1].position.x
        self.y2 = dock.poses[1].position.y

    def calculate_distance_to_boat(self):
        '''
        @name: calculate_distance_to_boat
        @brief: Calculates the distance between the USV and the dock. 
        @param: --
        @return: --
        '''
        xc = min([self.x1,self.x2]) + abs(self.x1 - self.x2)/2
        yc = min([self.y1,self.y2]) + abs(self.y1 - self.y2)/2

        self.distance = math.pow(xc*xc + yc*yc, 0.5)

    def center_point(self):
        '''
        @name: center_point
        @brief: Calculates the intersection point between the USV and the pinger
          location at the dock. Returns two waypoints as desired positions. The first
          waypoint is perpendicularly in front of the pinger to straighten the path.
          the second waypoint is the location of the pinger in the dock, for docking. 
        @param: --
        @return: --
        '''

        if self.y1 < self.y2:
            yl = self.y1
            xl = self.x1
            yr = self.y2
            xr = self.x2
        else:
            yl = self.y2
            xl = self.x2
            yr = self.y1
            xr = self.x1

        yd = yl - yr
        xd = xl - xr

        alpha = math.atan2(yd,xd) + math.pi/2
        if (abs(alpha) > (math.pi)):
            alpha = (alpha/abs(alpha))*(abs(alpha) - 2*math.pi)

        x_beta, y_beta = self.aux_to_body(1,0,self.signal_angle,self.x_body_origin,self.y_body_origin)

        common_denominator = (xl - xr)*(self.y_body_origin - y_beta) - (yl - yr)*(self.x_body_origin - x_beta)
        x_pinger = ((xl*yr-yl*xr)*(self.x_body_origin-x_beta)-(xl-xr)*(self.x_body_origin*y_beta-self.y_body_origin*x_beta)) / common_denominator
        y_pinger = ((xl*yr-yl*xr)*(self.y_body_origin-y_beta)-(yl-yr)*(self.x_body_origin*y_beta-self.y_body_origin*x_beta)) / common_denominator

        x_aux, y_aux = self.aux_to_body(-2,0,alpha,x_pinger,y_pinger)

        path_array = Float32MultiArray()
        path_array.layout.data_offset = 5
        path_array.data = [x_aux, y_aux, x_pinger, y_pinger, 2]

        self.desired(path_array)

    def aux_to_body(self, aux_x2, aux_y2, alpha, body_x1, body_y1):
        '''
        @name: aux_to_body
        @brief: Coordinate transformation between auxiliary and body reference frames.
        @param: aux_x2: target x coordinate in aux reference frame
                aux_y2: target y coordinate in aux reference frame
                alpha: angle between aux and body reference frames
                body_x1: aux x coordinate in body reference frame
                body_y1: aux y coordinate in body reference frame
        @return: body_x2: target x coordinate in body reference frame
                 body_y2: target y coordinate in body reference frame
        '''
        p = np.array([[aux_x2],[aux_y2]])
        J = self.rotation_matrix(alpha)
        n = J.dot(p)
        body_x2 = n[0] + body_x1
        body_y2 = n[1] + body_y1
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

    def desired(self, path):
    	self.path_pub.publish(path)

def main():
    rospy.init_node("acoustic_docking", anonymous=False)
    rate = rospy.Rate(20)
    acousticDocking = AcousticDocking()
    last_detection = []
    while not rospy.is_shutdown() and acousticDocking.activated:
        acousticDocking.calculate_distance_to_boat()
        if (acousticDocking.distance >= 5):
            acousticDocking.center_point()
        else:
            acousticDocking.status_pub.publish(1)

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

