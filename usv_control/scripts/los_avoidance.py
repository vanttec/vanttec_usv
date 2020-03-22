#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: los_avoidance.py
    @date: Fri Dec 20, 2019
    @modified: Sun Mar 22, 2020
    @author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @brief: Implementation of line-of-sight (LOS) algorithm with obstacle
            avoidance approach published in COMRob 2019
    @version: 1.1
    Open source
---------------------------------------------------------
'''

import os
import time
import math

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

class LOSAvoidance:
    def __init__(self):        
        self.active = True

        self.desired_speed = 0
        self.desired_heading = 0
        self.distance = 0
        self.bearing = 0

        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0

        self.reference_latitude = 0
        self.reference_longitude = 0

        self.waypoint_array = []
        self.last_waypoint_array = []

        self.delta_max = 10
        self.delta_min = 2
        self.gamma = 0.003

        self.k = 1

        self.waypoint_path = Pose2D()
        self.los_path = Pose2D()

        self.obstacle_view = "000"

        self.waypoint_mode = 0 # 0 for NED, 1 for GPS, 2 for body

        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)
        rospy.Subscriber("/vectornav/ins_2d/ins_ref", Vector3, self.gpsref_callback)
        rospy.Subscriber("/mission/waypoints", Float32MultiArray, self.waypoints_callback)
        rospy.Subscriber("/usv_perception/lidar_detector/obstacles",  String, self.obstacles_callback)

        # ROS Publishers
        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/usv_control/los/target", Pose2D, queue_size=10)
        self.LOS_pub = rospy.Publisher("/usv_control/los/los", Pose2D, queue_size=10)

    def ned_callback(self, ned):
        self.ned_x = ned.x
        self.ned_y = ned.y
        self.yaw = ned.theta

    def gpsref_callback(self, gps):
        self.reference_latitude = gps.x
        self.reference_longitude = gps.y

    def waypoints_callback(self, msg):
        waypoints = []
        leng = (msg.layout.data_offset)

        for i in range(int(leng)-1):
            waypoints.append(msg.data[i])
        self.waypoint_mode = msg.data[-1] # 0 for NED, 1 for GPS, 2 for body
        self.waypoint_array = waypoints

    def obstacles_callback(self, data):
        self.obstacle_view = data.data

    def los_manager(self, listvar):
        '''
        @name: los_manager
        @brief: Waypoint manager to execute the LOS algorithm.
        @param: listvar: list of variable waypoints
        @return: --
        '''
        if self.k < len(listvar)/2:
            x1 = listvar[2*self.k - 2]
            y1 = listvar[2*self.k - 1]
            x2 = listvar[2*self.k]
            y2 = listvar[2*self.k + 1]
            self.waypoint_path.x = x2
            self.waypoint_path.y = y2
            self.target_pub.publish(self.waypoint_path)
            x_squared = math.pow(x2 - self.ned_x, 2)
            y_squared = math.pow(y2 - self.ned_y, 2)
            self.distance = math.pow(x_squared + y_squared, 0.5)

            if self.distance > 1:
                self.los(x1, y1, x2, y2)
            else:
                self.k += 1
        else:
            self.desired(0, self.yaw)

    def los(self, x1, y1, x2, y2):
        '''
        @name: los
        @brief: Implementation of the LOS algorithm.
        @param: x1: x coordinate of the path starting-waypoint
                y1: y coordinate of the path starting-waypoint
                x2: x coordinate of the path ending-waypoint
                y2: y coordinate of the path ending-waypoint
        @return: --
        '''
        ak = math.atan2(y2 - y1, x2 - x1)
        ye = -(self.ned_x - x1)*math.sin(ak) + (self.ned_y - y1)*math.cos(ak)
        xe = (self.ned_x - x1)*math.cos(ak) + (self.ned_y - y1)*math.sin(ak)
        delta = (self.delta_max - self.delta_min)*math.exp(-(1/self.gamma)*abs(ye)) + self.delta_min
        psi_r = math.atan(-ye/delta)
        self.bearing = ak + psi_r

        if (abs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing) - 2*math.pi)

        x_los = x1 + (delta+xe)*math.cos(ak)
        y_los = y1 + (delta+xe)*math.sin(ak)
        self.los_path.x = x_los
        self.los_path.y = y_los
        self.LOS_pub.publish(self.los_path)
        self.vel = 1
	
        if self.distance < 6:
            self.vel = 0.4

        self.avoid(self.obstacle_view)

    def avoid(self, segment):
        '''
        @name: avoid
        @brief: Implementation of the collision avoidance algorithm.
        @param: segment: string  of the three divisions
        @return: --
        '''
        left_segment = segment[0]
        left_segment = int(left_segment)
        if left_segment > 0:
            left_segment = -5
        left = left_segment

        center_segment = segment[1]
        center_segment = int(center_segment)
        if center_segment > 0:
            center_segment = 1
        center = center_segment

        right_segment = segment[2]
        right_segment = int(right_segment)
        if right_segment > 0:
            right_segment = 3
        right = right_segment

        addition = left + center + right 

        if center == 0:
            self.vel = self.vel

        elif addition == -1 or addition == 1:
                self.vel = -0.4

        elif addition != 0:
            self.bearing = self.yaw + addition * 0.17
            if (abs(self.bearing) > (math.pi)):
                self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing) - 2*math.pi)

        self.desired(self.vel, self.bearing)

    def gps_to_ned(self, latitude_2, longitude_2):
        '''
        @name: gps_to_ned
        @brief: Coordinate transformation between geodetic and NED reference frames.
        @param: latitude_2: target x coordinate in geodetic reference frame
                longitude_2: target y coordinate in geodetic reference frame
        @return: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        '''
        latitude_1 = self.reference_latitude
        longitude_1 = self.reference_longitude

        longitud_distance = (longitude_1 - longitude_2)
        y_distance = math.sin(longitud_distance)*math.cos(latitude_2)
        x_distance = (math.cos(latitude_1)*math.sin(latitude_2) 
                     - math.sin(latitude_1)*math.cos(latitude_2)*math.cos(longitud_distance))
        bearing = math.atan2(-y_distance, x_distance)
        phi_1 = math.radians(latitude_1)
        phi_2 = math.radians(latitude_2)
        delta_phi = math.radians(latitude_2 - latitude_1)
        delta_longitude = math.radians(longitude_2 - longitude_1)
        a = (math.sin(delta_phi/2)*math.sin(delta_phi/2) 
             + math.cos(phi_1)*math.cos(phi_2)*math.sin(delta_longitude/2)*math.sin(delta_longitude/2))
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6378137*c

        ned_x2 = distance*math.cos(bearing)
        ned_y2 = distance*math.sin(bearing)

        return (ned_x2,ned_y2)

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
        J = np.array([[math.cos(self.yaw),
                      -1*math.sin(self.yaw)],
                      [math.sin(self.yaw),
                       math.cos(self.yaw)]])
        n = J.dot(p)
        ned_x2 = n[0] + self.ned_x
        ned_y2 = n[1] + self.ned_y
        return (ned_x2, ned_y2)

    def desired(self, _speed, _heading):
        self.desired_heading = _heading
        self.desired_speed = _speed
        self.d_heading_pub.publish(self.desired_heading)
        self.d_speed_pub.publish(self.desired_speed)

def main():
    rospy.init_node('los_avoidance', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    losAvoidance = LOSAvoidance()
    losAvoidance.last_waypoint_array = []
    aux_waypoint_array = []

    while not rospy.is_shutdown() and losAvoidance.active:
        if losAvoidance.last_waypoint_array != losAvoidance.waypoint_array:
            losAvoidance.k = 1
            losAvoidance.last_waypoint_array = losAvoidance.waypoint_array
            aux_waypoint_array = losAvoidance.last_waypoint_array
            x_0 = losAvoidance.ned_x
            y_0 = losAvoidance.ned_y

            if losAvoidance.waypoint_mode == 0:
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
            elif losAvoidance.waypoint_mode == 1:
                for i in range(0,len(aux_waypoint_array),2):
                    aux_waypoint_array[i], aux_waypoint_array[i+1] = losAvoidance.gps_to_ned(aux_waypoint_array[i],aux_waypoint_array[i+1])
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
            elif t.waypoint_mode == 2:
                for i in range(0,len(aux_waypoint_array),2):
                    aux_waypoint_array[i], aux_waypoint_array[i+1] = losAvoidance.body_to_ned(aux_waypoint_array[i],aux_waypoint_array[i+1])
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
        if len(aux_waypoint_array) > 1:
            losAvoidance.los_manager(losAvoidance.last_waypoint_array)
        rate.sleep()
    losAvoidance.desired(0,losAvoidance.yaw)
    rospy.logwarn('Finished')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
