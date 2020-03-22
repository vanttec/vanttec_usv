#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: los.py
    @date: Nov 2019
    @date_modif: Sat Mar 21, 2020
    @author: Alejandro Gonzalez
    @e-mail: alexglzg97@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
    @brief: Implementation of line-of-sight (LOS) algorithm with inputs on
      NED, geodetic and body reference frames
    Open source
----------------------------------------------------------
'''

import math
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float32MultiArray, Float64

# Class definition
class LOS:
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

        self.waypoint_mode = 0 # 0 for NED, 1 for GPS, 2 for body
         
        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)
        rospy.Subscriber("/vectornav/ins_2d/ins_ref", Pose2D, self.gpsref_callback)
        rospy.Subscriber("/mission/waypoints", Float32MultiArray, self.waypoints_callback)

        # ROS Publishers
        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/usv_control/LOS/target", Pose2D, queue_size=10)
        self.LOS_pub = rospy.Publisher("/usv_control/LOS/LOS", Pose2D, queue_size=10)

    def ned_callback(self, gps):
        self.ned_x = gps.x
        self.ned_y = gps.y
        self.yaw = gps.theta

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
        
        if abs(self.bearing - self.yaw) > np.pi/4:
            self.vel = self.vel * 0.5

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

    rospy.init_node('los', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    los = LOS()
    los.last_waypoint_array = []
    aux_waypoint_array = []

    while (not rospy.is_shutdown()) and los.active:
        if los.last_waypoint_array != los.waypoint_array:
            los.k = 1
            los.last_waypoint_array = los.waypoint_array
            aux_waypoint_array = los.last_waypoint_array
            x_0 = los.ned_x
            y_0 = los.ned_y
            
            if los.waypoint_mode == 0:
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
            elif los.waypoint_mode == 1:
                for i in range(0, len(aux_waypoint_array), 2):
                    aux_waypoint_array[i], aux_waypoint_array[i+1] = los.gps_to_ned(aux_waypoint_array[i],aux_waypoint_array[i+1])
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
            elif los.waypoint_mode == 2:
                for i in range(0, len(aux_waypoint_array), 2):
                    aux_waypoint_array[i], aux_waypoint_array[i+1] = los.body_to_ned(aux_waypoint_array[i],aux_waypoint_array[i+1])
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
        if len(aux_waypoint_array) > 1:
            los.los_manager(los.last_waypoint_array)
        rate.sleep()
    los.desired(0, los.yaw)
    rospy.logwarn('Finished')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
