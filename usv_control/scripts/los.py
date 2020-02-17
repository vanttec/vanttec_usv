#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: LOS.py
    @date: Nov 2019
    @date_modif: Tue, Feb 4 2020
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


class Test:


    def __init__(self):


        self.testing = True

        self.ds = 0
        self.dh = 0
        self.distance = 0
        self.bearing = 0

        self.NEDx = 0
        self.NEDy = 0
        self.yaw = 0

        self.latref = 0
        self.lonref = 0

        self.wp_array = []
        self.wp_t = []

        self.dmax = 10
        self.dmin = 2
        self.gamma = 0.003

        self.k = 1

        self.Waypointpath = Pose2D()
        self.LOSpath = Pose2D()

        self.waypoint_mode = 0 # 0 for NED, 1 for GPS, 2 for body
         
        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.gps_callback)
        rospy.Subscriber("/vectornav/ins_2d/ins_ref", Pose2D, self.gpsref_callback)
        rospy.Subscriber("/mission/waypoints", Float32MultiArray, self.waypoints_callback)

        # ROS Publishers
        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/usv_control/LOS/target", Pose2D, queue_size=10)
        self.LOS_pub = rospy.Publisher("/usv_control/LOS/LOS", Pose2D, queue_size=10)

    def gps_callback(self, _gps):
        self.NEDx = _gps.x
        self.NEDy = _gps.y
        self.yaw = _gps.theta

    def gpsref_callback(self, _gps):
        self.latref = _gps.x
        self.lonref = _gps.y

    def waypoints_callback(self, _msg):
        wp = []
        leng = (_msg.layout.data_offset)

        for i in range(int(leng)-1):
            wp.append(_msg.data[i])
        self.waypoint_mode = _msg.data[-1] # 0 for NED, 1 for GPS, 2 for body
        self.wp_array = wp

    def LOS_loop(self, _listvar):

        if self.k < len(_listvar)/2:

            x1 = _listvar[2*self.k - 2]
            y1 = _listvar[2*self.k - 1]
            x2 = _listvar[2*self.k]
            y2 = _listvar[2*self.k + 1]
            self.Waypointpath.x = x2
            self.Waypointpath.y = y2
            self.target_pub.publish(self.Waypointpath)
            xpow = math.pow(x2 - self.NEDx, 2)
            ypow = math.pow(y2 - self.NEDy, 2)
            self.distance = math.pow(xpow + ypow, 0.5)

            if self.distance > 1:
                self.LOS(x1, y1, x2, y2)
            else:
                self.k += 1
        else:
            self.desired(0, self.yaw)

    def LOS(self, _x1, _y1, _x2, _y2):
        ak = math.atan2(_y2 - _y1, _x2 - _x1)
        ye = -(self.NEDx - _x1)*math.sin(ak) + (self.NEDy - _y1)*math.cos(ak)
        xe = (self.NEDx - _x1)*math.cos(ak) + (self.NEDy - _y1)*math.sin(ak)
        delta = (self.dmax - self.dmin)*math.exp(-(1/self.gamma)*abs(ye)) + self.dmin
        psi_r = math.atan(-ye/delta)
        self.bearing = ak + psi_r

        if (abs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing) - 2*math.pi)

        xLOS = _x1 + (delta+xe)*math.cos(ak)
        yLOS = _y1 + (delta+xe)*math.sin(ak)
        self.LOSpath.x = xLOS
        self.LOSpath.y = yLOS
        self.LOS_pub.publish(self.LOSpath)

        self.vel = 1
        if self.distance < 6:
            self.vel = 0.4

        self.desired(self.vel, self.bearing)

    def gps_to_ned(self, _lat2, _lon2):
        lat1 = self.latref
        lon1 = self.lonref

        longitud_distance = (lon1 - _lon2)
        y_distance = math.sin(longitud_distance)*math.cos(_lat2)
        x_distance = (math.cos(lat1)*math.sin(_lat2) 
                     - math.sin(lat1)*math.cos(_lat2)*math.cos(longitud_distance))
        bearing = math.atan2(-y_distance, x_distance)
        phi1 = math.radians(lat1)
        phi2 = math.radians(_lat2)
        dphi = math.radians(_lat2 - lat1)
        dlam = math.radians(_lon2 - lon1)
        a = (math.sin(dphi/2)*math.sin(dphi/2) 
             + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)*math.sin(dlam/2))
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6378137*c

        nedx = distance*math.cos(bearing)
        nedy = distance*math.sin(bearing)

        return (nedx,nedy)

    def body_to_ned(self, _x, _y):
        p = np.array([_x, _y])
        J = np.array([[math.cos(self.yaw),
                      -1*math.sin(self.yaw)],
                      [math.sin(self.yaw),
                       math.cos(self.yaw)]])
        n = J.dot(p)

        nedx = n[0] + self.NEDx
        nedy = n[1] + self.NEDy

        return (nedx, nedy)

    def desired(self, _speed, _heading):
        self.dh = _heading
        self.ds = _speed
        self.d_heading_pub.publish(self.dh)
        self.d_speed_pub.publish(self.ds)


def main():

    rospy.init_node('los', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    test = Test()
    test.wp_t = []
    wp_LOS = []

    while (not rospy.is_shutdown()) and test.testing:
        if test.wp_t != test.wp_array:
    
            test.k = 1
            test.wp_t = test.wp_array
            wp_LOS = test.wp_t
            x_0 = test.NEDx
            y_0 = test.NEDy
            
            if test.waypoint_mode == 0:
                wp_LOS.insert(0,x_0)
                wp_LOS.insert(1,y_0)
            elif test.waypoint_mode == 1:
                for i in range(0, len(wp_LOS), 2):
                    wp_LOS[i], wp_LOS[i+1] = test.gps_to_ned(wp_LOS[i],wp_LOS[i+1])
                wp_LOS.insert(0,x_0)
                wp_LOS.insert(1,y_0)
            elif test.waypoint_mode == 2:
                for i in range(0, len(wp_LOS), 2):
                    wp_LOS[i], wp_LOS[i+1] = test.body_to_ned(wp_LOS[i],wp_LOS[i+1])
                wp_LOS.insert(0,x_0)
                wp_LOS.insert(1,y_0)
        if len(wp_LOS) > 1:
            test.LOS_loop(test.wp_t)
        rate.sleep()
    test.desired(0, test.yaw)
    rospy.logwarn('Finished')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
