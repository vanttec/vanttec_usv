#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: auto_nav_guidance.py
    @date: Thu Dec 26, 2019
    @modified: Sat Mar 21, 2020
    @author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @co-author: Rodolfo Cuan Urquizo
    @e-mail: fitocuan@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
    @brief: Motion planning. Script to navigate a USV through two sets of buoys 
            or markers, all by receiving obstacle positions and sending a 
            desired position for the USV.
    @version: 1.2
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

# Class Definition
class AutoNav:
    def __init__(self):
        self.NEDx = 0
        self.NEDy = 0
        self.yaw = 0
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.distance = 0
        self.InitTime = rospy.Time.now().secs
        self.offset = .55 #camera to ins offset
        self.target_x = 0
        self.target_y = 0
        self.ned_alpha = 0

        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)

        # ROS Publishers
        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

    def ins_pose_callback(self,pose):
        self.NEDx = pose.x
        self.NEDy = pose.y
        self.yaw = pose.theta

    def objs_callback(self,data):
        self.objects_list = []
        for i in range(data.len):
            if str(data.objects[i].clase) == 'bouy':
                self.objects_list.append({'X' : data.objects[i].X + self.offset, 
                                      'Y' : data.objects[i].Y, 
                                      'color' : data.objects[i].color, 
                                      'class' : data.objects[i].clase})

    def center_point(self):
        '''
        @name: center_point
        @brief: Returns two waypoints as desired positions. The first waypoint is
          between the pair of obstacles (gate) and the second a distance to the front 
        @param: --
        @return: --
        '''
        x_list = []
        y_list = []
        class_list = []
        distance_list = []
        for i in range(len(self.objects_list)):
            x_list.append(self.objects_list[i]['X'])
            y_list.append(self.objects_list[i]['Y'])
            class_list.append(self.objects_list[i]['class'])
            distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))

        ind_g1 = np.argsort(distance_list)[0]
        ind_g2 = np.argsort(distance_list)[1]

        x1 = x_list[ind_g1]
        y1 = -1*y_list[ind_g1]
        x2 = x_list[ind_g2]
        y2 = -1*y_list[ind_g2]
        xc = min([x1,x2]) + abs(x1 - x2)/2
        yc = min([y1,y2]) + abs(y1 - y2)/2

        if y1 < y2:
            yl = y1
            xl = x1
            yr = y2
            xr = x2
        else:
            yl = y2
            xl = x2
            yr = y1
            xr = x1

        yd = yl - yr
        xd = xl - xr

        alpha = math.atan2(yd,xd) + math.pi/2
        if (abs(alpha) > (math.pi)):
            alpha = (alpha/abs(alpha))*(abs(alpha) - 2*math.pi)

        self.ned_alpha = alpha + self.yaw
        if (abs(self.ned_alpha) > (math.pi)):
            self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha) - 2*math.pi)

        xm, ym = self.gate_to_body(3,0,alpha,xc,yc)

        self.target_x, self.target_y = self.body_to_ned(xm, ym)
        
        path_array = Float32MultiArray()
        path_array.layout.data_offset = 5
        path_array.data = [xc, yc, xm, ym, 2]

        self.desired(path_array)

    def calculate_distance_to_boat(self):
        '''
        @name: calculate_distance_to_boat
        @brief: Returns the distance from the USV to the next gate
        @param: --
        @return: --
        '''
        x_list = []
        y_list = []
        class_list = []
        distance_list = []
        for i in range(len(self.objects_list)):
            x_list.append(self.objects_list[i]['X'])
            y_list.append(self.objects_list[i]['Y'])
            class_list.append(self.objects_list[i]['class'])
            distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))

        ind_g1 = np.argsort(distance_list)[0]
        ind_g2 = np.argsort(distance_list)[1]

        x1 = x_list[ind_g1]
        y1 = -1*y_list[ind_g1]
        x2 = x_list[ind_g2]
        y2 = -1*y_list[ind_g2]
        xc = min([x1,x2]) + abs(x1 - x2)/2
        yc = min([y1,y2]) + abs(y1 - y2)/2

        self.distance = math.pow(xc*xc + yc*yc, 0.5)

    def farther(self):
        '''
        @name: farther
        @brief: Returns a waypoint farther to the front of the vehicle in the NED
          reference frame to avoid perturbations.
        @param: --
        @return: --
        '''
        self.target_x, self.target_y = self.gate_to_ned(1, 0, 
                                                        self.ned_alpha,
                                                        self.target_x,
                                                        self.target_y)
        path_array = Float32MultiArray()
        path_array.layout.data_offset = 3
        path_array.data = [self.target_x, self.target_y, 0]
        self.desired(path_array)

    def gate_to_body(self, gate_x2, gate_y2, alpha, body_x1, body_y1):
        '''
        @name: gate_to_body
        @brief: Coordinate transformation between gate and body reference frames.
        @param: gate_x2: obj x coordinate in gate reference frame
                gate_y2: obj y coordinate in gate reference frame
                alpha: angle between gate and body reference frames
                body_x1: gate x coordinate in body reference frame
                body_y1: gate y coordinate in body reference frame
        @return: body_x2: obj body x coordinate
                 body_y2: obj body y coordinate
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = np.array([[math.cos(alpha), -1*math.sin(alpha)],
                      [math.sin(alpha), math.cos(alpha)]])
        n = J.dot(p)
        body_x2 = n[0] + body_x1
        body_y2 = n[1] + body_y1
        return (body_x2, body_y2)

    def body_to_ned(self, x, y):
        '''
        @name: body_to_ned
        @brief: Coordinate transformation between body and NED reference frames.
        @param: x: boat x coordinate in body reference frame
                y: boat y coordinate in body reference frame
        @return: nedx: boat x coordinate in ned reference frame
                 nedy: boat y coordinate in ned reference frame
        '''
        p = np.array([x, y])
        J = np.array([[math.cos(self.yaw), -1*math.sin(self.yaw)],
                      [math.sin(self.yaw), math.cos(self.yaw)]])
        n = J.dot(p)
        nedx = n[0] + self.NEDx
        nedy = n[1] + self.NEDy
        return (nedx, nedy)

    def gate_to_ned(self, gate_x2, gate_y2, alpha, ned_x1, ned_y1):
        '''
        @name: gate_to_ned
        @brief: Coordinate transformation between gate and NED reference frames.
        @param: gate_x2: obj x coordinate in gate reference frame
                gate_y2: obj y coordinate in gate reference frame
                alpha: angle between gate and ned reference frames
                body_x1: gate x coordinate in ned reference frame
                body_y1: gate y coordinate in ned reference frame
        @return: body_x2: obj ned x coordinate
                 body_y2: obj ned y coordinate
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = np.array([[math.cos(alpha), -1*math.sin(alpha)],
                      [math.sin(alpha), math.cos(alpha)]])
        n = J.dot(p)
        ned_x2 = n[0] + ned_x1
        ned_y2 = n[1] + ned_y1
        return (ned_x2, ned_y2)

    def desired(self, path):
    	self.path_pub.publish(path)

def main():
    rospy.init_node("auto_nav_position", anonymous=False)
    rate = rospy.Rate(100)
    autoNav = AutoNav()
    autoNav.distance = 4
    last_detection = []
    while not rospy.is_shutdown() and autoNav.activated:
        if autoNav.objects_list != last_detection:
            if autoNav.state == -1:
                while (not rospy.is_shutdown()) and (len(autoNav.objects_list) < 2):
                    autoNav.test.publish(autoNav.state)
                    rate.sleep()
                autoNav.state = 0
                last_detection = autoNav.objects_list

            if autoNav.state == 0:
                autoNav.test.publish(autoNav.state)
                if len(autoNav.objects_list) >= 2:
                    autoNav.calculate_distance_to_boat()
                if (len(autoNav.objects_list) >= 2) and (autoNav.distance >= 2):
                    autoNav.center_point()
                else:
                    initTime = rospy.Time.now().secs
                    while ((not rospy.is_shutdown()) and 
                        (len(autoNav.objects_list) < 2 or autoNav.distance < 2)):
                        if rospy.Time.now().secs - initTime > 2:
                            autoNav.state = 1
                            rate.sleep()
                            break
                last_detection = autoNav.objects_list

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

        if autoNav.objects_list != last_detection:
            if autoNav.state == 2:
                autoNav.test.publish(autoNav.state)
                if len(autoNav.objects_list) >= 2:
                    autoNav.calculate_distance_to_boat()
                if len(autoNav.objects_list) >= 2 and autoNav.distance >= 2:
                    autoNav.center_point()
                else:
                    initTime = rospy.Time.now().secs
                    while ((not rospy.is_shutdown()) and 
                        (len(autoNav.objects_list) < 2 or autoNav.distance < 2)):
                        if rospy.Time.now().secs - initTime > 2:
                            autoNav.state = 3
                            rate.sleep()
                            break
                last_detection = autoNav.objects_list

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
