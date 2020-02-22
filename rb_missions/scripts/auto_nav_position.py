#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: auto_nav_guidance.py
    @date: Thu Dec 26, 2019
    @modified: Wed Feb 5, 2020
    @author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @co-author: Rodolfo Cuan Urquizo
    @e-mail: fitocuan@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
    @brief: Motion planning. Script to navigate a USV through two sets of buoys 
            or markers, all by receiving obstacle positions and sending a 
            desired position for the USV.
    @version: 1.1
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
        self.obj_list = []
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
        self.status_pub = rospy.Publisher("/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/test", Int32, queue_size=10)

    def ins_pose_callback(self,_pose):
        self.NEDx = _pose.x
        self.NEDy = _pose.y
        self.yaw = _pose.theta

    def objs_callback(self,_data):
        self.obj_list = []
        for i in range(_data.len):
            if str(_data.objects[i].clase) == 'bouy': #Change to buoy when obj.names in usv_perception is changed, in RoboBoat change to marker
                self.obj_list.append({'X' : _data.objects[i].X + self.offset, 
                                      'Y' : _data.objects[i].Y, 
                                      'color' : _data.objects[i].color, 
                                      'class' : _data.objects[i].clase})

    def middle_point(self):
        '''
        @name: middle_point
        @brief: Returns two waypoints as desired positions. The first waypoint is
          between the pair of obstacles (gate) and the second a distance to the front 
        @param: --
        @return: --
        '''
        x_list = []
        y_list = []
        class_list = []
        distance_list = []
        for i in range(len(self.obj_list)):
            x_list.append(self.obj_list[i]['X'])
            y_list.append(self.obj_list[i]['Y'])
            class_list.append(self.obj_list[i]['class'])
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

        _alpha = math.atan2(yd,xd) + math.pi/2
        if (abs(_alpha) > (math.pi)):
            _alpha = (_alpha/abs(_alpha))*(abs(_alpha) - 2*math.pi)

        self.ned_alpha = _alpha + self.yaw
        if (abs(self.ned_alpha) > (math.pi)):
            self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha) - 2*math.pi)

        xm, ym = self.gate_to_body(3,0,_alpha,xc,yc)

        self.target_x, self.target_y = self.body_to_ned(xm, ym)
        
        obj = Float32MultiArray()
        obj.layout.data_offset = 5
        obj.data = [xc, yc, xm, ym, 2]

        self.desired(obj)

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
        obj = Float32MultiArray()
        obj.layout.data_offset = 3
        obj.data = [self.target_x, self.target_y, 0]
        self.desired(obj)

    def gate_to_body(self, _gate_x2, _gate_y2, _alpha, _body_x1, _body_y1):
        '''
        @name: gate_to_body
        @brief: Coordinate transformation between gate and body reference frames.
        @param: _gate_x2: obj x coordinate in gate reference frame
                _gate_y2: obj y coordinate in gate reference frame
                _alpha: angle between gate and body reference frames
                _body_x1: gate x coordinate in body reference frame
                _body_y1: gate y coordinate in body reference frame
        @return: body_x2: obj body x coordinate
                 body_y2: obj body y coordinate
        '''
        p = np.array([[_gate_x2],[_gate_y2]])
        J = np.array([[math.cos(_alpha), -1*math.sin(_alpha)],
                      [math.sin(_alpha), math.cos(_alpha)]])
        n = J.dot(p)
        body_x2 = n[0] + _body_x1
        body_y2 = n[1] + _body_y1
        return (body_x2, body_y2)

    def body_to_ned(self, _x, _y):
        '''
        @name: body_to_ned
        @brief: Coordinate transformation between body and NED reference frames.
        @param: _x: boat x coordinate in body reference frame
                _y: boat y coordinate in body reference frame
        @return: nedx: boat x coordinate in ned reference frame
                 nedy: boat y coordinate in ned reference frame
        '''
        p = np.array([_x, _y])
        J = np.array([[math.cos(self.yaw), -1*math.sin(self.yaw)],
                      [math.sin(self.yaw), math.cos(self.yaw)]])
        n = J.dot(p)
        nedx = n[0] + self.NEDx
        nedy = n[1] + self.NEDy
        return (nedx, nedy)

    def gate_to_ned(self, _gate_x2, _gate_y2, _alpha, _ned_x1, _ned_y1):
        '''
        @name: gate_to_ned
        @brief: Coordinate transformation between gate and NED reference frames.
        @param: _gate_x2: obj x coordinate in gate reference frame
                _gate_y2: obj y coordinate in gate reference frame
                _alpha: angle between gate and ned reference frames
                _body_x1: gate x coordinate in ned reference frame
                _body_y1: gate y coordinate in ned reference frame
        @return: body_x2: obj ned x coordinate
                 body_y2: obj ned y coordinate
        '''
        p = np.array([[_gate_x2],[_gate_y2]])
        J = np.array([[math.cos(_alpha), -1*math.sin(_alpha)],
                      [math.sin(_alpha), math.cos(_alpha)]])
        n = J.dot(p)
        ned_x2 = n[0] + _ned_x1
        ned_y2 = n[1] + _ned_y1
        return (ned_x2, ned_y2)

    def desired(self, _obj):
    	self.path_pub.publish(_obj)

def main():
    rospy.init_node("auto_nav_position", anonymous=False)
    rate = rospy.Rate(100)
    autoNav = AutoNav()
    autoNav.distance = 4
    while not rospy.is_shutdown() and autoNav.activated:

        if autoNav.state == -1:
            while (not rospy.is_shutdown()) and (len(autoNav.obj_list) < 2):
                autoNav.test.publish(autoNav.state)
                rate.sleep()
            autoNav.state = 0

        elif autoNav.state == 0:
            autoNav.test.publish(autoNav.state)
            if (len(autoNav.obj_list) >= 2) and (autoNav.distance >= 3):
                autoNav.middle_point()
            else:
                initTime = rospy.Time.now().secs
                while ((not rospy.is_shutdown()) and 
                       (len(autoNav.obj_list) < 2 or autoNav.distance < 3)):
                    if rospy.Time.now().secs - initTime > 3:
                        autoNav.state = 1
                        rate.sleep()
                        break

        elif autoNav.state == 1:
            autoNav.test.publish(autoNav.state)
            if len(autoNav.obj_list) >= 2:
                autoNav.state = 2
            else:
                initTime = rospy.Time.now().secs
                while ((not rospy.is_shutdown()) and 
                       (len(autoNav.obj_list) < 2)):
                    if rospy.Time.now().secs - initTime > 1:
                        autoNav.farther()
                        rate.sleep()
                        break

        elif autoNav.state == 2:
            autoNav.test.publish(autoNav.state)
            if len(autoNav.obj_list) >= 2 and autoNav.distance >= 3:
                autoNav.middle_point()
            else:
                initTime = rospy.Time.now().secs
                while ((not rospy.is_shutdown()) and 
                       (len(autoNav.obj_list) < 2 or autoNav.distance < 3)):
                    if rospy.Time.now().secs - initTime > 3:
                        autoNav.state = 3
                        rate.sleep()
                        break

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
