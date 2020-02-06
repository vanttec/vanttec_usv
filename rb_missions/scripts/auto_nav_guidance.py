#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
OUTDATED SCRIPT
    @file: auto_nav_guidiance.py
    @modified: Wed Feb 5, 2020
	@author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @brief: Give a desired heading and velocity to the USV based on the position
            of markers in front of the USV, to navigate through such markers.
	@version: 1.0
    Open source
---------------------------------------------------------
'''

import math
import time

import numpy as np
import rospy
from std_msgs.msg import Int32, Float64, String
from custom_msgs.msg import obj_detected, obj_detected_list
from geometry_msgs.msg import Pose2D


class AutoNav:


    def __init__(self):


        self.yaw = 0
        self.obj_list = []
        self.activated = True
        self.state = -1
        self.ang = 0
        self.desired_speed = 0
        self.distance = 0
        self.InitTime = rospy.Time.now().secs
        self.desired_heading = 0

        rospy.Subscriber("/vectornav/ins_2d/ins_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/test", Int32, queue_size=10)

    def ins_pose_callback(self,pose):
        self.yaw = pose.theta

    def objs_callback(self,data):
        self.obj_list = []
        for i in range(data.len):
            if str(data.objects[i].clase) == 'marker':
                self.obj_list.append({'X' : data.objects[i].X + 0.55,
                                      'Y' : data.objects[i].Y, 
                                      'color' : data.objects[i].color, 
                                      'class' : data.objects[i].clase})

    def punto_medio(self):

        distances_list = []
        y_list = []
        class_list = []
        for i in range(len(self.obj_list)):
            distances_list.append(self.obj_list[i]['X'])
            y_list.append(self.obj_list[i]['Y'])
            class_list.append(self.obj_list[i]['class'])

        ind_x1 = np.argsort(distances_list)[0]
        ind_x2 = np.argsort(distances_list)[1]

        x1 = distances_list[ind_x1]
        y1 = -1*y_list[ind_x1]
        x2 = distances_list[ind_x2]
        y2 = -1*y_list[ind_x2]
        xc = min([x1,x2]) + abs(x1 - x2)/2
        yc = min([y1,y2]) + abs(y1 - y2)/2
        self.distance = xc
        offset = .55
        yc = 0.00001 if yc == 0 else yc

        relative_heading = math.atan((xc+offset)/yc)

        if relative_heading < 0:
            relative_heading = -1*(relative_heading + math.pi/2)
        else:
            relative_heading = math.pi/2 - relative_heading

        self.ang = -1 if relative_heading < 0 else 1

        self.desired_heading = relative_heading + self.yaw

        self.desired_speed = 1
        if self.distance < 7:
            self.desired_speed = 0.6
        if self.distance < 0.2:
            self.desired_speed = 0

        self.desired(self.desired_speed, self.desired_heading)

        '''plt.clf()
        plt.plot(y1,x1, 'go',markersize=5)
        plt.plot(y2,x2, 'go',markersize=5)
        plt.plot(0,0,'ro')
        plt.plot(yc,xc,'r*')
        plt.axis([-15, 15, 0, 20])
        plt.pause(0.0001)
        plt.draw()'''

    def straight(self):
        self.desired_speed = 0.7
        self.desired(self.desired_speed, self.yaw)

    def desired(self, speed, heading):
        self.d_speed_pub.publish(speed)
        self.d_heading_pub.publish(heading)

def main():
    rospy.init_node('auto_nav_guidance', anonymous=True)
    rate = rospy.Rate(100)
    E = AutoNav()
    while not rospy.is_shutdown() and E.activated:

        if E.state == -1:
            while not rospy.is_shutdown() and len(E.obj_list) < 2:
                E.test.publish(E.state)
                rate.sleep()
            E.state = 0

        if E.state == 0:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2:
                E.punto_medio()
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and len(E.obj_list) < 2:
                    if rospy.Time.now().secs - initTime > 5:
                        E.state = 1
                        rate.sleep()
                        break

        if E.state == 1:
            E.test.publish(E.state)
            E.straight()
            time.sleep(2)
            angle = E.yaw
            E.state = 2

        if E.state == 2:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2:
                E.state = 3
            else:
                E.desired(0.4,E.yaw)

        if E.state == 3:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2:
                E.punto_medio()
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and len(E.obj_list) < 2:
                    if rospy.Time.now().secs - initTime > 5:
                        E.state = 4
                        rate.sleep()
                        break

        if E.state == 4:
            E.test.publish(E.state)
            E.desired(0,E.yaw)
            E.activated = False
            time.sleep(1)
            E.status_pub.publish(1)

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
