#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: obstacle_channel.py
    @date: Mon Jun 8, 2020
    @modified: Sat May 15, 2021
    @author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @brief: Motion planning. Script to navigate a USV through
      an obstacle channel.
    @version: 1.0
    Open source
---------------------------------------------------------
'''

import math
import time

# import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose2D

from usv_perception.msg import obj_detected, obj_detected_list

# Class Definition
class ObsChan:
    def __init__(self):
        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.distance = 0
        self.distance_to_last = 0
        self.offset = .25 #camera to ins offset
        self.ned_channel_origin_x = 0
        self.ned_channel_origin_y = 0
        self.ned_alpha = 0

        self.waypoint_array_x = []
        self.waypoint_array_y = []
        self.path = []
        self.next_x = 0
        self.next_y = 0

        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)
        #rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.objs_callback)

        # ROS Publishers
        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

    def ins_pose_callback(self,pose):
        self.ned_x = pose.x
        self.ned_y = pose.y
        self.yaw = pose.theta

    def objs_callback(self,data):
        self.objects_list = []
        for i in range(data.len):
            if str(data.objects[i].clase) == 'buoy' and data.objects[i].X > 0.0:
                self.objects_list.append({'X' : data.objects[i].X + self.offset, 
                                      'Y' : -data.objects[i].Y, #Negate sensor input in Y
                                      'color' : data.objects[i].color, 
                                      'class' : data.objects[i].clase})

    def generate_obstacle_list(self):
        '''
        @name: generate_obstacle_list
        @brief: Returns four lists (x, y, class, distance), representing each obstacle 
        @param: --
        @return: x_list (list), y_list (list), class_list (list), distance_list (list)
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
        return (x_list, y_list, class_list, distance_list)

    def find_closer_buoys(self, distance_list):
        '''
        @name: find_closer_buoys
        @brief: Returns the index of the two closest buoys 
        @param: --
        @return: ind_b1 (int), ind_b2 (int)
        '''
        ind_b1 = np.argsort(distance_list)[0]
        ind_b2 = np.argsort(distance_list)[1]
        return (ind_b1, ind_b2)

    def new_reference_frame(self):
        '''
        @name: generate_obstacle_list
        @brief: Returns four lists (x, y, class, distance), representing each obstacle 
        @param: --
        @return: x_list (list), y_list (list), class_list (list), distance_list (list)
        '''
        x_list, y_list, class_list, distance_list = self.generate_obstacle_list()
        ind_b1, ind_b2 = self.find_closer_buoys(distance_list)
        x1 = x_list[ind_b1]
        y1 = y_list[ind_b1]
        x2 = x_list[ind_b2]
        y2 = y_list[ind_b2]
        x_center = min([x1,x2]) + abs(x1 - x2)/2
        y_center = min([y1,y2]) + abs(y1 - y2)/2

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

        self.ned_channel_origin_x, self.ned_channel_origin_y = self.body_to_ned(x_center, y_center)

    def rotate_obstacles_to_gate(self, x_list, y_list):
        x_gate_list = []
        y_gate_list = []
        for i in range(len(self.objects_list)):
            x_body = x_list[i]
            y_body = y_list[i]
            x_ned, y_ned = self.body_to_ned(x_body, y_body)
            x_gate, y_gate = self.ned_to_gate(x_ned, y_ned)
            x_gate_list.append(x_gate)
            y_gate_list.append(y_gate)
        return (x_gate_list, y_gate_list)

    def compute_path(self):
        '''
        @name: compute_path
        @brief: Returns a path to follow to navigate in an obstacle channel 
        @param: --
        @return: --
        '''
        x_list, y_list, class_list, distance_list = self.generate_obstacle_list()
        x_gate_list, y_gate_list = self.rotate_obstacles_to_gate(x_list, y_list)
        self.counted_pairs = []
        counted = False
        counted_2 = False
        same_x = []
        same_y = []
        self.waypoint_array_x = []
        self.waypoint_array_y = []
        self.path = []
        for i in range(len(self.objects_list)):
            for k in range(len(self.counted_pairs)):
                if i == self.counted_pairs[k]:
                    counted = True
            if counted != True:
                self.counted_pairs.append(i)
                current_x = x_gate_list[i]
                same_x = []
                same_y = []
                same_x.append(current_x)
                same_y.append(y_gate_list[i])
                for j in range(len(self.objects_list)):
                    if j != i:
                        for n in range(len(self.counted_pairs)):
                            if j == self.counted_pairs[n]:
                                counted_2 = True
                        if counted_2 != True:
                            comparison_x = x_gate_list[j]
                            if (comparison_x > (current_x - 0.5)) and (comparison_x < current_x + 0.5):
                                same_x.append(comparison_x)
                                same_y.append(y_gate_list[j])
                                self.counted_pairs.append(j)
                                if len(same_y) > 1:
                                    last = len(same_y) - 1
                                    left_index = np.argsort(same_y)[0]
                                    right_index = np.argsort(same_y)[last]
                                    x_left = same_x[left_index]
                                    y_left = same_y[left_index]
                                    x_right = same_x[right_index]
                                    y_right = same_y[right_index]
                                    x_center = min([x_left,x_right]) + abs(x_left - x_right)/2
                                    y_center = y_left + (y_right - y_left)/2
                                    self.waypoint_array_x.append(x_center)
                                    self.waypoint_array_y.append(y_center)
                        counted_2 = False
            counted = False
        sorted_waypoint_array_x = np.argsort(self.waypoint_array_x)
        for m in range(len(sorted_waypoint_array_x)):
            index = sorted_waypoint_array_x[m]
            ned_wp_x, ned_wp_y = self.gate_to_ned(self.waypoint_array_x[index], self.waypoint_array_y[index])
            self.path.append(ned_wp_x)
            self.path.append(ned_wp_y)
        last = len(sorted_waypoint_array_x) - 1
        if last > 0:
            index = sorted_waypoint_array_x[1]
        else:
            index = sorted_waypoint_array_x[0]
        self.next_x, self.next_y = self.gate_to_ned(self.waypoint_array_x[index], self.waypoint_array_y[index])
        self.last_x, self.last_y = self.gate_to_ned(self.waypoint_array_x[last] + 3, self.waypoint_array_y[last])
        self.path.append(self.last_x)
        self.path.append(self.last_y)
        self.path.append(0)
        path_array = Float32MultiArray()
        path_array.layout.data_offset = len(self.path)
        path_array.data = self.path

        self.desired(path_array)

    def compute_distance(self):
        '''
        @name: compute_distance
        @brief: Returns the distance between the USV and the next waypoint 
        @param: --
        @return: --
        '''
        x_dis = self.next_x - self.ned_x
        y_dis = self.next_y - self.ned_y
        self.distance = math.pow(x_dis*x_dis + y_dis*y_dis, 0.5)

    def compute_distance_to_last(self):
        '''
        @name: compute_distance_to_last
        @brief: Returns the distance between the USV and the last waypoint 
        @param: --
        @return: --
        '''
        x_dis = self.last_x - self.ned_x
        y_dis = self.last_y - self.ned_y
        self.distance_to_last = math.pow(x_dis*x_dis + y_dis*y_dis, 0.5)

    def gate_to_body(self, gate_x2, gate_y2, alpha, body_x1, body_y1):
        '''
        @name: gate_to_body
        @brief: Coordinate transformation between gate and body reference frames.
        @param: gate_x2: target x coordinate in gate reference frame
                gate_y2: target y coordinate in gate reference frame
                alpha: angle between gate and body reference frames
                body_x1: gate x coordinate in body reference frame
                body_y1: gate y coordinate in body reference frame
        @return: body_x2: target x coordinate in body reference frame
                 body_y2: target y coordinate in body reference frame
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = self.rotation_matrix(alpha)
        n = J.dot(p)
        body_x2 = n[0] + body_x1
        body_y2 = n[1] + body_y1
        return (body_x2, body_y2)

    def body_to_ned(self, x2, y2):
        '''
        @name: body_to_ned
        @brief: Coordinate transformation between body and NED reference frames.
        @param: x2: target x coordinate in body reference frame
                y2: target y coordinate in body reference frame
        @return: ned_x2: target x coordinate in NED reference frame
                 ned_y2: target y coordinate in NED reference frame
        '''
        p = np.array([x2, y2])
        J = self.rotation_matrix(self.yaw)
        n = J.dot(p)
        ned_x2 = n[0] + self.ned_x
        ned_y2 = n[1] + self.ned_y
        return (ned_x2, ned_y2)

    def gate_to_ned(self, gate_x2, gate_y2):
        '''
        @name: gate_to_ned
        @brief: Coordinate transformation between gate and NED reference frames.
        @param: gate_x2: target x coordinate in gate reference frame
                gate_y2: target y coordinate in gate reference frame
        @return: ned_x2: target x coordinate in NED reference frame
                 ned_y2: target y coordinate in NED reference frame
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = self.rotation_matrix(self.ned_alpha)
        n = J.dot(p)
        ned_x2 = n[0] + self.ned_channel_origin_x
        ned_y2 = n[1] + self.ned_channel_origin_y
        return (ned_x2, ned_y2)

    def ned_to_gate(self, ned_x2, ned_y2):
        '''
        @name: ned_to_gate
        @brief: Coordinate transformation between NED and gate reference frames.
        @param: ned_x2: target x coordinate in NED reference frame
                ned_y2: target y coordinate in NED reference frame
        @return: gate_x2: target x coordinate in gate reference frame
                 gate_y2: target y coordinate in gate reference frame
        '''
        p = np.array([ned_x2-self.ned_channel_origin_x, ned_y2-self.ned_channel_origin_y])
        J = self.rotation_matrix(self.ned_alpha)
        Jinv = np.linalg.inv(J)
        n = Jinv.dot(p)
        gate_x2 = n[0]
        gate_y2 = n[1]
        return (gate_x2, gate_y2)

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
    rospy.init_node("auto_nav_position", anonymous=False)
    rate = rospy.Rate(20)
    obsChan = ObsChan()
    last_detection = []
    time.sleep(10)
    while not rospy.is_shutdown() and obsChan.activated:
        if obsChan.objects_list != last_detection:
            if obsChan.state == -1:
                while (not rospy.is_shutdown()) and (len(obsChan.objects_list) < 2):
                    obsChan.test.publish(obsChan.state)
                    rate.sleep()
                obsChan.state = 0
                obsChan.new_reference_frame()
                obsChan.compute_path()
                last_detection = obsChan.objects_list

            if obsChan.state == 0:
                obsChan.test.publish(obsChan.state)
                obsChan.compute_distance()
                if obsChan.distance < 2:
                    if len(obsChan.objects_list) >= 2:
                        obsChan.compute_path()
                    else:
                        obsChan.compute_distance_to_last()
                        if obsChan.distance_to_last < 2:
                            obsChan.state = 1
                last_detection = obsChan.objects_list

        elif obsChan.state == 1:
            obsChan.test.publish(obsChan.state)
            time.sleep(1)
            obsChan.status_pub.publish(3)

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
