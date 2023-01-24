#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: obstacle_field.py
    @date: Sat May 15, 2021
    @author: Alejandro Gonzalez Garcia
    @e-mail: alexglzg97@gmail.com
    @brief: Motion planning. Script to select the marker
        the USV has to circle in the Obstacle Field
        challenge for the RoboBoat competition. Motion 
        planning selected to circle and send waypoints
        to an obstacle avoidance script.
    @version: 1.0
    Open source
---------------------------------------------------------
'''

import math
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Int32, String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from usv_perception.msg import obj_detected, obj_detected_list

#EARTH_RADIUS = 6371000

# Class Definition
class ObsField:
    def __init__(self):
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0
        self.lat = 0
        self.lon = 0
        self.InitTime = rospy.Time.now().secs
        self.distance = 0
        self.offset = .55 #camera to ins offset
        self.target_x = 0
        self.target_y = 0
        self.gate_x = 0
        self.gate_y = 0
        self.ned_alpha = 0
        
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
            if str(data.objects[i].clase) == 'marker':
                self.objects_list.append({'X' : data.objects[i].X + self.offset,
                                          'Y' : data.objects[i].Y,
                                          'color' : data.objects[i].color, 
                                          'class' : data.objects[i].clase})

    def buoy_waypoints(self,buoy_x,buoy_y):
        '''
        @name: buoy_waypoints
        @brief: Returns 4 waypoints. The first three form a circle a certain radius
          from the obstacle, and the next waypoints return the vehicle the starting
          point
        @param: buoy_x: buoy x coordinate
                buoy_y: buoy y coordinate
        @return: --
        '''
        rospy.loginfo("Marker waypoints has just started")
            
        radius = 2.0

        w1 = [buoy_x, buoy_y + radius]
        w2 = [buoy_x + radius, buoy_y]
        w3 = [buoy_x, buoy_y - radius]

        path_array = Float32MultiArray()
        path_array.layout.data_offset = 11
        
        w1_x, w1_y = self.body_to_ned(w1[0], w1[1])
        w2_x, w2_y = self.body_to_ned(w2[0], w2[1])
        w3_x, w3_y = self.body_to_ned(w3[0], w3[1])
        path_array.data = [self.ned_x, self.ned_y, 
                          w1_x, w1_y, w2_x, w2_y, w3_x, w3_y,
                          self.ned_x, self.ned_y, 0]
        self.desired(path_array)

        return(self.ned_x, self.ned_y)

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
    rospy.init_node("obstacle_field", anonymous=False)
    rate = rospy.Rate(20)
    obsField = ObsField()
    last_detection = []
    while not rospy.is_shutdown() and obsField.activated:
        if obsField.objects_list != last_detection:
            if obsField.state == -1:
                while (not rospy.is_shutdown()) and (len(obsField.objects_list) < 0):
                    obsField.test.publish(obsField.state)
                    rate.sleep()
                obsField.state = 0
                last_detection = obsField.objects_list
            if obsField.state == 0:
                obsField.test.publish(obsField.state)
                x_list = []
                y_list = []
                class_list = []
                distance_list = []
                for i in range(len(obsField.objects_list)):
                    x_list.append(obsField.objects_list[i]['X'])
                    y_list.append(obsField.objects_list[i]['Y'])
                    class_list.append(obsField.objects_list[i]['class'])
                    distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))
                    ind_0 = np.argsort(distance_list)[0]
                if ((len(obsField.objects_list) >= 1)):
                    marker_x = obsField.objects_list[ind_0]['X']
                    marker_y = - obsField.objects_list[ind_0]['Y']
                    obsField.state = 1
                last_detection = obsField.objects_list
            if obsField.state == 1:
                obsField.test.publish(obsField.state)
                x_final, y_final = obsField.buoy_waypoints(marker_x,marker_y)
                obsField.state = 2
                last_detection = obsField.objects_list
            if obsField.state == 2:
                obsField.test.publish(obsField.state)
                x_squared = math.pow(x_final - obsField.ned_x, 2)
                y_squared = math.pow(y_final - obsField.ned_y, 2)
                distance_final = math.pow(x_squared + y_squared, 0.5)
                if distance_final > 1:
                    pass
                else:
                    obsField.status_pub.publish(4)
                last_detection = obsField.objects_list
        rate.sleep()    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
