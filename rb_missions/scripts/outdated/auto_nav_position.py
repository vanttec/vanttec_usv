#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: auto_nav_position.py
    @date: Thu Dec 26, 2019
    @modified: Sat May 15, 2021
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
        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.distance = 0
        self.InitTime = rospy.Time.now().secs
        self.offset = .25 #camera to ins offset
        self.target_x = 0
        self.target_y = 0
        self.ned_alpha = 0

        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)
        #rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.objs_callback)

        # ROS Publishers
        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)
    
    def gates_in_front(self):
        count = 0
        for i in range(len(self.objects_list)):
            if self.objects_list[i]['X'] > 0:
                count+=1
        if count >= 2:
            return True
        return False

    def ins_pose_callback(self,pose):
        self.ned_x = pose.x
        self.ned_y = pose.y
        self.yaw = pose.theta

    def objs_callback(self,data):
        self.objects_list = []
        for i in range(data.len):
            # Checar que las boyas estén entre (0.2 -> 5) en 'x' y entre (-5 -> 5) en 'y'
            if str(data.objects[i].clase) == 'buoy' and (data.objects[i].X > 0.2) and (data.objects[i].X < 5) and (data.objects[i].Y < 5) and (data.objects[i].Y > -5):
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
        
        # Almacenar los datos de las boyas en una lista
        x_list = []
        y_list = []
        class_list = []
        distance_list = []
        for i in range(len(self.objects_list)):
            x_list.append(self.objects_list[i]['X'])
            y_list.append(self.objects_list[i]['Y'])
            class_list.append(self.objects_list[i]['class'])
            distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))

        # Encontrar los indices de la boya 1 y la boya 2 dependiendo de cuál está más cerca
        ind_g1 = np.argsort(distance_list)[0]
        ind_g2 = np.argsort(distance_list)[1]

        # Asignar variables para cada coordenada
        x1 = x_list[ind_g1]
        y1 = -1*y_list[ind_g1]
        x2 = x_list[ind_g2]
        y2 = -1*y_list[ind_g2]

        # Coordenadas del punto central entre las dos boyas
        xc = (x1 + x2)/2
        yc = (y1 + y2)/2

        # Asignar cada par de coordenadas a la izquierda o derecha
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

        # Calcular la distancia en 'x' y 'y' entre las dos boyas
        yd = yl - yr
        xd = xl - xr

        # Angulo entre xd y yd + 90 grados en el marco del barco (porque debe de estar con respecto a 'surge')
        alpha = math.atan2(yd,xd) + math.pi/2
        if (abs(alpha) > (math.pi)):
            # Modificar alpha de tal forma que se encuentre entre -pi y pi
            alpha = (alpha/abs(alpha))*(abs(alpha) - 2*math.pi)

        # Modificar el angulo actual del barco para agregarle el angulo alpha para representar la orientacion en ned
        self.ned_alpha = alpha + self.yaw
        if (abs(self.ned_alpha) > (math.pi)):
            # Modificar el ned_alpha para que vaya de entre -pi y pi
            self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha) - 2*math.pi)

        # Coordenadas deseadas en el marco Body de 3 metros adelante de [xc,yc] sobre la línea que se traza a 'alpha' grados de ese punto
        xm, ym = self.gate_to_body(3,0,alpha,xc,yc)

        # Coordenadas deseadas en marco NED
        self.target_x, self.target_y = self.body_to_ned(xm, ym)
        
        # Mandar los datos con las coordenadas hacia donde ir, primero hacia el punto entre las dos boyas,
        # y luego con el punto 3 metros adelante de dichas coordenadas
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
        xc = (x1 + x2)/2
        yc = (y1 + y2)/2

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
        @return: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        '''
        p = np.array([x2, y2])
        J = self.rotation_matrix(self.yaw)
        n = J.dot(p)
        ned_x2 = n[0] + self.ned_x
        ned_y2 = n[1] + self.ned_y
        return (ned_x2, ned_y2)

    def gate_to_ned(self, gate_x2, gate_y2, alpha, ned_x1, ned_y1):
        '''
        @name: gate_to_ned
        @brief: Coordinate transformation between gate and NED reference frames.
        @param: gate_x2: target x coordinate in gate reference frame
                gate_y2: target y coordinate in gate reference frame
                alpha: angle between gate and ned reference frames
                body_x1: gate x coordinate in ned reference frame
                body_y1: gate y coordinate in ned reference frame
        @return: body_x2: target x coordinate in ned reference frame
                 body_y2: target y coordinate in ned reference frame
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = self.rotation_matrix(alpha)
        n = J.dot(p)
        ned_x2 = n[0] + ned_x1
        ned_y2 = n[1] + ned_y1
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
    rospy.init_node("auto_nav_position", anonymous=False)
    rate = rospy.Rate(10)
    autoNav = AutoNav()
    autoNav.distance = 4
    last_detection = []
    time.sleep(10)
    while not rospy.is_shutdown() and autoNav.activated:
        if autoNav.objects_list != last_detection or autoNav.state < 1:
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
                if (len(autoNav.objects_list) >= 2) and (autoNav.distance >= 3):
                    autoNav.center_point()
                else:
                    x_dif = autoNav.target_x - autoNav.ned_x
                    y_dif = autoNav.target_y - autoNav.ned_y
                    dist = math.pow(x_dif**2 + y_dif**2, 0.5) 
                    if dist < 3:
                        autoNav.state = 1
                        rate.sleep()
                '''else:
                    initTime = rospy.Time.now().secs
                    while ((not rospy.is_shutdown()) and 
                        (len(autoNav.objects_list) < 2 or autoNav.distance < 2)):
                        if rospy.Time.now().secs - initTime > 1:
                            autoNav.state = 1
                            rate.sleep()
                            break'''
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

        if autoNav.objects_list != last_detection or autoNav.state < 3:
            if autoNav.state == 2:
                autoNav.test.publish(autoNav.state)
                if len(autoNav.objects_list) >= 2:
                    autoNav.calculate_distance_to_boat()
                if len(autoNav.objects_list) >= 2 and autoNav.distance >= 3:
                    autoNav.center_point()
                else:
                    x_dif = autoNav.target_x - autoNav.ned_x
                    y_dif = autoNav.target_y - autoNav.ned_y
                    dist = math.pow(x_dif**2 + y_dif**2, 0.5) 
                    if dist < 3:
                        autoNav.state = 3
                        rate.sleep()
                '''else:
                    initTime = rospy.Time.now().secs
                    while ((not rospy.is_shutdown()) and 
                        (len(autoNav.objects_list) < 2 or autoNav.distance < 2)):
                        if rospy.Time.now().secs - initTime > 2:
                            autoNav.state = 3
                            rate.sleep()
                            break'''
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
