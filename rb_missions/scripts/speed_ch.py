#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math
import time
import matplotlib.pyplot as plt

#EARTH_RADIUS = 6371000

class SpeedCh:
    def __init__(self):
        self.obj_list = []
        self.activated = True
        self.state = -1
        self.NEDx = 0
        self.NEDy = 0
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
        
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber('/usv_perception/yolo_zed/objects_detected', obj_detected_list, self.objs_callback)
        self.path_pub = rospy.Publisher('/mission/waypoints', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/test", Int32, queue_size=10)

    def ins_pose_callback(self,pose):
        self.NEDx = pose.x
        self.NEDy = pose.y
        self.yaw = pose.theta

    def objs_callback(self,data):
        self.obj_list = []
        for i in range(data.len):
            if str(data.objects[i].clase) == 'bouy':
                self.obj_list.append({'X' : data.objects[i].X + self.offset, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})
    '''
    def gps_point_trans(self,y,x):
        p = np.array([x,y])
        J = np.array([[math.cos(self.yaw), -1*math.sin(self.yaw)],[math.sin(self.yaw), math.cos(self.yaw)]])
        n = J.dot(p)

        phi1 = math.radians(self.lat)

        latitude2  = self.lat  + (n[1] / EARTH_RADIUS) * (180 / math.pi)
        longitude2 = self.lon + (n[0] / EARTH_RADIUS) * (180 / math.pi) / math.cos(phi1)
        
        return latitude2,longitude2
    '''
    def punto_medio(self):
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

        alpha = math.atan2(yd,xd) + math.pi/2
        if (abs(alpha) > (math.pi)):
            alpha = (alpha/abs(alpha))*(abs(alpha)-2*math.pi)

        self.ned_alpha = alpha + self.yaw
        if (abs(self.ned_alpha) > (math.pi)):
            self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha)-2*math.pi)

        xm, ym = self.gate_to_body(2,0,alpha,xc,yc)

        self.target_x, self.target_y = self.body_to_ned(xm, ym)
        self.gate_x, self.gate_y = self.body_to_ned(xc, yc)
        
        obj = Float32MultiArray()
        obj.layout.data_offset = 5
        obj.data = [xc, yc, xm, ym, 2]

        self.desired(obj)
        
    def waypoints_vuelta(self,v_x,v_y):
        
        print('Empezo waypoints')
            
        radio = 3

        w1 = [v_x,v_y+radio]
        w2 = [v_x+radio,v_y]
        w3 = [v_x,v_y-radio]

        obj = Float32MultiArray()
        obj.layout.data_offset = 11
        
        w1_x, w1_y = self.body_to_ned(w1[0],w1[1])
        w2_x, w2_y = self.body_to_ned(w2[0],w2[1])
        w3_x, w3_y = self.body_to_ned(w3[0],w3[1])

        w5_x, w5_y = self.gate_to_ned(-3, 0, self.ned_alpha, self.gate_x, self.gate_y)

        #obj.data = [(self.gps_point_trans(w1[0],w1[1]))[0],(self.gps_point_trans(w1[0],w1[1]))[1],(self.gps_point_trans(w2[0],w2[1]))[0],(self.gps_point_trans(w2[0],w2[1]))[1],(self.gps_point_trans(w3[0],w3[1]))[0],(self.gps_point_trans(w3[0],w3[1]))[1],self.start_gps[0],self.start_gps[1],1]
        obj.data = [w1_x, w1_y, w2_x, w2_y, w3_x, w3_y, self.gate_x, self.gate_y, w5_x, w5_y, 0]

        self.desired(obj)

    def farther(self):
        
        self.target_x, self.target_y = self.gate_to_ned(1.5, 0, self.ned_alpha, self.target_x, self.target_y)
        
        obj = Float32MultiArray()
        obj.layout.data_offset = 3
        obj.data = [self.target_x, self.target_y, 0]

        self.desired(obj)

    def gate_to_body(self, gate_x2, gate_y2, alpha, body_x1, body_y1):
        p = np.array([[gate_x2],[gate_y2]])
        J = np.array([[math.cos(alpha), -1*math.sin(alpha)],[math.sin(alpha), math.cos(alpha)]])
        n = J.dot(p)

        body_x2 = n[0] + body_x1
        body_y2 = n[1] + body_y1

        return (body_x2, body_y2)

    def body_to_ned(self, x, y):
        p = np.array([x,y])
        J = np.array([[math.cos(self.yaw), -1*math.sin(self.yaw)],[math.sin(self.yaw), math.cos(self.yaw)]])
        n = J.dot(p)

        nedx = n[0] + self.NEDx
        nedy = n[1] + self.NEDy

        return (nedx, nedy)

    def gate_to_ned(self, gate_x2, gate_y2, alpha, ned_x1, ned_y1):
        p = np.array([[gate_x2],[gate_y2]])
        J = np.array([[math.cos(alpha), -1*math.sin(alpha)],[math.sin(alpha), math.cos(alpha)]])
        n = J.dot(p)

        ned_x2 = n[0] + ned_x1
        ned_y2 = n[1] + ned_y1

        return (ned_x2, ned_y2)

    def desired(self, obj):

    	self.path_pub.publish(obj)
    
def main():
    rospy.init_node('speed_ch', anonymous=True)
    rate = rospy.Rate(100)    
    E = SpeedCh()
    E.distance = 4
    while not rospy.is_shutdown() and E.activated:

        if E.state == -1:
            while not rospy.is_shutdown() and len(E.obj_list) < 2:
                E.test.publish(E.state)
                rate.sleep()
            E.state = 0

        elif E.state == 0:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2 and E.distance >= 3:
                E.punto_medio()
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list) < 2 or E.distance < 3):
                    if rospy.Time.now().secs - initTime > 3:
                        E.state = 1
                        rate.sleep()
                        break

        elif E.state == 1:
            E.test.publish(E.state)
            x_list = []
            y_list = []
            class_list = []
            distance_list = []
            for i in range(len(E.obj_list)):
                x_list.append(E.obj_list[i]['X'])
                y_list.append(E.obj_list[i]['Y'])
                class_list.append(E.obj_list[i]['class'])
                distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))
                ind_0 = np.argsort(distance_list)[0]
            if len(E.obj_list) >= 1 and (str(E.obj_list[ind_0]['color']) == 'blue'):
                E.state = 2
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list)) < 1:
                    if rospy.Time.now().secs - initTime > 1:
                        E.farther()
                        rate.sleep()
                        break

        elif E.state == 2:
            E.test.publish(E.state)
            x_list = []
            y_list = []
            class_list = []
            distance_list = []
            for i in range(len(E.obj_list)):
                x_list.append(E.obj_list[i]['X'])
                y_list.append(E.obj_list[i]['Y'])
                class_list.append(E.obj_list[i]['class'])
                distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))
                ind_0 = np.argsort(distance_list)[0]
            if (len(E.obj_list) >= 1) and (E.obj_list[ind_0]['X'] < 8):
                v_x = E.obj_list[0]['X']
                v_y = E.obj_list[0]['Y']
                E.state = 3
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list)) < 1:
                    if rospy.Time.now().secs - initTime > 1:
                        E.farther()
                        rate.sleep()
                        break
                
        elif E.state == 3:
            E.test.publish(E.state)
            E.waypoints_vuelta(v_x,v_y)
            E.state = 4

        elif E.state == 4:
            E.test.publish(E.state)
            time.sleep(1)
            E.status_pub.publish(1)

        rate.sleep()    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
