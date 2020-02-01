#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
from geometry_msgs.msg import Pose2D
import numpy as np
import math
import time
import matplotlib.pyplot as plt

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
            if str(data.objects[i].clase) == 'marker':
                self.obj_list.append({'X' : data.objects[i].X + self.offset, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})

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

        xm, ym = self.gate_to_body(3,0,alpha,xc,yc)

        self.target_x, self.target_y = self.body_to_ned(xm, ym)
        
        obj = Float32MultiArray()
        obj.layout.data_offset = 5
        obj.data = [xc, yc, xm, ym, 2]

        self.desired(obj)

    def farther(self):
        
        self.target_x, self.target_y = self.gate_to_ned(1, 0, self.ned_alpha, self.target_x, self.target_y)
        
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
    rospy.init_node('auto_nav_position', anonymous=True)
    rate = rospy.Rate(100)
    E = AutoNav()
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
            if len(E.obj_list) >= 2:
                E.state = 2
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list)) < 2:
                    if rospy.Time.now().secs - initTime > 1:
                        E.farther()
                        rate.sleep()
                        break

        elif E.state == 2:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2 and E.distance >= 3:
                E.punto_medio()
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list) < 2 or E.distance < 3):
                    if rospy.Time.now().secs - initTime > 3:
                        E.state = 3
                        rate.sleep()
                        break

        elif E.state == 3:
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
