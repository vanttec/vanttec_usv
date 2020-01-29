#!/usr/bin/env python

import os
import time
import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

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

        #self.lat = 0
        #self.lon = 0

        self.latref = 0
        self.lonref = 0
        #self.altref = 0

        #self.ecefxref = 0
        #self.ecefyref = 0
        #self.ecefzref = 0

        self.wp_array = []
        self.wp_t = []

        self.dmax = 10
        self.dmin = 2
        self.gamma = 0.003

        self.k = 1

        self.Waypointpath = Pose2D()
        self.LOSpath = Pose2D()

        self.obstacle_view = "000"

        self.waypoint_mode = 0 # 0 for NED, 1 for GPS, 2 for body

        #self.Rne = np.zeros((3, 3), dtype=np.float)
        #self.Rea = 6378137
        #self.e = 0.08181919
        #self.Pe_ref = np.zeros((3,1), dtype=np.float)

        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)
        #rospy.Subscriber("/vectornav/ins_2d/ins_pose", Pose2D, self.gps_callback)
        rospy.Subscriber("/vectornav/ins_2d/ins_ref", Vector3, self.gpsref_callback)
        #rospy.Subscriber("/vectornav/ins_2d/ecef_ref", Vector3, self.ecefref_callback)
        rospy.Subscriber("/mission/waypoints", Float32MultiArray, self.waypoints_callback)
        rospy.Subscriber("/usv_perception/lidar_detector/obstacles",  String, self.obstacles_callback)

        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/usv_control/los/target", Pose2D, queue_size=10)
        self.LOS_pub = rospy.Publisher("/usv_control/los/los", Pose2D, queue_size=10)

    def ned_callback(self, ned):
        self.NEDx = ned.x
        self.NEDy = ned.y
        self.yaw = ned.theta

    '''def gps_callback(self, gps):
        self.lat = gps.x
        self.lon = gps.y'''

    def gpsref_callback(self, gps):
        self.latref = gps.x
        self.lonref = gps.y
        #self.altref = gps.z

    '''def ecefref_callback(self, ecef):
        self.ecefxref = ecef.x
        self.ecefyref = ecef.y
        self.ecefzref = ecef.z'''

    def waypoints_callback(self, msg):
        wp = []
        leng = (msg.layout.data_offset)

        for i in range(int(leng)-1):
            wp.append(msg.data[i])
        self.waypoint_mode = msg.data[-1]
        self.wp_array = wp

    def obstacles_callback(self, data):
        self.obstacle_view = data.data

    def LOSloop(self, listvar):
        if self.k < len(listvar)/2:
            x1 = listvar[2*self.k - 2]
            y1 = listvar[2*self.k - 1]
            x2 = listvar[2*self.k]
            y2 = listvar[2*self.k + 1]
            self.Waypointpath.x = x2
            self.Waypointpath.y = y2
            self.target_pub.publish(self.Waypointpath)
            xpow = math.pow(x2 - self.NEDx, 2)
            ypow = math.pow(y2 - self.NEDy, 2)
            self.distance = math.pow(xpow + ypow, 0.5)
            if self.distance > 2:
                self.LOS(x1, y1, x2, y2)
            else:
                self.k += 1
        else:
            self.desired(0, self.yaw)

    def LOS(self, x1, y1, x2, y2):
        ak = math.atan2(y2-y1,x2-x1)
        ye = -(self.NEDx - x1)*math.sin(ak) + (self.NEDy - y1)*math.cos(ak)
        xe = (self.NEDx - x1)*math.cos(ak) + (self.NEDy - y1)*math.sin(ak)
        delta = (self.dmax - self.dmin)*math.exp(-(1/self.gamma)*abs(ye)) + self.dmin
        psi_r = math.atan(-ye/delta)
        self.bearing = ak + psi_r

        if (abs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing)-2*math.pi)

        xlos = x1 + (delta+xe)*math.cos(ak)
        ylos = y1 + (delta+xe)*math.sin(ak)
        self.LOSpath.x = xlos
        self.LOSpath.y = ylos
        self.LOS_pub.publish(self.LOSpath)

        self.vel = 1
        if self.distance < 6:
            self.vel = 0.6

        self.avoid(self.obstacle_view)

    def avoid(self, segment):

        #left, center, right
        cl = segment[0]
        cl = int(cl)
        if cl > 0:
            cl = -5
        left = cl

        cc = segment[1]
        cc = int(cc)
        if cc > 0:
            cc = 1
        center = cc

        cr = segment[2]
        cr = int(cr)
        if cr > 0:
            cr = 3
        right = cr

        addition = left + center + right 

        if center == 0:
            self.vel = self.vel

        elif addition == -1 or addition == 1:
                self.vel = -0.4

        elif addition != 0:
            self.bearing = self.yaw + addition * 0.17
            if (abs(self.bearing) > (math.pi)):
                self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing)-2*math.pi)

        self.desired(self.vel, self.bearing)

    '''
    def gps_to_ecef_to_ned(self, lat, lon):
        self.Rne = np.array([[-math.sin(self.latref) * math.cos(self.lonref), -math.sin(self.latref) * math.sin(self.lonref), math.cos(self.latref)],
                    [-math.sin(self.lonref), math.cos(self.lonref), 0],
                    [-math.cos(self.latref) * math.cos(self.lonref), -math.cos(self.latref) * math.sin(self.lonref), -math.sin(self.latref)]])
        self.Pe_ref = np.array([[self.ecefxref], [self.ecefyref], [self.ecefzref]])
        _ne = 1 - (self.e**2)*math.pow(math.sin(lat),2)
        Ne = self.Rea/(math.pow(_ne, 0.5))
        xe = (Ne + self.altref)*math.cos(lat)*math.cos(lon)
        ye = (Ne + self.altref)*math.cos(lat)*math.sin(lon)
        ze = (Ne*(1-self.e**2) + self.altref)*math.sin(lat)
        Pe = np.array([[xe],[ye],[ze]])
        Pn = np.matmul(self.Rne, Pe - self.Pe_ref)
        nedx = Pn[0]
        nedy = Pn[1]

        return (nedx,nedy)'''

    def gps_to_ned(self, lat2, lon2):
        lat1 = self.latref
        lon1 = self.lonref

        longitud_distance = (lon1 - lon2)
        y_distance = math.sin(longitud_distance) * math.cos(lat2)
        x_distance = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(longitud_distance)
        bearing = math.atan2(-y_distance, x_distance)
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)*math.sin(dphi/2) + math.cos(phi1)*math.cos(phi2)* math.sin(dlam/2)*math.sin(dlam/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6378137 * c

        nedx = distance*math.cos(bearing)
        nedy = distance*math.sin(bearing)

        return (nedx,nedy)

    def body_to_ned(self, x, y):
        p = np.array([x,y])
        J = np.array([[math.cos(self.yaw), -1*math.sin(self.yaw)],[math.sin(self.yaw), math.cos(self.yaw)]])
        n = J.dot(p)

        nedx = n[0] + self.NEDx
        nedy = n[1] + self.NEDy

        return (nedx, nedy)

    def desired(self, speed, heading):
        self.dh = heading
        self.ds = speed
        self.d_heading_pub.publish(self.dh)
        self.d_speed_pub.publish(self.ds)

def main():
    rospy.init_node('los_avoidance', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    t.wp_t = []
    wp_LOS = []
    while not rospy.is_shutdown() and t.testing:
        if t.wp_t != t.wp_array:
            t.k = 1
            t.wp_t = t.wp_array
            wp_LOS = t.wp_t
            x_0 = t.NEDx
            y_0 = t.NEDy
            if t.waypoint_mode == 0:
                wp_LOS.insert(0,x_0)
                wp_LOS.insert(1,y_0)
            elif t.waypoint_mode == 1:
                for i in range(0,len(wp_LOS),2):
                    wp_LOS[i], wp_LOS[i+1] = t.gps_to_ned(wp_LOS[i],wp_LOS[i+1])
                wp_LOS.insert(0,x_0)
                wp_LOS.insert(1,y_0)
            elif t.waypoint_mode == 2:
                for i in range(0,len(wp_LOS),2):
                    wp_LOS[i], wp_LOS[i+1] = t.body_to_ned(wp_LOS[i],wp_LOS[i+1])
                wp_LOS.insert(0,x_0)
                wp_LOS.insert(1,y_0)
        if len(wp_LOS) > 1:
            t.LOSloop(t.wp_t)
        rate.sleep()
    t.desired(0,t.yaw)
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
