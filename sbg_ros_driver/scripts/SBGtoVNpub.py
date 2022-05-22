#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float32MultiArray, Float64
from sbg_driver.msg import SbgGpsPos, SbgImuData


class SBG_to_VN_node:
    def __init__(self):

        #Initial declarations of variables and objects that'll be used in the node.
        rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, self.callback_sbg_gps_pos)
        #rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.callback_yolo_det)
        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.callback_sbg_imu_data)

        self.SBGGpsData = SbgGpsPos()
        self.SBGImuData = SbgImuData()
        self.NEDPToPublish = Pose2D()
        self.INSPToPublish = Pose2D()
        self.INSRToPublish = Pose2D()
        self.NED_pose_node_pub = rospy.Publisher('/vectornav/ins_2d/NED_pose', Pose2D, queue_size=10)
        self.ins_pose_node_pub = rospy.Publisher('/vectornav/ins_2d/ins_pose', Pose2D, queue_size=10)
        self.ins_ref_node_pub = rospy.Publisher('/vectornav/ins_2d/ins_ref', Pose2D, queue_size=10)
        
    def callback_sbg_gps_pos(self, data):
        self.SBGGpsData = data
        self.NEDPToPublish.x = self.SBGGpsData.position_accuracy.x
        self.NEDPToPublish.y = self.SBGGpsData.position_accuracy.y
        self.INSPToPublish.x = self.SBGGpsData.latitude
        self.INSPToPublish.y = self.SBGGpsData.longitude

    def callback_sbg_imu_data(self, data):
        self.SBGImuData = data
        



def main():
                #First coordinate = Latitude, second = longitude
    ConverterNode = SBG_to_VN_node()
    rospy.init_node("SBG_converter_node", anonymous=False)
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        ConverterNode.NED_pose_node_pub.publish(ConverterNode.NEDPToPublish)
        ConverterNode.ins_pose_node_pub.publish(ConverterNode.INSPToPublish)
        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass