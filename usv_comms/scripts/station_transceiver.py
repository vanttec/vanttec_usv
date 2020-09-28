#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: boat_transceiver.py
    @date: Tue Dec 17, 2019
    @date_modif: Sat Mar 21, 2020
    @author: Alejandro Gonzalez
    @e-mail: alexglzg97@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
	@co-author: Roberto Mendivil Castro
    @e-mail: robertomc97@gmail.com
	@brief: Script that handles communications with the boat in the station
    Open source
----------------------------------------------------------
'''

import argparse
import sys
import time

import rospy
from std_msgs.msg import String
boat_general_status=""

def config_callback(self, config):
    rospy.loginfo('[STATION] Sending following message: ' + str(config.data))

def general_status_callback(self, status):
    boat_general_status = status.data

def main():
    rospy.loginfo(' +--------------------------------------+')
    rospy.loginfo(' |                Station               |')
    rospy.loginfo(' +--------------------------------------+\n')

    boat_data_pub = rospy.Publisher("/usv_comms/station_transceiver/boat_data", String, queue_size=10)
    rospy.Subscriber("/usv_comms/station_transceiver/course_config", String, config_callback)
    rospy.Subscriber("/usv_comms/transceiver/general_status", String, general_status_callback)
    rospy.loginfo('[STATION] ROS Node initialized.')

    rospy.init_node('station_transceiver_v2', anonymous=False)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        boat_data_pub.publish = boat_general_status
    rospy.spin()

    sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
