#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: boat_transceiver.py
    @date: Tue Dec 17, 2019
    @date_modif: Sat Oct 4, 2020
    @author: Alejandro Gonzalez
    @e-mail: alexglzg97@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
	@co-author: Roberto Mendivil Castro
    @e-mail: robertomc97@gmail.com
    @co-author: Ian De La Garza González
    @e-mail: iandelagg@gmail.com
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

def boat_confirmation_callback(self, config):
    rospy.loginfo('[STATION] Sending following message: ' + str(config.data))
    

def boat_status_callback(self, status):
    boat_general_status = status.data

def main():
    rospy.loginfo(' +--------------------------------------+')
    rospy.loginfo(' |                Station               |')
    rospy.loginfo(' +--------------------------------------+\n')

    boat_commands = rospy.Publisher("/usv_comms/station_transceiver/station_commands", String, queue_size=10)
    rospy.Subscriber("/usv_comms/transceiver/boat_confirmation", String, boat_confirmation_callback)
    rospy.Subscriber("/usv_comms/transceiver/boat_status", String, boat_status_callback)
    rospy.loginfo('[STATION] ROS Node initialized.')

    rospy.init_node('station_transceiver', anonymous=False)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
       rate.sleep()


    rospy.spin()

    sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
