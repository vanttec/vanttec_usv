#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: boat_transceiver.py
    @date: Tue Dec 17, 2019
    @date_modif: Sat Mar 21, 2020
    @author: Alejandro Gonzalez
    @e-mail: alexglzg97@gmail.com
	@co-author: Roberto Mendivil Castro
    @e-mail: robertomc97@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
    @co-author: Ian De La Garza González
    @e-mail: iandelagg@gmail.com
	@brief: Script that handles communications with the station in the boat
    Open source
----------------------------------------------------------
'''

import argparse
import sys
import time

import rospy
from std_msgs.msg import Empty, String


# ROS Publisher
stop_pub = rospy.Publisher("/usv_comms/boat_transceiver/stop_mission", Empty, queue_size=10)
course_confirmation = rospy.Publisher("/usv_comms/boat_transceiver/course_confirmation", String, queue_size=10)
start_pub = rospy.Publisher("/usv_comms/boat_transceiver/start_mission", Empty, queue_size=10)
boat_status_pub = rospy.Publisher("/usv_comms/boat_transceiver/boat_status", String, queue_size=10)

def data_callback(data):
    message = data.data
    rospy.loginfo(message)
            
    if message == 's' or message == 'S' or message == "s" or message == "S":
        rospy.loginfo('Starting mission...')
        #message = 'START '
        publishing = 'START'
        stop_pub.publish(Empty())
    elif message == 'k' or message == 'K' or message == "k" or message == "K":

        rospy.loginfo('Stopping mission...')
        #message = 'STOP'
        publishing = 'STOP'
        start_pub.publish(Empty())

    course_confirmation.publish(publishing)

def usv_master_callback(status):
    usv_master_status = status.data
    boat_status_pub.publish(usv_master_status)


def main():
    rospy.loginfo(" +-------------------------------------------------+")
    rospy.loginfo(" |                       Boat                      |")
    rospy.loginfo(" +-------------------------------------------------+\n")
    
    # ROS Subscriber
    rospy.Subscriber("/usv_comms/transceiver/station_commands", String, data_callback)
    rospy.Subscriber("/usv_master/usv_master_status", String, usv_master_callback)
    
    rospy.init_node('boat_transceiver', anonymous=False)
    
    # ROS Configuration
    ros_rate = rospy.Rate(100)

    empty_msg = Empty()
    boat_data = String()

    comm_active = True
    publishing = ""
    while not rospy.is_shutdown():
        ros_rate.sleep()

    rospy.spin()
    sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
