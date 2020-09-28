#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import sys
import time

import rospy
from std_msgs.msg import Empty, String

message=""
usv_master_status=""

# ROS Publisher
stop_pub = rospy.Publisher("/usv_comms/boat_transceiver/stop_mission", Empty, queue_size=10)
course_pub = rospy.Publisher("/usv_comms/boat_transceiver/course_config", String, queue_size=10)
start_pub = rospy.Publisher("/usv_comms/boat_transceiver/start_mission", Empty, queue_size=10)
general_status_pub = rospy.Publisher("/usv_comms/boat_transceiver/general_status", String, queue_size=10)

def data_callback(data):
    message = data.data
    rospy.loginfo(message)
            
    if message == 's' or message == 'S' or message == "s" or message == "S":
        rospy.loginfo('Starting mission...')
        #message = 'START '
        publishing = 'START'
    elif message == 'k' or message == 'K' or message == "k" or message == "K":

        rospy.loginfo('Stopping mission...')
        #message = 'STOP'
        publishing = 'STOP'
    course_pub.publish(publishing)

def usv_master_callback(status):
    usv_master_status = status.data


def main():
    rospy.loginfo(" +-------------------------------------------------+")
    rospy.loginfo(" |                       Boat                      |")
    rospy.loginfo(" +-------------------------------------------------+\n")
    """
    # ROS Publisher
    stop_pub = rospy.Publisher("/usv_comms/boat_transceiver/stop_mission", Empty, queue_size=10)
    course_pub = rospy.Publisher("/usv_comms/boat_transceiver/course_config", String, queue_size=10)
    start_pub = rospy.Publisher("/usv_comms/boat_transceiver/start_mission", Empty, queue_size=10)
    general_status_pub = rospy.Publisher("/usv_comms/boat_transceiver/general_status", String, queue_size=10)
    """
    
    # ROS Subscriber
    rospy.Subscriber("/usv_comms/transceiver/data_input", String, data_callback)
    rospy.Subscriber("/usv_master/usv_master_status", String, usv_master_callback)
    
    rospy.init_node('boat_transceiver_v2', anonymous=False)
    
    # ROS Configuration
    ros_rate = rospy.Rate(100)

    empty_msg = Empty()
    boat_data = String()

    comm_active = True
    publishing = ""
    while not rospy.is_shutdown():
        """
        if message == 's' or message == 'S' or message == "s" or message == "S":
            start_pub.publish(empty_msg)
            rospy.loginfo('Starting mission...')
            #message = 'START '
            publishing = 'START'
        elif message == 'k' or message == 'K' or message == "k" or message == "K":
            stop_pub.publish(empty_msg)
            rospy.loginfo('Stopping mission...')
            #message = 'STOP'
            publishing = 'STOP'
        
        course_pub.publish(publishing)
        
        """
        ros_rate.sleep()

    rospy.spin()
    sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
