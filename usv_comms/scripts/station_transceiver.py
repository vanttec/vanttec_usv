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

from digi.xbee.devices import XBeeDevice
import rospy
from std_msgs.msg import String

# Replace with the serial port where your local module is connected to.
PORT = "/dev/xbee_station"
# Replace with the baud rate of your local module.
BAUD_RATE = 9600
# The XBee node you are trying to communicate with
REMOTE_NODE_ID = "BOAT_XBEE"
# Frequency in Hz in which the node will be run at.
ROS_RATE = 100


class XbeeStation:


    def __init__(self, port, baud_rate, remote_id, ros_rate):


        # Initialize and configure the DigiXTend Xbee Device
        self.device = XBeeDevice(port, baud_rate)
    
        self.device.open()    
        if not self.device.is_open():
            rospy.loginfo('[STATION] Device could not be opened.')
            raise Exception()

        self.device.flush_queues()
        self.xnetwork = self.device.get_network()
        self.remote_device = self.xnetwork.discover_device(remote_id)
        if self.remote_device is None:
            rospy.loginfo('[STATION] Could not find the remote device.')
            self.device.close()
            raise Exception()
            
        rospy.loginfo('[STATION] Digi XTend device initialized.')

        # ROS Configuration
        self.ros_rate = rospy.Rate(ros_rate)

        # ROS Publisher
        self.boat_data_pub = rospy.Publisher("/usv_comms/station_transceiver/boat_data", String, queue_size=10)

        # ROS Subscriber
        rospy.Subscriber("/usv_comms/station_transceiver/course_config", String, self.config_callback)
        rospy.Subscriber("/usv_comms/boat_transceiver/general_status", String, self.general_status_callback)
        rospy.loginfo('[STATION] ROS Node initialized.')

        self.comm_active = True
        rospy.loginfo('[STATION] Awaiting conversation...\n')

    def config_callback(self, config):
        rospy.loginfo('[STATION] Sending following message: ' + str(config.data))
        self.device.send_data_async(self.remote_device, str(config.data))
        if str(config.data) == 'exit':
            self.comm_active = False

    def general_status_callback(self, status):
        self.boat_general_status = status.data

def main():
    rospy.loginfo(' +--------------------------------------+')
    rospy.loginfo(' |                Station               |')
    rospy.loginfo(' +--------------------------------------+\n')

    rospy.init_node('station_transceiver', anonymous=False)
    
    try:
        station = XbeeStation(PORT, BAUD_RATE, REMOTE_NODE_ID, ROS_RATE)
    except:
        rospy.loginfo('[STATION] Digi XTend device could not be initialized.')
        sys.exit(1)

    try:
        while not rospy.is_shutdown() and station.comm_active:

            #Read data and chek if something has been received 
            xbee_message = station.device.read_data()

            if xbee_message is not None:
                #Decode and rospy.loginfo the message 
                message = xbee_message.data.decode()
                station.boat_data_pub.publish(str(message))

            station.ros_rate.sleep()

    finally:
        rospy.loginfo('[STATION] Terminating Session...')

        if station.device is not None and station.device.is_open():
            station.device.close()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
