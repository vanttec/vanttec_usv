#!/usr/bin/env python

import argparse
import rospy
import time
import sys
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice

#parser = argparse.ArgumentParser()
#parser.add_argument('dev')
#args = parser.parse_args()

#****************************************************************************************#
# Replace with the serial port where your local module is connected to.
PORT = rospy.get_param("station_transceiver/xbee_port")
# Replace with the baud rate of your local module.
BAUD_RATE = 9600
# El nodo XBee con el que se quiere comunicar.
REMOTE_NODE_ID = "vtecboat"
# Frecuencia en Hz a la que se va a correr el nodo.
ROS_RATE = 100
#****************************************************************************************#

class XbeeStation:
    def __init__(self, _port, _baud_rate, _remote_id, _ros_rate):
        
        # Initialize and configure the DigiXTend Xbee Device
        self.device = XBeeDevice(_port, _baud_rate)
    
        self.device.open()    
        if not self.device.is_open():
            rospy.loginfo('[STATION] Device could not be opened.')
            raise Exception()

        self.device.flush_queues()
        self.xnetwork = self.device.get_network()
        self.remote_device = self.xnetwork.discover_device(_remote_id)
        if self.remote_device is None:
            rospy.loginfo('[STATION] Could not find the remote device.')
            self.device.close()
            raise Exception()
            
        rospy.loginfo('[STATION] Digi XTend device initialized.')

        # ROS Configuration
        self.ros_rate = rospy.Rate(_ros_rate)

        # ROS Publisher
        self.boat_data_pub = rospy.Publisher('/usv_comms/station_transceiver/boat_data', String, queue_size=10)

        # ROS Subscriber
        rospy.Subscriber('/usv_comms/station_transceiver/course_config', String, self.config_callback)
        rospy.Subscriber("/usv_comms/boat_transceiver/general_status", String, self.general_status_callback)
        rospy.loginfo('[STATION] ROS Node initialized.')

        self.comm_active = True
        rospy.loginfo('[STATION] Awaiting conversation...\n')

    def config_callback(self, _config):
        rospy.loginfo('[STATION] Sending following message: ' + str(_config.data))
        self.device.send_data_async(self.remote_device, str(_config.data))
        if str(_config.data) == 'exit':
            self.comm_active = False

    def general_status_callback(self, status):
        self.boat_general_status = status.data

def main():
    rospy.loginfo(" +--------------------------------------+")
    rospy.loginfo(" |                Station               |")
    rospy.loginfo(" +--------------------------------------+\n")

    rospy.init_node('station_transceiver', anonymous=True)
    
    try:
        station = XbeeStation(PORT, BAUD_RATE, REMOTE_NODE_ID, ROS_RATE)
    except:
        rospy.loginfo('[STATION] Digi XTend device could not be initialized.')
        sys.exit(1)

    try:
        while not rospy.is_shutdown() and station.comm_active:
            
            xbee_message = station.device.read_data()
            if xbee_message is not None:
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