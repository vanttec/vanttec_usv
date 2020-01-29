#!/usr/bin/env python

import argparse
import sys
import time

from digi.xbee.devices import XBeeDevice
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String

#parser = argparse.ArgumentParser()
#parser.add_argument('dev')
#args = parser.parse_args()

#****************************************************************************************#
# Replace with the serial port where your local module is connected to.
PORT = rospy.get_param("boat_transceiver/xbee_port")
# Replace with the baud rate of your local module.
BAUD_RATE = 9600
# El nodo XBee con el que se quiere comunicar.
REMOTE_NODE_ID = "vtecstation"
# Frecuencia en Hz a la que se va a correr el nodo.
ROS_RATE = 100
#****************************************************************************************#

class XbeeBoat:
    def __init__(self, _port, _baud_rate, _remote_id, _ros_rate):
        
        self.usv_master_status = ""

        # Initialize and configure the DigiXTend Xbee Device
        self.device = XBeeDevice(_port, _baud_rate)

        self.device.open()    
        if not self.device.is_open():
            rospy.loginfo('[USV] Device could not be opened.')
            raise Exception()
        
        self.device.flush_queues()
        self.xnetwork = self.device.get_network()
        self.remote_device = self.xnetwork.discover_device(_remote_id)
        if self.remote_device is None:
            rospy.loginfo('[USV] Could not find the remote device.')
            self.device.close()
            raise Exception()
            
        rospy.loginfo('[USV] Digi XTend device initialized.')

        # ROS Configuration
        self.ros_rate = rospy.Rate(_ros_rate)

        # ROS Subscriber
        rospy.Subscriber("/usv_comms/boat_transceiver/data_input", String, self.data_callback)
        rospy.Subscriber("/usv_master/usv_master_status", String, self.usv_master_callback)
        

        # ROS Publisher
        self.stop_pub = rospy.Publisher("/usv_comms/boat_transceiver/stop_mission", Empty, queue_size=10)
        self.course_pub = rospy.Publisher("/usv_comms/boat_transceiver/course_config", String, queue_size=10)
        self.start_pub = rospy.Publisher("/usv_comms/boat_transceiver/start_mission", Empty, queue_size=10)
        self.general_status_pub = rospy.Publisher("/usv_comms/boat_transceiver/general_status", String, queue_size=10)

        self.empty_msg = Empty()
        self.boat_data = String()

        self.comm_active = True
        rospy.loginfo('[USV] Awaiting conversation...\n')

    def data_callback(self, _data):
        self.boat_data = _data.data
        #rospy.loginfo('[USV] Sending data: ', self.boat_data)
        #self.device.send_data_async(self.remote_device, self.boat_data)

    def usv_master_callback(self, status):
        self.usv_master_status = status.data

def main():
    rospy.loginfo(" +-------------------------------------------------+")
    rospy.loginfo(" |                       Boat                      |")
    rospy.loginfo(" +-------------------------------------------------+\n")
    
    rospy.init_node('boat_transceiver', anonymous=True)

    try:
        usv = XbeeBoat(PORT, BAUD_RATE, REMOTE_NODE_ID, ROS_RATE)
    except:
        rospy.loginfo('[USV] Digi XTend device could not be initialized.')
        sys.exit(1)

    try:
        while not rospy.is_shutdown() and usv.comm_active:
            #Read data and chek if something has been received 
            xbee_message = usv.device.read_data()

            if xbee_message is not None:
                #Decode and rospy.loginfo the message 
                message = xbee_message.data.decode()

                if message == 's' or message == 'S':
                    usv.start_pub.publish(usv.empty_msg)
                    usv.device.send_data_async(usv.remote_device, 'Starting mission...')
                    message='START '
                elif message == 'k' or message == 'K':
                    usv.stop_pub.publish(usv.empty_msg)
                    usv.device.send_data_async(usv.remote_device, 'Stopping mission...')
                    message='STOP '
                
                usv.course_pub.publish(message)
                
                '''
                if message[0] == 'c':
                    usv.course_pub.publish(message)
                    usv.device.send_data_async(usv.remote_device, 'Changing to course ' + message[1])
                elif len(message) >= 1:
                    usv.course_pub.publish(message)
                    usv.device.send_data_async(usv.remote_device, 'Changing')
                '''

                #usv.device.send_data_async(usv.remote_device, usv.usv_master_status)
                            
                
            usv.ros_rate.sleep()
    #If the device is not closed, close it.
    finally:
        rospy.loginfo('[USV] Terminating Session...')
        if usv.device is not None and usv.device.is_open():
            usv.device.close()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
