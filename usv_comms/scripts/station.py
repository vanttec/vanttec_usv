#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

PORT = "/dev/ttyUSB1"
BAUD_RATE = 9600
STATION_TO_BOAT = "..."
REMOTE_NODE_ID = "BOAT_XBEE"

def topic_callback(data):
    global STATION_TO_BOAT
    STATION_TO_BOAT = data.data

def data_callback(xbee_message):
            rospy.loginfo("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     xbee_message.data.decode()))
            publisher.pub(xbee_message.data.decode())

def station():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('station', anonymous=True)
    rospy.Subscriber("STATION_TO_BOAT", String, topic_callback)
    publisher = rospy.Publisher('received_in_station',String, queue_size = 10)
    
    try:
        device = XBeeDevice(PORT, BAUD_RATE)
        device.open()
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
        if remote_device is None:
            rospy.loginfo("Could not find the remote device")
            exit(1)

        device.add_data_received_callback(data_callback)

        rospy.loginfo("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), STATION_TO_BOAT))
        LAST_SENT = ""
        while not rospy.is_shutdown():
            if LAST_SENT != STATION_TO_BOAT:
                device.send_data(remote_device, STATION_TO_BOAT)
                LAST_SENT = STATION_TO_BOAT
        

    finally:
        if device is not None and device.is_open():
            device.close()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    station()