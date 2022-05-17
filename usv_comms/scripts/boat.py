#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

# TODO: Replace with the serial port where your local module is connected to.
PORT = "/dev/xbee_boat"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 9600

BOAT_TO_STATION = "..."

# RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20041CF8F96")))

REMOTE_NODE_ID = "STATION_XBEE"

def topic_callback(data):
    global BOAT_TO_STATION
    BOAT_TO_STATION = data.data

def data_callback(xbee_message):
            rospy.loginfo("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     xbee_message.data.decode()))
            publisher.pub(xbee_message.data.decode())
    
def boat():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('boat', anonymous=True)
    rospy.Subscriber("BOAT_TO_STATION", String, topic_callback)
    publisher = rospy.Publisher('received_in_boat',String, queue_size = 10)

    try:
        device = XBeeDevice(PORT, BAUD_RATE)
        device.open()

        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

        if remote_device is None:
            rospy.loginfo("Could not find the remote device")
            exit(1)

        device.add_data_received_callback(data_callback)

        rospy.loginfo("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), BOAT_TO_STATION))
        LAST_SENT = ""
        while not rospy.is_shutdown():
            if LAST_SENT != BOAT_TO_STATION:
                device.send_data(remote_device, BOAT_TO_STATION)
                LAST_SENT = BOAT_TO_STATION

    finally:
        if device is not None and device.is_open():
            device.close()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    try:
        boat()
    except rospy.ROSInterruptException:
        pass
