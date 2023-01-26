#!/usr/bin/env python3
import string
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

import socket

HOST = '127.0.0.1'    # The remote host
PORT = 50000              # The same port as used by the server

#XBEE_PORT = "/dev/xbee_boat"
XBEE_PORT = "/dev/ttyUSB1"
BAUD_RATE = 9600
BOAT_TO_STATION = "..."
REMOTE_NODE_ID = "STATION_XBEE"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
device = XBeeDevice(XBEE_PORT, BAUD_RATE)


def topic_callback(data):
    global BOAT_TO_STATION
    BOAT_TO_STATION = data.data

def data_callback(xbee_message):
    print('a')
    s.send(xbee_message.data)
    print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                xbee_message.data.decode()))
    #publisher = rospy.Publisher('received_in_boat',String, queue_size = 10)
    #publisher.pub(xbee_message.data.decode())
    
def main():
    s.connect((HOST, PORT))

    try:
        device.open()

        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

        device.add_data_received_callback(data_callback)

        print("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), BOAT_TO_STATION))
        LAST_SENT = ""
        while True:
            if LAST_SENT != BOAT_TO_STATION:
                device.send_data(remote_device, BOAT_TO_STATION)
                LAST_SENT = BOAT_TO_STATION

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        s.close()
        device.close()
        print('Closed TCP connection')
