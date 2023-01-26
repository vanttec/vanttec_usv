#!/usr/bin/env python3
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

#PORT = "/dev/xbee_station"
PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
STATION_TO_BOAT = "..."
REMOTE_NODE_ID = "BOAT_XBEE"

device = XBeeDevice(PORT, BAUD_RATE)

def topic_callback(data):
    global STATION_TO_BOAT
    STATION_TO_BOAT = data.data


def data_callback(xbee_message):
    print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(), xbee_message.data.decode()))

def main():
    
    try:
        global device
        global STATION_TO_BOAT
        device.open()
        
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

        device.add_data_received_callback(data_callback)

        print("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), STATION_TO_BOAT))
        LAST_SENT = ""
        while True:
            STATION_TO_BOAT = input('mensaje alv: ')
            if LAST_SENT != STATION_TO_BOAT:
                device.send_data(remote_device, STATION_TO_BOAT.encode('UTF-8'))
                LAST_SENT = STATION_TO_BOAT
            LAST_SENT = "owo"
        

    finally:
        if device is not None and device.is_open():
            device.close()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        device.close()
        print('Closed TCP connection')