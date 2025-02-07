#!/usr/bin/env python3

from digi.xbee.devices import XBeeDevice
import time

def test_xbee(port="/dev/ttyUSB0", baud_rate=115200):
    print(f"Testing XBee on {port} at {baud_rate} baud...")
    
    try:
        device = XBeeDevice(port, baud_rate)
        device.open()
        
        # Get and print device information
        print("Device opened successfully!")
        print(f"Node ID: {device.get_node_id()}")
        print(f"64-bit address: {device.get_64bit_addr()}")
        print(f"16-bit address: {device.get_16bit_addr()}")
        
        # Test network discovery
        xbee_network = device.get_network()
        print("\nDiscovering network nodes...")
        xbee_network.start_discovery_process()
        while xbee_network.is_discovery_running():
            time.sleep(0.5)
        
        devices = xbee_network.get_devices()
        if devices:
            print(f"\nFound {len(devices)} device(s):")
            for remote in devices:
                print(f"- Node ID: {remote.get_node_id()}")
                print(f"  Address: {remote.get_64bit_addr()}")
        else:
            print("\nNo other devices found in network")
        
        device.close()
        print("\nTest completed successfully!")
        
    except Exception as e:
        print(f"Error: {str(e)}")
        print("\nTroubleshooting steps:")
        print("1. Check if the port is correct")
        print("2. Verify the XBee is properly connected")
        print("3. Check if you have permission to access the port:")
        print("   sudo chmod 666 " + port)
        print("4. Try a different baud rate")
        print("5. Make sure the XBee is properly powered")
        print("\nTo check available ports:")
        print("   ls /dev/ttyUSB*")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Test XBee communication')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    test_xbee(args.port, args.baud)
