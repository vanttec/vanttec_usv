#!/usr/bin/env python3

import os
import csv
import math 
import struct
import yaml
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float32, Float64
from usv_interfaces.msg import Waypoint, ObjectList, Object

#!/usr/bin/env python3.6

import os
import csv
import math 
import struct
import yaml
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float32, Float64
from usv_interfaces.msg import Waypoint



class XbeeBoatNode(Node):
    pose_list = [0.0]*3
    vel_list = [0.0]*3
    wp_list = [0.0]*2
    heading_d = 0.0
    velocity_d = 0.0
    left_thruster = 0.0
    right_thruster = 0.0
    obj_list = [0.0]*3*5

    remote_id = "STATION_XBEE"
    device = XBeeDevice("/dev/ttyUSB1", 115200)
    
    def __init__(self):
        super().__init__('xbee_boat_node')

        self.device.open()
        xbee_network = self.device.get_network()
        self.remote_device = xbee_network.discover_device(self.remote_id)
        self.device.add_data_received_callback(XbeeBoatNode.data_callback)
        self.device.set_sync_ops_timeout(100)

        if self.remote_device is None:
            print(" ...could not find", self.remote_id)
            exit(1)
        
        self.pose_sub_ = self.create_subscription(
            Pose2D,
            '/usv/state/pose',
            self.pose_callback,
            10)
        self.obj_list_sub_ = self.create_subscription(
            ObjectList,
            '/objects',
            self.obj_list_callback,
            10)
        self.velocity_sub_ = self.create_subscription(
            Vector3,
            '/usv/state/velocity',
            self.velocity_callback,
            10)
        self.wp_sub_ = self.create_subscription(
            Waypoint,
            '/usv/waypoint',
            self.wp_callback,
            10)
        self.heading_d_sub_ = self.create_subscription(
            Float64,
            '/guidance/desired_heading',
            self.heading_d_callback,
            10)
        self.velocity_d_sub_ = self.create_subscription(
            Float64,
            '/guidance_desired_velocity',
            self.velocity_d_callback,
            10)
        self.left_thruster_sub_ = self.create_subscription(
            Float64,
            '/usv/left_thruster',
            self.left_thruster_callback,
            10)
        self.right_thruster_sub_ = self.create_subscription(
            Float64,
            '/usv/right_thruster',
            self.right_thruster_callback,
            10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pub_list = self.pose_list + self.vel_list + self.wp_list + \
        [self.heading_d, self.velocity_d, self.left_thruster, \
        self.right_thruster] + self.obj_list

        pub_list = pub_list

        pub_struct = struct.pack('!'+'f'*27, *pub_list)
        if(len(pub_list) > 0):
            st = ','.join(str(x) for x in pub_list)
            # print(pub_struct)
            self.device.send_data(self.remote_device, pub_struct)
        else:
            print("Waiting for topics...")

    def pose_callback(self, msg):
        self.pose_list = [msg.x, msg.y, msg.theta]
        self.pose_list = [round(x,1) for x in self.pose_list]

    def obj_list_callback(self, msg):
        for i in range(5):
            self.obj_list[i * 3] = msg.obj_list[i].x
            self.obj_list[i * 3 + 1] = msg.obj_list[i].y
            self.obj_list[i * 3 + 2] = msg.obj_list[i].color

    def velocity_callback(self, msg):
        self.vel_list = [msg.x, msg.y, msg.z]
        self.vel_list = [round(x,1) for x in self.vel_list]

    def wp_callback(self, msg):
        self.wp_list = [msg.x, msg.y]
        self.wp_list = [round(x,1) for x in self.wp_list]

    def heading_d_callback(self, msg):
        self.heading_d = round(msg.data,1)

    def velocity_d_callback(self, msg):
        self.velocity_d = round(msg.data,1)

    def left_thruster_callback(self, msg):
        self.left_thruster = round(msg.data,1)

    def right_thruster_callback(self, msg):
        self.right_thruster = round(msg.data,1)

    def data_callback(self, xbee_message):
        pass

def main(args=None):
    rclpy.init(args=args)
    xbee_boat_node = XbeeBoatNode()
    rclpy.spin(xbee_boat_node)
    xbee_boat_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
