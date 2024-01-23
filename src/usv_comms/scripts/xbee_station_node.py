#!/usr/bin/env python3.6

import os
import csv
import math 
import yaml
import struct
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice, XBeeNetwork

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float32, Float64
from usv_interfaces.msg import Waypoint



class XbeeStationNode(Node):
    pose = Pose2D()
    vel = Vector3()
    wp = Waypoint()
    heading_d = Float64()
    velocity_d = Float64()
    left_thruster = Float64()
    right_thruster = Float64()
    remote_id = "BOAT_XBEE"
    device = XBeeDevice("/dev/ttyUSB0", 115200)

    def __init__(self):
        super().__init__('xbee_station_node')
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        self.vel.x = 0.0
        self.vel.y = 0.0
        self.vel.z = 0.0
        self.wp.x = 0.0
        self.wp.y = 0.0
        self.heading_d.data = 0.0
        self.velocity_d.data = 0.0
        self.left_thruster.data = 0.0
        self.right_thruster.data = 0.0

        self.device.open()
        xbee_network = self.device.get_network()
        remote_device = xbee_network.discover_device(self.remote_id)
        self.device.add_data_received_callback(self.data_callback)
        self.device.set_sync_ops_timeout(100)

        if remote_device is None:
            print(" ...could not find", self.remote_id)
            exit(1)
        
        self.pose_pub_ = self.create_publisher(Pose2D, '/usv_comms/usv/pose', 10)
        self.vel_pub_ = self.create_publisher(Vector3, '/usv_comms/usv/velocity', 10)
        self.wp_pub_ = self.create_publisher(Waypoint, '/usv_comms/usv/waypoint', 10)
        self.heading_d_pub_ = self.create_publisher(Float64, '/usv_comms/usv/guidance/desired_heading', 10)
        self.velocity_d_pub_ = self.create_publisher(Float64, '/usv_comms/usv/guidance/desired_velocity', 10)
        self.left_thruster_pub_ = self.create_publisher(Float64, '/usv_comms/usv/left_thruster', 10)
        self.right_thruster_pub_ = self.create_publisher(Float64, '/usv_comms/usv/right_thruster', 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pose_pub_.publish(self.pose)
        self.vel_pub_.publish(self.vel)
        self.heading_d_pub_.publish(self.heading_d)
        self.velocity_d_pub_.publish(self.velocity_d)
        self.left_thruster_pub_.publish(self.left_thruster)
        self.right_thruster_pub_.publish(self.right_thruster)

    def data_callback(self, xbee_message):
        data = struct.unpack('!'+'f'*12, xbee_message.data)
        self.pose.x = data[0]
        self.pose.y = data[1]
        self.pose.theta = data[2]
        self.vel.x = data[3]
        self.vel.y = data[4]
        self.vel.z = data[5]
        self.wp.x = data[6]
        self.wp.y = data[7]
        self.heading_d.data = data[8]
        self.velocity_d.data = data[9]
        self.left_thruster.data = data[10]
        self.right_thruster.data = data[11]


def main(args=None):
    rclpy.init(args=args)
    xbee_station_node = XbeeStationNode()
    rclpy.spin(xbee_station_node)
    xbee_station_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
