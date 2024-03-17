#!/usr/bin/env python3

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
from std_msgs.msg import Float32, Float64, Int8
from usv_interfaces.msg import Waypoint, ObjectList, Object



class XbeeStationNode(Node):
    pose = Pose2D()
    vel = Vector3()
    wp = Waypoint()
    heading_d = Float64()
    velocity_d = Float64()
    left_thruster = Float64()
    right_thruster = Float64()
    obj_list = ObjectList()
    m1_state = Float64()
    m2_state = Float64()
    m3_state = Float64()
    m4_state = Float64()

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
        self.m1_state.data = 0.0
        self.m2_state.data = 0.0
        self.m3_state.data = 0.0
        self.m4_state.data = 0.0

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
        self.obj_list_pub_ = self.create_publisher(ObjectList, '/usv_comms/obj_list', 10)
        self.m1_state_pub_ = self.create_publisher(Float64, '/usv_comms/m1_state', 10)
        self.m2_state_pub_ = self.create_publisher(Float64, '/usv_comms/m2_state', 10)
        self.m3_state_pub_ = self.create_publisher(Float64, '/usv_comms/m3_state', 10)
        self.m4_state_pub_ = self.create_publisher(Float64, '/usv_comms/m4_state', 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pose_pub_.publish(self.pose)
        self.vel_pub_.publish(self.vel)
        self.heading_d_pub_.publish(self.heading_d)
        self.velocity_d_pub_.publish(self.velocity_d)
        self.left_thruster_pub_.publish(self.left_thruster)
        self.right_thruster_pub_.publish(self.right_thruster)
        self.obj_list_pub_.publish(self.obj_list)
        self.m1_state_pub_.publish(self.m1_state)
        self.m2_state_pub_.publish(self.m2_state)
        self.m3_state_pub_.publish(self.m3_state)
        self.m4_state_pub_.publish(self.m4_state)
        

    def data_callback(self, xbee_message):
        data = struct.unpack('!'+'f'*46, xbee_message.data)
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
        self.m1_state.data = data[12]
        self.m2_state.data = data[13]
        self.m3_state.data = data[14]
        self.m4_state.data = data[15]
        base = 16
        self.obj_list = ObjectList()
        for i in range(5):
            obj_t = Object()
            obj_t.x = data[base + i*3]
            obj_t.y = data[base + i*3 + 1]
            obj_t.color = int(data[base + i*3 + 2])
            obj_t.type = "XBEE STD"
            self.obj_list.obj_list.append(obj_t)

def main(args=None):
    rclpy.init(args=args)
    xbee_station_node = XbeeStationNode()
    rclpy.spin(xbee_station_node)
    xbee_station_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()