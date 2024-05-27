#!/usr/bin/env python3

import os
import csv
import math 
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class PathPublisherNode(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        parent_frame = 'world'

        self.path_pub_ = self.create_publisher(Path, "/usv/path_to_follow", 10)

        self.timer = self.create_timer(1, self.timer_callback)

        # self.waypoints_file_ = os.path.join(
        #     get_package_share_directory('usv_control'),
        #     'config',
        #     'example.csv'
        # )

        self.path_ = Path()
        self.path_.header.frame_id = parent_frame
        self.path_.header.stamp = self.get_clock().now().to_msg()

        for i in range(10):
            pose_stmpd = PoseStamped()
            pose_stmpd.header.frame_id = parent_frame
            pose_stmpd.pose.position.x = 10 * i * 0.6
            pose_stmpd.pose.position.y = 10 * math.sin(i)
            self.path_.poses.append(pose_stmpd)

    def timer_callback(self):
        self.path_pub_.publish(self.path_)

def main(args=None):
    rclpy.init(args=args)

    path_publisher_node = PathPublisherNode()
    rclpy.spin(path_publisher_node)
    path_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()