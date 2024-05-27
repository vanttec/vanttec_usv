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
from usv_interfaces.msg import Waypoint, WaypointList

class PathPublisherNode(Node):

    def __init__(self):
        super().__init__('waypoint_handler_node')

        self.parent_frame = 'world'

        self.path_pub_ = self.create_publisher(Path, "/usv/path_to_follow", 10)

        self.goals_sub_ = self.create_subscription(
            WaypointList, '/usv/goals', self.wp_handle, 10
        )

        self.timer = self.create_timer(1, self.timer_callback)

        self.path_ = Path()
        self.path_.header.frame_id = self.parent_frame
        self.path_.header.stamp = self.get_clock().now().to_msg()

    def timer_callback(self):
        self.path_pub_.publish(self.path_)

    def dist(self, arr, i, j):
        return ((arr[i][0] - arr[j][0])**2 + (arr[i][1] - arr[j][1])**2) ** (1/2)

    def catmulify(self, wp_list):
        new_list = WaypointList()

        for i in range(1, len(wp_list) - 2):
            p = []
            t = []

            for j in range(4):
                idx = i - 1 + j
                arr = np.array([wp_list[idx].x, wp_list[idx].y])
                p.append(arr)
            
            for k in range(3):
                t.append(self.dist(p, k, k+1) ** (1/2))

            m1 = p[2] - p[1] + t[1] * ((p[1] - p[0]) / t[0] - (p[2] - p[0]) / (t[0] + t[1]))
            m2 = p[2] - p[1] + t[1] * ((p[3] - p[2]) / t[2] - (p[3] - p[1]) / (t[1] + t[2]))

            a = 2 * (p[1] - p[2]) + m1 + m2
            b = -3 * (p[1] - p[2]) - 2 * m1 - m2
            c = m1
            d = p[1]

            n = 50
            for l in range(n):
                tmp = l / n
                new_wp_arr = a * (tmp ** 3) + b * (tmp ** 2) + c * tmp + d
                wp_tmp = Waypoint()
                wp_tmp.x = new_wp_arr[0]
                wp_tmp.y = new_wp_arr[1]
                new_list.waypoint_list.append(wp_tmp)

        return new_list    
    
    def wp_handle(self, msg):
        self.path_.poses.clear()

        wp_list_inter = self.catmulify(msg.waypoint_list)

        for i in wp_list_inter.waypoint_list:
            self.pose_stmpd = PoseStamped()
            self.pose_stmpd.header.frame_id = self.parent_frame
            self.pose_stmpd.pose.position.x = i.x
            self.pose_stmpd.pose.position.y = i.y

            self.path_.poses.append(self.pose_stmpd)


def main(args=None):
    rclpy.init(args=args)

    waypoint_handler_node = PathPublisherNode()
    rclpy.spin(waypoint_handler_node)
    waypoint_handler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()