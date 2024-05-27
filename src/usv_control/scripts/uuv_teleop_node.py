#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

import numpy as np
import math

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')
        self.jsub = self.create_subscription(
            Joy, '/joy', self.convert_joy, 10
        )

        self.vel_pub = self.create_publisher(Twist, '/uuv/cmd_twist', 10)
        self.vel_msg = Twist()

        self.current_vel = np.zeros(6)
        self.dt = 0.1
    
    def convert_joy(self, msg):
        roll_ax = 0
        pitch_ax = 1
        surge_neg_ax = 2
        sway_ax = 3
        heave_ax = 4
        surge_pos_ax = 5
        yaw_neg_btn = 4
        yaw_pos_btn = 5

        surge_p = (-msg.axes[surge_pos_ax] + 1 ) / 2
        surge_n = (msg.axes[surge_neg_ax] - 1 ) / 2
        surge_v = surge_p + surge_n
        sway_v = -msg.axes[sway_ax]
        heave_v = msg.axes[heave_ax]
        roll_v = -msg.axes[roll_ax]
        pitch_v = -msg.axes[pitch_ax]
        yaw_p = msg.buttons[yaw_pos_btn]
        yaw_n = -msg.buttons[yaw_neg_btn]
        yaw_v = yaw_p + yaw_n * 1.0

        new_cmd = np.array([surge_v, sway_v, heave_v, roll_v, pitch_v, yaw_v])

        for i in range(6):
            if new_cmd[i] == 0:
                if abs(self.current_vel[i]) < 0.1:
                    new_cmd[i] = -self.current_vel[i]
                else:
                    new_cmd[i] = -math.copysign(1.0, self.current_vel[i])

        self.current_vel += self.dt * new_cmd

        self.current_vel = [max(-1.0, min(1.0, i)) for i in self.current_vel]

        self.vel_msg.linear= Vector3(x = self.current_vel[0], y = self.current_vel[1], z = self.current_vel[2])
        self.vel_msg.angular= Vector3(x = self.current_vel[3], y = self.current_vel[4], z = self.current_vel[5])
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    rmn = TeleopControl()
    rclpy.spin(rmn)
    rmn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()