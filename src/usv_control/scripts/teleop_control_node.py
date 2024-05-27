#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')
        self.jsub = self.create_subscription(
            Joy, '/joy', self.convert_joy, 10
        )

        self.key_sub = self.create_subscription(
            Twist, '/cmd_vel', self.convert_key, 10
        )

        self.angular_vel_pub = self.create_publisher(Float64, '/guidance/desired_angular_velocity', 10)
        self.angular_vel_msg = Float64()

        self.vel_pub = self.create_publisher(Float64, '/guidance/desired_velocity', 10)
        self.vel_msg = Float64()

        self.max_angle_vel = 2.0
        self.curr_angle_vel = 0.0

        self.max_vel = 1.0
        self.curr_vel = 0.0
        self.throttle_increment = 0.1

        self.brake_increment = 0.1

    def convert_key(self, msg):
        self.vel_msg.data = msg.linear.x
        self.angular_vel_msg.data = msg.angular.z
        self.angular_vel_pub.publish(self.angular_vel_msg)
        self.vel_pub.publish(self.vel_msg)

    def convert_joy(self, msg):
        joy_steer_i = 0
        joy_throttle_i = 5
        joy_brake_i = 2

        steer = -msg.axes[joy_steer_i]
        self.curr_angle_vel = steer * self.max_angle_vel

        throttle = (-msg.axes[joy_throttle_i] + 0.5) / 1.5 # From [1,-1] to [-0.333, 1]

        brake = (-msg.axes[joy_brake_i] + 1) / 2 # From [1,-1] to [0, 1]

        self.curr_vel += (throttle * self.throttle_increment - brake * self.brake_increment)

        self.curr_vel = max( 0.0, min( self.curr_vel, self.max_vel ))

        if(abs(self.curr_angle_vel) < 0.2):
            self.curr_angle_vel = 0.0

        self.angular_vel_msg.data = self.curr_angle_vel
        self.vel_msg.data = self.curr_vel

        self.angular_vel_pub.publish(self.angular_vel_msg)
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    rmn = TeleopControl()
    rclpy.spin(rmn)
    rmn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()