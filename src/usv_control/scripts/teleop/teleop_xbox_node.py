#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, UInt16
import math

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')

        # Subscriptions
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.convert_joy, 10
        )

        # Publishers for left and right thruster PWM
        self.left_pwm_pub = self.create_publisher(Float64, '/usv/left_thruster', 10)
        self.right_pwm_pub = self.create_publisher(Float64, '/usv/right_thruster', 10)
        self.op_mode_pub = self.create_publisher(UInt16, '/usv/op_mode', 10)
        self.left_pwm_msg = Float64()
        self.right_pwm_msg = Float64()
        self.op_mode_msg = UInt16()
        
        # PWM output limits (thruster pair signals)
        self.max_pwm = 73.0 # maximum reverse PWM
        self.last_btn = 0

        self.timer = self.create_timer(0.01,
                                       self.timer_callback)
    
    def timer_callback(self):
        # op_mode:0 = autonomous
        # op_mode:1 = teleoperated
        if self.op_mode_msg.data:
            self.left_pwm_pub.publish(self.left_pwm_msg)
            self.right_pwm_pub.publish(self.right_pwm_msg)
        self.op_mode_pub.publish(self.op_mode_msg)
        
    def convert_joy(self, msg: Joy):
        # msg.axes[0] goes from -1 to 1, which should be inverted.
        # However, gz sim and rviz both use ENU instead of NED.
        steer = msg.axes[0]
        throttle = msg.axes[1]

        if math.fabs(steer) < 0.15:
            steer = 0.
        if math.fabs(throttle) < 0.15:
            throttle = 0.

        left = (throttle + steer)*self.max_pwm
        right = (throttle - steer)*self.max_pwm

        self.left_pwm_msg.data = clamp(left, -60.0, 73.0)
        self.right_pwm_msg.data = clamp(right, -60.0, 73.0)

        btn = msg.buttons[7]
        if (btn != self.last_btn) and (btn):
            self.op_mode_msg.data = not self.op_mode_msg.data
        self.last_btn = btn

def main(args=None):
    rclpy.init(args=args)
    node = TeleopControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
