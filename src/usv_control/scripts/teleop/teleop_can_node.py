#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64
from numpy import interp
import math

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')

        # Subscriptions
        self.key_sub = self.create_subscription(
            Twist, '/cmd_vel', self.convert_key, 10
        )

        # Publishers for left and right thruster PWM
        self.left_pwm_pub = self.create_publisher(Float64, '/usv/left_thruster', 10)
        self.right_pwm_pub = self.create_publisher(Float64, '/usv/right_thruster', 10)
        self.left_pwm_msg = Float64()
        self.right_pwm_msg = Float64()
        
        # PWM output limits (thruster signals)
        self.max_pwm = 10.0 # maximum reverse PWM

        # Gain for turning (mixing differential)
        self.turn_gain = 10.0        # adjust this gain so that full steer produces appropriate differential
        
    def convert_key(self, msg: Twist):
        surge_command = msg.linear.x * 2.0
        turn_command = msg.angular.z * 1.0

        left_pwm_norm = surge_command - turn_command
        right_pwm_norm = surge_command + turn_command

        # Normalize
        max_val = max(abs(left_pwm_norm), abs(right_pwm_norm))
        if abs(max_val) > 0.0:
            left_pwm_norm /= max_val
            right_pwm_norm /= max_val
        print("t: ",left_pwm_norm,right_pwm_norm)
        
        self.left_pwm_msg.data = interp(left_pwm_norm,[-1.0,1.0],[-self.max_pwm,self.max_pwm])
        self.right_pwm_msg.data = interp(right_pwm_norm,[-1.0,1.0],[-self.max_pwm,self.max_pwm])

        self.left_pwm_pub.publish(self.left_pwm_msg)
        self.right_pwm_pub.publish(self.right_pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
