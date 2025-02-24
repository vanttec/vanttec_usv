#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64
import math

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')

        # Subscriptions
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.convert_joy, 10
        )
        self.key_sub = self.create_subscription(
            Twist, '/cmd_vel', self.convert_key, 10
        )
        self.pose_sub = self.create_subscription(
            Pose2D, '/usv/state/pose', self.pose_callback, 10
        )

        # Publishers for desired heading and surge velocity
        self.angle_pub = self.create_publisher(Float64, '/guidance/desired_heading', 10)
        self.vel_pub = self.create_publisher(Float64, '/guidance/desired_velocity', 10)

        self.angle_msg = Float64()
        self.vel_msg = Float64()

        # Desired heading and velocity variables
        self.curr_angle = 0.0
        self.curr_vel = 0.0

        # Current boat orientation (from /usv/state/pose)
        self.current_pose_theta = 0.0

        # Control parameters
        self.max_vel = 1.0
        self.throttle_increment = 0.1
        self.brake_increment = 0.1

    def pose_callback(self, msg):
        # Update the current pose's heading from the state
        self.current_pose_theta = msg.theta

    def convert_key(self, msg):
        # For key commands, use the angular.z as a small increment to the current pose.
        if abs(msg.angular.z) < 1e-3:
            # If no angular command, lock desired heading to current pose
            self.curr_angle = self.current_pose_theta
        else:
            # Add a small rotation relative to current pose
            self.curr_angle = self.current_pose_theta + msg.angular.z * 0.1
            self.curr_angle = (self.curr_angle + math.pi) % (2 * math.pi) - math.pi

        # Use the linear velocity from the key input directly.
        self.curr_vel = msg.linear.x

        self.vel_msg.data = self.curr_vel
        self.angle_msg.data = self.curr_angle

        self.vel_pub.publish(self.vel_msg)
        self.angle_pub.publish(self.angle_msg)

    def convert_joy(self, msg):
        # Define indices for the joystick axes
        joy_steer_i = 0
        
        # Indices for throttle and brake (adjust based on your controller)
        joy_throttle_i = 5  # Wired
        joy_brake_i = 2     # Wired
        # For wireless, you might use:
        # joy_throttle_i = 4
        # joy_brake_i = 5
        
        # Read steering command and invert if needed
        steer = -msg.axes[joy_steer_i]
        
        # If steering is near zero, set desired heading to the current pose
        if abs(steer) < 0.1:
            self.curr_angle = self.current_pose_theta
        else:
            # Compute a small offset relative to the current orientation
            self.curr_angle = self.current_pose_theta + steer * 0.1
            self.curr_angle = (self.curr_angle + math.pi) % (2 * math.pi) - math.pi

        # Scale throttle and brake to compute surge (velocity) command.
        throttle = (-msg.axes[joy_throttle_i] + 0.5) / 1.5  # Maps from [1, -1] to roughly [-0.33, 1]
        brake = (-msg.axes[joy_brake_i] + 1) / 2              # Maps from [1, -1] to [0, 1]

        self.curr_vel += (throttle * self.throttle_increment - brake * self.brake_increment)
        self.curr_vel = max(0.0, min(self.curr_vel, self.max_vel))

        self.angle_msg.data = self.curr_angle
        self.vel_msg.data = self.curr_vel

        self.angle_pub.publish(self.angle_msg)
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
