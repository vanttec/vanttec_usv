#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64
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
        self.key_sub = self.create_subscription(
            Twist, '/cmd_vel', self.convert_key, 10
        )
        self.pose_sub = self.create_subscription(
            Pose2D, '/usv/state/pose', self.pose_callback, 10
        )

        # Publishers for left and right thruster PWM
        self.left_pwm_pub = self.create_publisher(Float64, '/usv/left_thruster', 10)
        self.right_pwm_pub = self.create_publisher(Float64, '/usv/right_thruster', 10)
        self.left_pwm_msg = Float64()
        self.right_pwm_msg = Float64()

        # Internal state variables
        self.current_pose_theta = 0.0  # current orientation (radians) from /usv/state/pose
        self.curr_vel = 0.0          # surge command (normalized 0 to max_vel; joystick mode does not allow reverse)
        
        # Parameters for surge/brake integration (joystick)
        self.max_vel = 1.0           # normalized maximum surge speed
        self.throttle_increment = 0.1
        self.brake_increment = 0.1

        # PWM output limits (thruster signals)
        self.max_forward_pwm = 36.5  # maximum forward PWM
        self.max_reverse_pwm = -30.0 # maximum reverse PWM

        # Gain for turning (mixing differential)
        self.turn_gain = 10.0        # adjust this gain so that full steer produces appropriate differential
        
        # Deadzone settings
        self.steer_deadzone = 0.1    # for joystick steering
        self.angular_deadzone = 1e-3 # for key angular input

    def pose_callback(self, msg: Pose2D):
        # Update the current heading from the boat's state.
        self.current_pose_theta = msg.theta

    def compute_pwm(self, surge, turn):
        """
        Map the surge command (normalized velocity) and turning term into
        left/right PWM signals for the thrusters.
        """
        # Map surge to a base PWM command.
        # For forward commands (surge >= 0), map 1.0 -> max_forward_pwm.
        # For reverse commands (surge < 0), map -1.0 -> max_reverse_pwm.
        if surge >= 0:
            surge_pwm = surge * self.max_forward_pwm
        else:
            surge_pwm = surge * (-self.max_reverse_pwm)  # note: max_reverse_pwm is negative

        # Differential mixing: a positive turn command increases left thruster and decreases right thruster.
        left_pwm = surge_pwm + turn
        right_pwm = surge_pwm - turn

        # Clamp both PWM outputs to the allowed range.
        left_pwm = clamp(left_pwm, self.max_reverse_pwm, self.max_forward_pwm)
        right_pwm = clamp(right_pwm, self.max_reverse_pwm, self.max_forward_pwm)
        return left_pwm, right_pwm

    def convert_key(self, msg: Twist):
        """
        For key commands: linear.x provides the surge command and angular.z the turning input.
        If the angular input is negligible, the desired turning term is set to zero so that
        the heading reference “locks” to the current pose.
        """
        surge_command = msg.linear.x  # assume normalized command in [-1, 1]
        if abs(msg.angular.z) < self.angular_deadzone:
            turn_command = 0.0
        else:
            turn_command = msg.angular.z * self.turn_gain

        # Directly use the surge command from the key input.
        self.curr_vel = surge_command

        left_pwm, right_pwm = self.compute_pwm(self.curr_vel, turn_command)
        self.left_pwm_msg.data = left_pwm
        self.right_pwm_msg.data = right_pwm

        self.left_pwm_pub.publish(self.left_pwm_msg)
        self.right_pwm_pub.publish(self.right_pwm_msg)

    def convert_joy(self, msg: Joy):
        """
        For joystick commands:
          - The steering axis (index 0) provides a turning input.
            When its magnitude is below a deadzone, the turning command is set to zero,
            which makes the desired heading equal to the current pose.
          - Throttle (axis index 5) and brake (axis index 2) adjust the surge command.
          - The surge command is integrated and clamped between 0 and max_vel.
        """
        # Indices (adjust based on your controller configuration)
        joy_steer_i = 0
        joy_throttle_i = 5  # wired controller throttle axis
        joy_brake_i = 2     # wired controller brake axis

        # Steering: invert axis if necessary.
        steer = -msg.axes[joy_steer_i]
        if abs(steer) < self.steer_deadzone:
            turn_command = 0.0
        else:
            turn_command = steer * self.turn_gain

        # Compute surge command from throttle and brake.
        # Throttle is scaled from axis value in [1, -1] to roughly [-0.33, 1].
        throttle = (-msg.axes[joy_throttle_i] + 0.5) / 1.5
        # Brake is scaled from axis value in [1, -1] to [0, 1].
        brake = (-msg.axes[joy_brake_i] + 1) / 2

        self.curr_vel += (throttle * self.throttle_increment - brake * self.brake_increment)
        # Clamp surge command (for joystick, we assume only forward thrust)
        self.curr_vel = max(0.0, min(self.curr_vel, self.max_vel))

        # If no steering is provided, lock turn_command to zero so that both thrusters get equal surge.
        if abs(steer) < self.steer_deadzone:
            turn_command = 0.0

        left_pwm, right_pwm = self.compute_pwm(self.curr_vel, turn_command)
        self.left_pwm_msg.data = left_pwm
        self.right_pwm_msg.data = right_pwm

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
