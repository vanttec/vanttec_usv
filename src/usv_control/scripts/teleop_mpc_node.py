#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Joy
from usv_interfaces.msg import WaypointList, Waypoint
from visualization_msgs.msg import Marker
import math

def normalize_angle(angle):
    """Normalize angle to be between -pi and pi."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')
        
        # Subscribe to the boat's current pose.
        self.pose_sub = self.create_subscription(
            Pose2D, '/usv/state/pose', self.pose_callback, 10
        )
        
        # Subscribe to joystick messages.
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        
        # Publisher for the waypoint list.
        self.waypoint_pub = self.create_publisher(WaypointList, '/usv/goals', 10)
        
        # Publisher for the visualization marker (virtual waypoint arrow).
        self.marker_pub = self.create_publisher(Marker, '/usv/tmp_goal', 10)
        
        # Latest boat pose (from /usv/state/pose).
        self.current_pose = Pose2D()
        
        # Active waypoint (None if no waypoint is being generated).
        self.active_waypoint = None
        
        # Last published waypoint (if none, start at (0,0,0)).
        self.last_waypoint = Waypoint()
        self.last_waypoint.x = 0.0
        self.last_waypoint.y = 0.0
        self.last_waypoint.theta = 0.0
        
        # Store previous joy state for detecting rising edges.
        self.prev_buttons = None
        self.prev_axes = None
        
        # Button mappings (adjust indices for your joystick if necessary).
        self.BACK_BUTTON = 6   # Press BACK to start waypoint creation.
        self.START_BUTTON = 7  # Press START to publish the waypoint.
        self.LB_BUTTON = 4     # Rotate waypoint left (90°).
        self.RB_BUTTON = 5     # Rotate waypoint right (90°).
        
        # D-PAD axes indices.
        # In the NED frame: x is forward, y is sideways.
        self.DPAD_HORIZONTAL = 6  # Left (-1) / Right (+1) → adjusts y (sideways)
        self.DPAD_VERTICAL = 7    # Down (-1) / Up (+1) → adjusts x (forwards)

    def pose_callback(self, msg: Pose2D):
        """Update the current pose from /usv/state/pose."""
        self.current_pose = msg

    def publish_virtual_marker(self):
        """Publish a red arrow marker representing the active virtual waypoint."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"
        marker.ns = "virtual_waypoint"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Use the active waypoint values to set the arrow's pose.
        if self.active_waypoint is not None:
            marker.pose.position.x = self.active_waypoint.x
            marker.pose.position.y = self.active_waypoint.y
            marker.pose.position.z = 0.0
            # Convert theta to a quaternion (rotation about z-axis).
            yaw = self.active_waypoint.theta
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw/2.0)
            marker.pose.orientation.w = math.cos(yaw/2.0)
        
        # Set the scale (size) of the arrow.
        marker.scale.x = 1.0   # Arrow length
        marker.scale.y = 0.1   # Shaft diameter
        marker.scale.z = 0.1   # Head diameter
        
        # Set color to red.
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Lifetime 0 means the marker remains until it is deleted.
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

    def delete_virtual_marker(self):
        """Delete the virtual waypoint marker."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"
        marker.ns = "virtual_waypoint"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def joy_callback(self, msg: Joy):
        """Process joystick inputs to generate/modify a waypoint."""
        # Initialize previous state on the first message.
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)
        if self.prev_axes is None:
            self.prev_axes = list(msg.axes)

        # --- BACK Button: Start waypoint creation ---
        # Rising edge detection.
        if msg.buttons[self.BACK_BUTTON] == 1 and self.prev_buttons[self.BACK_BUTTON] == 0:
            self.get_logger().info("BACK button pressed: creating waypoint at last sent waypoint")
            self.active_waypoint = Waypoint()
            self.active_waypoint.x = self.last_waypoint.x
            self.active_waypoint.y = self.last_waypoint.y
            self.active_waypoint.theta = self.last_waypoint.theta
            self.publish_virtual_marker()

        # --- Modify the Active Waypoint ---
        if self.active_waypoint is not None:
            # D-PAD Horizontal: adjust y coordinate by 1 meter (sideways).
            if msg.axes[self.DPAD_HORIZONTAL] != 0 and self.prev_axes[self.DPAD_HORIZONTAL] == 0:
                delta = msg.axes[self.DPAD_HORIZONTAL]  # Expected to be 1 or -1.
                self.active_waypoint.y += delta * 1.0
                self.get_logger().info(f"D-PAD horizontal: y shifted by {delta * 1.0} m")
                self.publish_virtual_marker()
            
            # D-PAD Vertical: adjust x coordinate by 1 meter (forwards/backwards).
            if msg.axes[self.DPAD_VERTICAL] != 0 and self.prev_axes[self.DPAD_VERTICAL] == 0:
                delta = msg.axes[self.DPAD_VERTICAL]  # Expected to be 1 or -1.
                self.active_waypoint.x += delta * 1.0
                self.get_logger().info(f"D-PAD vertical: x shifted by {delta * 1.0} m")
                self.publish_virtual_marker()
            
            # LB Button: Rotate waypoint left by 90° (add π/2).
            if msg.buttons[self.LB_BUTTON] == 1 and self.prev_buttons[self.LB_BUTTON] == 0:
                self.active_waypoint.theta = normalize_angle(self.active_waypoint.theta + math.pi/2)
                self.get_logger().info("LB pressed: waypoint rotated left 90°")
                self.publish_virtual_marker()
            
            # RB Button: Rotate waypoint right by 90° (subtract π/2).
            if msg.buttons[self.RB_BUTTON] == 1 and self.prev_buttons[self.RB_BUTTON] == 0:
                self.active_waypoint.theta = normalize_angle(self.active_waypoint.theta - math.pi/2)
                self.get_logger().info("RB pressed: waypoint rotated right 90°")
                self.publish_virtual_marker()
            
            # --- START Button: Publish the waypoint ---
            if msg.buttons[self.START_BUTTON] == 1 and self.prev_buttons[self.START_BUTTON] == 0:
                self.get_logger().info("START button pressed: publishing waypoint")
                wp_list = WaypointList()
                wp_list.waypoint_list.append(self.active_waypoint)
                self.waypoint_pub.publish(wp_list)
                # Save the published waypoint as the new last waypoint.
                self.last_waypoint = self.active_waypoint
                # Reset active waypoint after publishing.
                self.active_waypoint = None
                # Delete the virtual marker.
                self.delete_virtual_marker()

        # Save current state for next callback (for rising edge detection).
        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
