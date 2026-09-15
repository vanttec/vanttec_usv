import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # CAN interface argument — override with:
    #   ros2 launch usv_can can_launch.py can_interface:=can0
    # Note: physical interface must be brought up first:
    #   sudo ip link set can0 type can bitrate 125000
    #   sudo ip link set up can0
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface name (e.g. can0, can_vtec)"
    )

    can_node = Node(
        package="usv_can",
        executable="can_node",
        parameters=[{
            "can_interface": LaunchConfiguration("can_interface"),
        }],
        remappings=[
            # RX — STM32 → Jetson
            ("out/stm32_ping",  "/usv/can/stm32_ping"),   # UInt32: STM32 uptime ms (1 Hz)
            ("out/battery",     "/usv/battery"),            # Float32MultiArray: [voltage, current]
            ("out/actuators",   "/usv/actuators"),          # UInt8MultiArray: [pump, actuator]

            # TX — Jetson thruster teleop (IndividualThrusterNode → motors topic)
            ("in/left_motor",   "/usv/left_thruster"),
            ("in/right_motor",  "/usv/right_thruster"),
        ],
    )

    return LaunchDescription([
        can_interface_arg,
        can_node,
    ])
