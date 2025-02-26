import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    can_node = Node(
        package="vanttec_can_comms",
        executable="can_node",
        output="screen",
        remappings=[
            ("out/mode", "/usv/op_mode"),
            ("out/stm32_ping", "/usv/can/stm32_ping"),
            ("in/left_motor", "/usv/left_thruster"),
            ("in/right_motor", "/usv/right_thruster"),
        ],
    )

    return LaunchDescription([
        can_node,
    ])
