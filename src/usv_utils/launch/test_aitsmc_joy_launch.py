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
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    teleop_node = Node(
        package="usv_control",
        executable="teleop_aitsmc_node.py",
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        # Also, run regular LAUNCH on jetson (CAN+SBG)
    ])
