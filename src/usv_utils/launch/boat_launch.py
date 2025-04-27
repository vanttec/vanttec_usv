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
    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'can_launch.py'
            ])
        ]),
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'sbg_launch.py'
            ])
        ]),
    )

    system_validation_node = Node(
        package="usv_utils",
        executable="system_validation_node",
    )

    sik_boat_node = Node(
        package="usv_comms",
        executable="sik_boat_node.py",
    )

    return LaunchDescription([
        can_launch,
        sik_boat_node,
        sbg_launch,
        system_validation_node,
    ])

