'''
This script is used to launch a gz sim environment for control. This 1/2 file deals with all the gz sim's side and interface to ros2
'''

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
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'gazebo_launch.py'
            ])
        ]),
    )

    ks = Node(
        package='usv_utils',
        executable='killswitch_node',
        output='screen',
    )

    odom = Node(
        package='usv_utils',
        executable='odom_converter_node',
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
    )
    
    gz_sim_zed_node = Node(
        package="usv_utils",
        executable="gz_sim_zed_node")

    return LaunchDescription([
        gz,
        ks,
        odom,
        # rviz,
        gz_sim_zed_node,
    ])