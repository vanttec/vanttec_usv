'''
Launch file with all scripts needed for autonomous navigation
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
    spline_publisher_node = Node(
        package="usv_control",
        executable="spline_publisher_node"
    )

    los_node = Node(
        package="usv_control",
        executable="los_node"
    )

    global_obstacle_register_node = Node(
        package="usv_missions",
        executable="global_obstacle_register_node"
    )

    cluster_node = Node(
            package='lidar_cluster',
            executable='euclidean_spatial',
            output='screen',
            parameters=[
                {'points_in_topic': 'velodyne_points'},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'tolerance': 0.8},
                {'verbose1': False},
            ]
    )

    lidar_mission_finder_node = Node(
        package="usv_utils",
        executable="lidar_mission_finder_node"
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visionsystemx'),
                'launch',
                'vision.launch.py'
            ])
        ]),
    )

    aitsmc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'aitsmc_launch.py'
            ])
        ]),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_missions'),
                'launch',
                'mission_launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        spline_publisher_node,
        los_node,
        global_obstacle_register_node,
        lidar_mission_finder_node,
        cluster_node,
        
        vision_launch,
        aitsmc_launch,
        mission_launch,
    ])
