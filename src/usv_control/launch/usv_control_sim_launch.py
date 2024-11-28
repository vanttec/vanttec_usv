import os
from launch import LaunchDescription

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    dynamic_sim_node = Node(
        package="usv_control",
        executable="dynamic_model_node",
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

    aitsmc_node = Node(
        package="usv_control",
        executable="aitsmc_node",
        remappings=[
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
        ],
        parameters=[
            {"k_u": 0.3},
            {"k_r": 0.2},
            {"kmin_u": 0.01},
            {"kmin_r": 0.01},
            {"k2_u": 0.01},
            {"k2_r": 0.01},
            {"mu_u": 0.05},
            {"mu_r": 0.04},
            {"tc_u": 2.0},
            {"tc_r": 2.0},
            {"q_u": 3.0},
            {"q_r": 3.0},
            {"p_u": 5.0},
            {"p_r": 5.0},
        ],
    )

    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")

    los_node = Node(
        package="usv_control",
        executable="los_node")    

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'teleop_launch.py'
            ])
        ]),
    )

    base_link_tf = launch_ros.actions.Node(
        name='usv_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1.5','0','0','0','0','3.14159','usv','base_link']
        )

    return LaunchDescription([
        rviz,
        dynamic_sim_node,
        aitsmc_node,
        los_node,
        foxglove_bridge,
        # teleop_launch,
        base_link_tf,
    ])
