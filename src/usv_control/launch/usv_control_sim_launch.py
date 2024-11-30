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

    aitsmc_new_node = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        remappings=[
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("setpoint/heading", "/guidance/desired_heading"),
        ],
        parameters=[
            {"k_u": 1.},
            {"k_psi": 0.2},
            {"epsilon_u": 0.3},
            {"k_alpha_u": 1.},
            {"k_beta_u": 0.5},
            {"epsilon_psi": 0.3},
            {"k_alpha_psi": 3.},
            {"k_beta_psi": 0.1},
            {"tc_u": 2.0},
            {"tc_psi": 2.0},
            {"q_u": 3.0},
            {"q_psi": 3.0},
            {"p_u": 5.0},
            {"p_psi": 5.0},
            {"adaptive": 0.},
        ],
    )

    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")

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
        arguments=['0.','0','0','0','0','3.14159','usv','base_link']
        )

    world_tf = launch_ros.actions.Node(
        name='usv_tf2',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.','0','0','0','0','3.14159','world','inertial']
        )

    return LaunchDescription([
        rviz,
        dynamic_sim_node,
        aitsmc_new_node,
        foxglove_bridge,
        teleop_launch,
        base_link_tf,
        world_tf,
    ])
