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
    waypoint_handler_node = Node(
        package="usv_control",
        executable="waypoint_handler_node",
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
    
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_missions'),
                'launch',
                'mission_launch.py'
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
            {"epsilon_u": 0.1},
            {"k_alpha_u": 1.},
            {"k_beta_u": 0.5},
            {"epsilon_psi": 0.1},
            {"k_alpha_psi": 0.4},
            {"k_beta_psi": 0.5},
            {"tc_u": 2.0},
            {"tc_psi": 2.0},
            {"q_u": 3.0},
            {"q_psi": 3.0},
            {"p_u": 5.0},
            {"p_psi": 5.0},
            {"adaptive": 1.},
        ],
    )

    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'can_launch.py'
            ])
        ]),
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

    return LaunchDescription([
        aitsmc_new_node,        
        sbg_launch,
        can_launch,
     #   waypoint_handler_node,
        vision_launch,
    #     mission_launch,
    ])
