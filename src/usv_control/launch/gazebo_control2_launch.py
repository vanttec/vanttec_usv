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

    aitsmc_new_node = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        remappings=[
            ("setpoint/velocity", "/guidance/desired_velocity"),
            # ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("setpoint/heading", "/guidance/desired_heading"),
        ],
        parameters=[
            {"k_u": 1.},
            {"k_psi": 0.2},
            {"epsilon_u": 0.3},
            {"k_alpha_u": 0.8},
            {"k_beta_u": 0.5},
            {"epsilon_psi": 0.3},
            {"k_alpha_psi": 3.0},
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


    obstacle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_missions'),
                'launch',
                'obstacle_launch.py'
            ])
        ]),
    )

    mission_handler_node = Node(
        package="usv_missions",
        executable="mission_handler_node",
    )

    weights_config = os.path.join(
        get_package_share_directory('usv_control'),
        'config',
        'weights.yaml'
    )

    mpc_node = Node(
        package="usv_control",
        executable="mpc_node",
        parameters=[weights_config],
    )

    waypoint_handler_node = Node(
        package="usv_control",
        executable="waypoint_handler_node",
    )

    obstacle_nearest_publisher = Node(
        package="usv_utils",
        executable="obstacle_nearest_publisher",
    )

    return LaunchDescription([
        # gz,
        # ks,
        # odom,

        # rviz,
        # foxglove_bridge,

        aitsmc_new_node,

        # teleop_launch,
        
        mpc_node,
        waypoint_handler_node,
        
        obstacle_launch,
        mission_handler_node,
        obstacle_nearest_publisher,
    ])