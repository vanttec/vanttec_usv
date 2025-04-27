import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():

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

    path_publisher_node = Node(
        package="usv_control",
        executable="path_publisher_node",
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

    waypoint_handler_node = Node(
        package="usv_control",
        executable="waypoint_handler_node",
    )

    return LaunchDescription([
        # mpc_node,
        waypoint_handler_node,
        # sbg_launch,
        aitsmc_new_node,
        # can_node,
    ])
