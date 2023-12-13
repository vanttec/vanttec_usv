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
    is_sim = DeclareLaunchArgument(
        'is_simulation',
        default_value = 'true',
        description = 'Defines if the application will run in simulation or in real life'
    )

    dynamic_sim_node = Node(
        package="usv_control",
        executable="dynamic_model_node",
        namespace="simulator",
        remappings=[
            ("input/left_thruster", "/usv/left_thruster"),
            ("input/right_thruster", "/usv/right_thruster"),
            ("output/pose", "/usv/state/pose"),
            ("output/vel", "/usv/state/velocity"),
        ],
    )

    usv_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('is_simulation'))
    )

    asmc_node = Node(
        package="usv_control",
        executable="asmc_node",
        namespace="asmc",
        remappings=[
            ("input/pose", "/usv/state/pose"),
            ("input/velocity", "/usv/state/velocity"),
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/heading", "/guidance/desired_heading"),
            ("output/left_thruster", "/usv/left_thruster"),
            ("output/right_thruster", "/usv/right_thruster"),
        ],
        parameters=[
            {"k_u": 0.1},
            {"k_psi": 0.5},
            {"kmin_u": 0.075},
            {"kmin_psi": 0.1},
            {"k2_u": 0.02},
            {"k2_psi": 0.2},
            {"mu_u": 0.01},
            {"mu_psi": 0.015},
            {"lambda_u": 0.001},
            {"lambda_psi": 1.5},
        ],
    )

    aitsmc_node = Node(
        package="usv_control",
        executable="aitsmc_node",
        namespace="asmc",
        remappings=[
            ("in/mode", "/usv/op_mode"),
            ("in/pose", "/usv/state/pose"),
            ("in/velocity", "/usv/state/velocity"),
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("output/left_thruster", "/usv/left_thruster"),
            ("output/right_thruster", "/usv/right_thruster"),
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
        ]
    )

    twist_to_setpoint_node = Node(
        package="usv_control",
        executable="twist_to_setpoint_node",
        namespace="simulator",
        remappings=[
            ("velocity", "/guidance/desired_velocity"),
            ("heading", "/guidance/desired_heading"),
            ("velocity_twist", "/guidance/desired_twist"),
        ],
    )


    return LaunchDescription([
        is_sim,

        # Log the value of is_simulation for debugging purposes
        LogInfo(
            condition=IfCondition(LaunchConfiguration('is_simulation')),
            msg="Running SIM mode."
        ),
        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('is_simulation')),
            msg="Running IRL mode."
        ),

        usv_description_launch,
        
        dynamic_sim_node,
        asmc_node,
        # aitsmc_node,
        twist_to_setpoint_node,
    ])