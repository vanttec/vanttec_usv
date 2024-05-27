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
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'gazebo_launch.py'
            ])
        ]),
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
            ("setpoint/pivot", "/guidance/pivot_enable"),
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

    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")
    
    waypoint_handler = Node(
        package="usv_control",
        executable="waypoint_handler_node")    

    odom_converter = Node(
        package="usv_utils",
        executable="odom_converter_node")    

    tf2 = Node(
        package="usv_control",
        executable="usv_tf2_broadcaster_node") 
    
    ks = Node(
        package='usv_utils',
        executable='killswitch_node',
        output='screen',
    )

    los_node = Node(
        package="usv_control",
        executable="los_node")    

    return LaunchDescription([
        rviz,
        # asmc_node,
        aitsmc_node,
        los_node,
        odom_converter,
        foxglove_bridge,
        # waypoint_handler,
        tf2,
        gz,
        ks,
    ])