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
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    dynamic_sim_node = Node(
        package="usv_control",
        executable="dynamic_model_node",
        parameters=[
            {"boatname": "usv"},
        ],
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

    # asmc_node = Node(
    #     package="usv_control",
    #     executable="asmc_node",
    #     namespace="asmc",
    #     remappings=[
    #         ("setpoint/velocity", "/guidance/desired_velocity"),
    #         ("setpoint/heading", "/guidance/desired_heading"),
    #     ],
    #     parameters=[
    #         {"k_u": 0.05},
    #         {"k_psi": 0.2},
    #         {"kmin_u": 0.025},
    #         {"kmin_psi": 0.1},
    #         {"k2_u": 0.02},
    #         {"k2_psi": 0.2},
    #         {"mu_u": 0.05},
    #         {"mu_psi": 0.01},
    #         {"lambda_u": 0.001},
    #         {"lambda_psi": 0.5},
    #     ],
    # )

    aitsmc_new_node = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        remappings=[
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
        ],
        parameters=[
            {"k_u": 1.},
            {"k_r": 0.2},
            {"epsilon_u": 0.5},
            {"alpha_u": 5.},
            {"beta_u": 0.01},
            {"epsilon_r": 0.5},
            {"alpha_r": 0.2},
            {"beta_r": 0.2},
            {"tc_u": 2.0},
            {"tc_r": 2.0},
            {"q_u": 3.0},
            {"q_r": 3.0},
            {"p_u": 5.0},
            {"p_r": 5.0},
            {"adaptive": 1.},
        ],
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

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'teleop_launch.py'
            ])
        ]),
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
        rviz,
        dynamic_sim_node,
        # asmc_node,
        # aitsmc_node,
        aitsmc_new_node,
        # los_node,
        foxglove_bridge,
        obstacle_launch,
        # teleop_launch,
        mission_handler_node,
        waypoint_handler_node,
        obstacle_nearest_publisher,
        mpc_node,
    ])
