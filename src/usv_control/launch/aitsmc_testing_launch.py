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
    dynamic_sim_node1 = Node(
        package="usv_control",
        executable="dynamic_model_node",
        namespace="new1",
        parameters=[
            {"boatname": "usv"},
        ],
    )

    dynamic_sim_node2 = Node(
        package="usv_control",
        executable="dynamic_model_node",
        namespace="new2",
        parameters=[
            {"boatname": "usv2"},
        ],
    )

    dynamic_sim_node3 = Node(
        package="usv_control",
        executable="dynamic_model_node",
        namespace="normal",
        parameters=[
            {"boatname": "usv3"},
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

    aitsmc_new_node1 = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        namespace="new1",
        # output='screen',
        # emulate_tty=True,
        # arguments=[('__log_level:=debug')],
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
            {"k_beta_u": 0.1},
            {"epsilon_psi": 0.3},
            {"k_alpha_psi": 1.},
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

    aitsmc_new_node2 = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        namespace="new2",
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
            {"k_beta_u": 0.1},
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

    aitsmc_node = Node(
        package="usv_control",
        executable="aitsmc_node",
        namespace="normal",
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

    # asmc_node = Node(
    #     package="usv_control",
    #     executable="asmc_node",
    #     namespace="c2",
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

    return LaunchDescription([
        # rviz,
        dynamic_sim_node1,
        dynamic_sim_node2,
        # dynamic_sim_node3,
        aitsmc_new_node1,
        aitsmc_new_node2,
        # aitsmc_node,
        # asmc_node,
        foxglove_bridge,
        teleop_launch,
    ])
    
