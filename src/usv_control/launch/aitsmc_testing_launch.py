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

    dynamic_sim_node2 = Node(
        package="usv_control",
        executable="dynamic_model_node",
        namespace="simulator2",
        remappings=[
            ("input/left_thruster", "/usv_new/left_thruster"),
            ("input/right_thruster", "/usv_new/right_thruster"),
            ("output/pose", "/usv_new/state/pose"),
            ("output/vel", "/usv_new/state/velocity"),
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

    aitsmc_new_node = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        namespace="asmc2",
        remappings=[
            ("in/mode", "/usv/op_mode"),
            ("in/pose", "/usv_new/state/pose"),
            ("in/velocity", "/usv_new/state/velocity"),
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("output/left_thruster", "/usv_new/left_thruster"),
            ("output/right_thruster", "/usv_new/right_thruster"),
        ],
        parameters=[
            {"new_k_u": 0.3},
            {"new_k_r": 0.2},
            {"new_epsilon_u": 0.2},
            {"new_alpha_u": 0.2},
            {"new_beta_u": 0.2},
            {"new_epsilon_r": 0.2},
            {"new_alpha_r": 0.2},
            {"new_beta_r": 0.2},
            {"new_tc_u": 2.0},
            {"new_tc_r": 2.0},
            {"new_q_u": 3.0},
            {"new_q_r": 3.0},
            {"new_p_u": 5.0},
            {"new_p_r": 5.0},
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

    return LaunchDescription([
        # rviz,
        dynamic_sim_node,
        dynamic_sim_node2,
        aitsmc_new_node,
        aitsmc_node,
        foxglove_bridge,
        # teleop_launch,
    ])
