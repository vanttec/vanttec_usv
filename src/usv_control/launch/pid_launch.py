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
    pid_node = Node(
        package="usv_control",
        executable="pid_node",
        remappings=[
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("setpoint/heading", "/guidance/desired_heading"),
        ],
        parameters=[
            {"pk_u": 1.},
            {"i_u": 0.5},
            {"d_u": 0.2},
            {"pk_psi": 1.},
            {"i_psi": 0.5},
            {"d_psi": 0.2},
            {"tc_u": 2.0},
            {"tc_psi": 2.0},
            {"q_u": 3.0},
            {"q_psi": 3.0},
            {"p_u": 5.0},
            {"p_psi": 5.0},
        ],
    )

    return LaunchDescription([
        pid_node,
    ])
