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
    imu_converter_node = Node(
        package='usv_utils',
        executable='imu_converter_node',
        output='screen',
        remappings=[
            ("out/pose", "/usv/state/pose"),
            ("out/velocity", "/usv/state/velocity"),
            ("in/data", "/imu/data"),
            ("in/odometry", "/imu/odometry"),
            ("in/pose", "/imu/pos_ecef"),
        ],
    )

    urdf_file_name = 'usv.urdf'
    urdf = os.path.join(
        get_package_share_directory("usv_description"), "urdf/", urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
   
    urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
        arguments=[urdf])

    tf2 = Node(
        package="usv_control",
        executable="usv_tf2_broadcaster_node",
    )

    return LaunchDescription([
        imu_converter_node,
        urdf_node,
        tf2,
    ])

