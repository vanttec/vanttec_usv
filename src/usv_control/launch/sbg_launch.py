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
    sbg_config = os.path.join(
        get_package_share_directory('usv_control'),
        'config',
        'sbg_device.yaml'
    )

    sbg_node = Node(
        package='sbg_driver',
        executable='sbg_device',
        output='screen',
        parameters=[sbg_config],
    )

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


    return LaunchDescription([
        sbg_node,
        imu_converter_node,
    ])
