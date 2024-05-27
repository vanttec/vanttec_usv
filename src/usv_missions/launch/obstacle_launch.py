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
    obstacles_config = os.path.join(
        get_package_share_directory('usv_missions'),
        'config',
        'obstacles.yaml'
    )

    obstacle_publisher_node = Node(
        package='usv_missions',
        executable='obstacle_publisher_node',
        parameters=[obstacles_config],
    )

    return LaunchDescription([
        obstacle_publisher_node
    ])
