'''
Launches fixed path for control to follow, with a file that sets the path's shape and size
'''

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

    path_config = os.path.join(
        get_package_share_directory('usv_control'),
        'config',
        'path_config.yaml'
    )

    path_publisher_node = Node(
        package="usv_control",
        executable="path_publisher_node",
        parameters=[path_config],
    )

    return LaunchDescription([
        path_publisher_node,
    ])
