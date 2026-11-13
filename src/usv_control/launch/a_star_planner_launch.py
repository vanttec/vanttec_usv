import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('usv_control'), 'config')

    site = DeclareLaunchArgument(
        'site', default_value='sim',
        description='Sitio: nombre de un archivo en config/sites/ (sin .yaml)')

    planner = Node(
        package='usv_control',
        executable='a_star_planner_node',
        name='a_star_planner_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'a_star_planner.yaml'),
            PathJoinSubstitution([config_dir, 'sites', [LaunchConfiguration('site'), '.yaml']]),
        ],
    )

    return LaunchDescription([site, planner])
