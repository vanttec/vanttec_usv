from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_localization',
            executable='mag_orientation_sbg',
            output='screen'
        )
    ])
