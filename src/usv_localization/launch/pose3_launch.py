from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_localization',
            executable='pose3_sbg',
            output='screen'
        )
    ])