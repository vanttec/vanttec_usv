import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_master',
            executable='usv_master',
            name='usv_master',
        )
    ])