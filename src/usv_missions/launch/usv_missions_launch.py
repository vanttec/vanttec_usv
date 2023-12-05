import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_missions',
            executable='usv_missions',
            name='usv_missions',
        )
    ])