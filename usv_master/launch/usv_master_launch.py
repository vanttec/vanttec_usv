from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_master',
            type='usv_master.cpp',
            name='usv_master'
        )
    ])