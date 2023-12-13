import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    master_node = Node(
        package='usv_master',
        executable='usv_master',
        name='usv_master',
    )

    wp_node = Node(
        package="usv_control",
        executable="waypoint_handler_node",
        remappings=[
            ("/setpoint/heading", "/guidance/desired_heading"),
            ("/setpoint/velocity", "/guidance/desired_velocity"),
        ],
    )

    tf2_node = Node(
        package='usv_control',
        executable='usv_tf2_broadcaster_node',
        name='broadcaster1',
        parameters=[
              {'usv_name': 'usv'}
        ],
        condition=IfCondition(LaunchConfiguration('is_simulation'))
    )

    return LaunchDescription([
        tf2_node,
        wp_node,
        master_node

        
    ])