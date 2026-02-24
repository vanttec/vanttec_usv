'''
This script launches the mpc node with its parameters, and auxiliary nodes
'''

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # weights_config = os.path.join(
    #     get_package_share_directory('usv_control'),
    #     'config',
    #     'weights.yaml'
    # )

    mpc_node = Node(
        package="usv_control",
        executable="mpc_node",
        # parameters=[weights_config],
        ros_arguments=['--log-level', 'FATAL'],
    )

    spline_publisher_node = Node(
        package="usv_control",
        executable="spline_publisher_node",
    )

    obstacle_nearest_publisher = Node(
        package="usv_utils",
        executable="obstacle_nearest_publisher",
    )
    
    mpc_gui = Node(
        package="usv_control",
        executable="mpc_gui.py",
    )

    return LaunchDescription([
        mpc_node,
        # spline_publisher_node,
        obstacle_nearest_publisher,
        mpc_gui,
    ])
