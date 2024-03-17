from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='voxel_grid_filter',
            executable='voxel_grid_filter',
            name='voxel_filter_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='sdv_lidar_processing',
            executable='Lidar_Processing_node',
            name='lidar_processing_node',
            output='screen',
            parameters=[],
        ),
    ])
