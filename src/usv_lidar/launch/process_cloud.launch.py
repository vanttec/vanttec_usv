from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('usv_lidar'),'rviz','testing_viz.rviz')
    params_file_path = os.path.join(get_package_share_directory('usv_lidar'),'config','params.yaml')


    rviz_node = Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', config_file_path])

    kitti_publishers_node =  Node(
            package='ros2_kitti_publishers', executable='kitti_publishers', output='screen')

    clustering_segmentation_node = Node(
        package='usv_lidar', name='clustering_segmentation', executable='clustering_segmentation', parameters=[params_file_path], output='screen')

    testing_node = Node(
        package='usv_lidar', name= 'testing_cloud', executable='testing_cloud', parameters=[params_file_path], output='screen')

    return LaunchDescription([
        # rviz_node,
        clustering_segmentation_node,
        # testing_node
    ])
