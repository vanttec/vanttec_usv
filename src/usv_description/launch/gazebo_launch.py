# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_usv_description = get_package_share_directory('usv_description')

    # Launch Gazebo with command
    #  Useful if user has different versions of Gazebo
    world_path = PathJoinSubstitution(
        [pkg_usv_description, 'worlds', 'waves.sdf'])
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_path],
        output='screen',
        additional_env={
            'GZ_IP': '127.0.0.1'
        }
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/vtec_s4/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/vtec_s4/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/gz_sim/odometry@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
            # '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/zed_rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/zed_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/zed_rgbd/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            ("/zed_rgbd/image", "/bebblebrox/video"),
            ("/zed_rgbd/points", "/bebblebrox/points"),
            ("/lidar/points", "/velodyne_points"),
        ],

        output='screen',
        additional_env={
            'GZ_IP': '127.0.0.1',
        }
    )

    # base_link -> cam tf
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.65', '0', '0.3', '0', '0', '0',
                'usv',
                'vtec_s4/base_link/rgbd_camera']
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.55', '0', '0', '0',
                'usv',
                'vtec_s4/base_link/gpu_lidar']
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        cam_tf,
        lidar_tf,
    ])


'''
# Lidar Frame
vtec_s4/base_link/gpu_lidar
'''
