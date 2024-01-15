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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_usv_description = get_package_share_directory('usv_description')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
            pkg_usv_description,
            'worlds',
            # 'diffo.sdf'
            'nbpark_custom.sdf'
        ])}.items(),
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_usv_description, 'rviz', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/model/vtec_s3/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                   '/model/vtec_s3/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                   '/world/nbpark/model/vtec_s3/link/base_link/sensor/imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/world/nbpark/model/vtec_s3/link/base_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                   '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/zed_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   ],
        # parameters=[{
        #     'qos_overrides./model/my_roboboat.subscriber.reliability': 'reliable',
        #             }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        # rviz
    ])

'''
# Lidar Frame
vtec_s3/base_link/gpu_lidar
'''