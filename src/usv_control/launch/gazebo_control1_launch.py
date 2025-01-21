import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'gazebo_launch.py'
            ])
        ]),
    )

    ks = Node(
        package='usv_utils',
        executable='killswitch_node',
        output='screen',
    )

    odom = Node(
        package='usv_utils',
        executable='odom_converter_node',
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
    )
    
    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")

    return LaunchDescription([
        gz,
        ks,
        odom,

        rviz,
        foxglove_bridge,

        # aitsmc_new_node,

        # # teleop_launch,
        
        # mpc_node,
        # waypoint_handler_node,
        
        # obstacle_launch,
        # mission_handler_node,
        # obstacle_nearest_publisher,
    ])