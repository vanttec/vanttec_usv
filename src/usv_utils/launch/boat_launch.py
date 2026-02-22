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
    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_can'),
                'launch',
                'can_launch.py'
            ])
        ]),
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'sbg_launch.py'
            ])
        ]),
    )

    # Using general launch file in usv_control package.
    # As of RB 2026, this only calls new aitsmc node and line of sight node
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'usv_control_launch.py'
            ])
        ])
    )

    # For launching components related to zed camera, velodyne lidar and their fusion
    # Also launches yolo models
    vision_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visionsystemx'),
                'launch',
                # Temporary array of launch files for diverse use, parameters to be implemented
                # 'vision.launch_2.py' # <--- for launching both yolo models with all components
                # 'vision.launch.py' # <--- for launching only main yolo with all components
                # 'yolo.launch_2.py' # <--- for launching both yolos only (no camera or lidar, for rosbag use)
                # 'yolo.launch.py' # <---- for launching only main yolo
                # 'yolo.launch_3.py' # <----- for launching only secondary (indicator) yolo
                # 'cam_lidar.launch.py' # <---- for launching only camera and lidar, with fusion
                # 'cam.launch.py' # <---- for launching only camera
                # 'lidar.launch.py' # <---- for launching only lidar
		'vision.launch_noLiDAR.py'
            ])
        ])
    )

    system_validation_node = Node(
        package="usv_utils",
        executable="system_validation_node",
    )

    # Using general launch file in usv_missions package.
    # This mission_launch.py file takes parameters from conf files in the package, contents in this package are expected to be changing
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_missions'),
                'launch',
                'mission_launch.py'
            ])
        ])
    )

    # Legacy launch argument
    # sik_boat_node = Node(
    #     package="usv_comms",
    #     executable="sik_boat_node.py",
    # )

    return LaunchDescription([
        can_launch,
        sbg_launch,
        system_validation_node,
        control_launch,
        vision_lidar_launch,
        mission_launch,
    ])

# SMC , AITSMC, ASMC cualquiera es valido, son lo mismo USV_CONTROL
# Lo de vision, con el lidar y el pnp (fusion de sensores) 
# CAN ya sabbe 
# Mission handler
