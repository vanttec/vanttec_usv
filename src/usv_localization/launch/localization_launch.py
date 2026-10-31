import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('usv_localization')

    sbg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sbg_launch.py')
        )
    )

    return LaunchDescription([
        sbg,
        Node(
            package='usv_localization',
            executable='localization',
            output='screen'
        )
    ])
