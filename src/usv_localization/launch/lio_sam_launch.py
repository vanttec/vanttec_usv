import os
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('usv_localization')
    custom_config = os.path.join(share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lio_sam'),
                    'launch',
                    'run.launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': custom_config,
            }.items()
        )
    ])
