import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_localization')

    sbg_config = os.path.join(pkg_share, 'config', 'sbg_device_uart.yaml')

    sbg_driver = Node(
        package='sbg_driver',
        executable='sbg_device',
        name='sbg_device',
        output='screen',
        parameters=[sbg_config],
    )

    localization = Node(
        package='usv_localization',
        executable='localization',
        name='usv_localization',
        output='screen',
    )

    return LaunchDescription([
        sbg_driver,
        localization,
    ])
