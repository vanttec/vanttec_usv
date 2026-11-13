import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_localization(context):
    # site vacío = comportamiento de siempre (origen = primer fix, sin YAML).
    site = LaunchConfiguration('site').perform(context)
    params = []
    if site:
        params.append(os.path.join(get_package_share_directory('usv_control'),
                                   'config', 'sites', site + '.yaml'))
    return [Node(
        package='usv_localization',
        executable='localization',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_localization')

    sbg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sbg_launch.py')
        )
    )

    site = DeclareLaunchArgument(
        'site', default_value='',
        description='Sitio en usv_control/config/sites/ (sin .yaml). Vacío = datum automático')

    return LaunchDescription([
        site,
        sbg,
        OpaqueFunction(function=make_localization),
    ])
