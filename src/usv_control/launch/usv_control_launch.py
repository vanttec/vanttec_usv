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
    is_sim = DeclareLaunchArgument(
        'is_simulation',
        default_value = 'false',
        description = 'Defines if the application will run in simulation or in real life'
    )

    weights_config = os.path.join(
        get_package_share_directory('usv_control'),
        'config',
        'weights.yaml'
    )

    mpc_node = Node(
        package="usv_control",
        executable="mpc_node",
        parameters=[weights_config],
    )

    waypoint_handler_node = Node(
        package="usv_control",
        executable="waypoint_handler_node",
    )

    imu_converter_node = Node(
        package='usv_utils',
        executable='imu_converter_node',
        output='screen',
        remappings=[
            ("out/pose", "/usv/state/pose"),
            ("out/velocity", "/usv/state/velocity"),
            ("in/data", "/imu/data"),
            ("in/odometry", "/imu/odometry"),
            ("in/pose", "/imu/pos_ecef"),
        ],
        condition=UnlessCondition(LaunchConfiguration('is_simulation')),
    )


    dynamic_sim_node = Node(
        package="usv_control",
        executable="dynamic_model_node",
        namespace="simulator",
        remappings=[
            ("input/left_thruster", "/usv/left_thruster"),
            ("input/right_thruster", "/usv/right_thruster"),
            ("output/pose", "/usv/state/pose"),
            ("output/vel", "/usv/state/velocity"),
        ],
        condition=IfCondition(LaunchConfiguration('is_simulation')),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
        # condition=IfCondition(LaunchConfiguration('is_simulation'))
    )

    sbg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'sbg_launch.py'
            ])
        ]),
        # condition=IfCondition(LaunchConfiguration('is_simulation'))
    )


    asmc_node = Node(
        package="usv_control",
        executable="asmc_node",
        namespace="asmc",
        remappings=[
            ("input/pose", "/usv/state/pose"),
            ("input/velocity", "/usv/state/velocity"),
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/heading", "/guidance/desired_heading"),
            ("setpoint/pivot", "/guidance/pivot_enable"),
            ("output/left_thruster", "/usv/left_thruster"),
            ("output/right_thruster", "/usv/right_thruster"),
        ],
        parameters=[
            {"k_u": 0.05},
            {"k_psi": 0.2},
            {"kmin_u": 0.025},
            {"kmin_psi": 0.1},
            {"k2_u": 0.02},
            {"k2_psi": 0.2},
            {"mu_u": 0.05},
            {"mu_psi": 0.01},
            {"lambda_u": 0.001},
            {"lambda_psi": 0.5},
        ],
    )

    aitsmc_node = Node(
        package="usv_control",
        executable="aitsmc_node",
        namespace="asmc",
        remappings=[
            ("in/mode", "/usv/op_mode"),
            ("in/pose", "/usv/state/pose"),
            ("in/velocity", "/usv/state/velocity"),
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("output/left_thruster", "/usv/left_thruster"),
            ("output/right_thruster", "/usv/right_thruster"),
        ],
        parameters=[
            {"k_u": 0.3},
            {"k_r": 0.2},
            {"kmin_u": 0.01},
            {"kmin_r": 0.01},
            {"k2_u": 0.01},
            {"k2_r": 0.01},
            {"mu_u": 0.05},
            {"mu_r": 0.04},
            {"tc_u": 2.0},
            {"tc_r": 2.0},
            {"q_u": 3.0},
            {"q_r": 3.0},
            {"p_u": 5.0},
            {"p_r": 5.0},
        ],
    )

    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")
    
    tf2 = Node(
        package="usv_control",
        executable="usv_tf2_broadcaster_node",
    )

    los_node = Node(
        package="usv_control",
        executable="los_node")    

    obstacle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_missions'),
                'launch',
                'obstacle_launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('is_simulation'))
    )


    aitsmc_new_node = Node(
        package="usv_control",
        executable="aitsmc_new_node",
        remappings=[
            ("setpoint/velocity", "/guidance/desired_velocity"),
            ("setpoint/angular_velocity", "/guidance/desired_angular_velocity"),
            ("setpoint/heading", "/guidance/desired_heading"),
        ],
        parameters=[
            {"k_u": 1.},
            {"k_psi": 0.2},
            {"epsilon_u": 0.1},
            {"k_alpha_u": 1.},
            {"k_beta_u": 0.5},
            {"epsilon_psi": 0.1},
            {"k_alpha_psi": 0.8},
            {"k_beta_psi": 0.5},
            {"tc_u": 2.0},
            {"tc_psi": 2.0},
            {"q_u": 3.0},
            {"q_psi": 3.0},
            {"p_u": 5.0},
            {"p_psi": 5.0},
            {"adaptive": 1.},
        ],
    )

    can_node = Node(
        package="vanttec_can_comms",
        executable="can_node",
        output="screen",
        remappings=[
            ("out/mode", "/usv/op_mode"),
            ("out/stm32_ping", "/usv/can/stm32_ping"),
            ("in/left_motor", "/usv/left_thruster"),
            ("in/right_motor", "/usv/right_thruster"),
        ],
        condition=UnlessCondition(LaunchConfiguration('is_simulation'))
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usv_control'),
                'launch',
                'teleop_launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        is_sim,

       # rviz,
        # dynamic_sim_node,
        # asmc_node,
        # aitsmc_node,
        aitsmc_new_node,        
        # los_node,
        # sbg_launch,
        # imu_converter_node,
        # foxglove_bridge,
        # tf2,
        # can_node,
        # teleop_launch,
        # obstacle_launch,
        # waypoint_handler_node,
        # mpc_node,
    ])
