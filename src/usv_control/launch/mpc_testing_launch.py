'''
This script launches the mpc node with its parameters. Ideally this launch will not exist nor will it be used. It's just for testing MPC performance!
'''

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dynamic_sim_node = Node(
        package="usv_control",
        executable="dynamic_model_node",
        parameters=[
            {"boatname": "usv"},
        ],
    )

    mpc_node = Node(
        package="usv_control",
        executable="mpc_node",
    )

    aitsmc_node = Node(
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
            {"epsilon_u": 0.3},
            {"k_alpha_u": 1.},
            {"k_beta_u": 0.5},
            {"epsilon_psi": 0.5},
            {"k_alpha_psi": 1.},
            {"k_beta_psi": 0.75},
            {"tc_u": 2.0},
            {"tc_psi": 2.0},
            {"q_u": 3.0},
            {"q_psi": 3.0},
            {"p_u": 5.0},
            {"p_psi": 5.0},
            {"adaptive": 1.0},
        ],
    )
     
    return LaunchDescription([
        dynamic_sim_node,
        mpc_node,
        aitsmc_node,
    ])
