from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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

    dynamic_model_node = Node(
        package="usv_control",
        executable="dynamic_model_node"
    )

    return LaunchDescription([
        aitsmc_node,
        dynamic_model_node,
    ])
