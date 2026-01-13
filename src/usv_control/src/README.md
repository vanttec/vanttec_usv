# MPC Demo Manual
## Launch files
ros2 launch usv_description rviz_launch.py
ros2 launch usv_control mpc_launch.py
ros2 launch usv_control obstacles_mpc_launch.py
ros2 launch usv_control control_sim_launch.py (if extreme mode, run dynamic_model_node instead and uncomment thruster_pub_ lines at approx line 540)

## Static avoidance:
- Avoidance weight set to 2.0

- In obstacles.yaml, change:

    - (line 3): dynamic_obstacles: false

    - (line 9): x: 3.0

          

## Dynamic avoidance

- Avoidance weight set to 5.0

- In obstacles.yaml, change:

    - (line 3): dynamic_obstacles: true

    - (line 9): x: 3000.0