# USV_control 

## Package overview

### CMakeLists

The CMakeLists.txt file configures the build process of a ROS2 package named usv_control with various executables and dependencies. It creates executables for various node source files specifying dependencies and language features, specifies installation destinations for them and other resources, enables linting and testing and generates the package manifest.

### config

The config subdirectory is the home of multiple yaml files, which define the behavior of multiple processes in this package. It includes:

- sbg_device.yaml: A configuration file in YAML format used for the configuration of a ROS2 node that interacts with an SBG device via a UART interface. This includes node frequency, UART settings, sensor parameters, magnetometer settings, GPS configurations, output configurations, etc. This configuration overrides the configured settings of the used Ellipse-2d as long as confWithRos is set to true.
- path_config.yaml: configures parameters for the path_publisher_node. Note: This is not currently included in the main launch file.
- weights_config.yaml: Defines weights for Model Predictive control. Note: Model Predictive Control is not currently included in the main launch file.


### launch

The launch subdirectory includes a series of python launch files, which handle the configuration and execution of ROS2 nodes and accompanying executables. Relevant launch files include:

- usv_control_launch.py: Main launch file for boat deployment. Launches all the necessary nodes for the functionality of the currently utilized contol strategy, including launching localization nodes. Currently, this includes:
  -  the SBG device ROS2 node, alongside the converter of its output topics.
  -  The waypoint handler node.
  -  The ligh
  -  The AITSMC new node.
- sbg_launch.py: Launch file called by usv_control_launch.py, it launches both the sbg node and the imu converter node.

## ROS2 nodes

### aitsmc_new_node

The aitsmc_new_node defines the most recent version of sliding mode control for the USV: Adaptive integral terminal sliding mode control. This ros2 node is responsible for the implementation of the logic of the controller, as a node that receives setpoint and state information then computes control outputs and publishes thruster values. 

Note: some topics for this node are remapped in usv_control_launch.py. The listed topics are the resulting topics after the remapping:

#### ROS2 topic subscriptions:

1. /guidance/desired_velocity (std_msgs/msg/Float64)
2. /guidance/desired_angular_velocity (std_msgs/msg/Float64)
3. /guidance/desired_heading (std_msgs/msg/Float64)
4. /usv/state/velocity (geometry_msgs/msg/Vector3)
5. /usv/state/pose (geometry_msgs/msg/Pose2D)

#### ROS2 topic publications:

1. /usv/right_thruster (std_msgs/msg/Float64)
2. /usv/left_thruster (std_msgs/msg/Float64)
3. A wide array of debug Float64 topics

#### Other SMC nodes

This package also includes an asmc_node and a aitsmc_node. These nodes includes older iterations of the control logic but maintain most of the functionality stated above, including topic subscriptions and publications.

### Waypoint handler node

The waypoint handler node is the primary connection between usv_control and the usv_missions package. It intakes the established goals from usv_missions, and publishes key information such as when a waypoint is completed and the path to follow.

#### ROS2 topic subscriptions:

1. /usv/state/pose (geometry_msgs/msg/Pose2D)
2. /usv/goals (usv_interfaces/msg/WaypointList)
3. /usv/mission/id (std_msgs/msg/Int8) 

#### ROS2 topic publications:

1. /usv/wp_arrived (std_msgs/msg/Bool)
2. /usv/path_to_follow (nav_msgs/msg/Path)
3. /usv/current_path_ref (nav_msgs/msg/Path)
4. /goals_markers (visualization_msgs/msg/MarkerArray)

#### dynamic_model_sim

ROS2 node that simulates the dynamics of our unmanned surface vehicle with a dynamic model and publishes its position, velocity and odometry information.

#### kinematic_model

ROS2 node that simulates the kinematics of our unmanned surface vehicle with a kinematic model and publishes its position, velocity and odometry information.

#### twist_to_setpoint

ROS2 node that converts Twist messages which describe velocity and rotational motion in ROS2 into separate velocity and heading setpoint messages published to their corresponding topics.

#### usv_tf2_broadcaster

ROS2 node that listens to the position topic, transforms it into a TF2 transform message that describe the relationship between different coordinate frames in a system and passes it to the TF2 system. This converted value can then be used by other nodes for localization, navigation or visualization.

