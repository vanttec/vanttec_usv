<p align="right">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/humble/docs/VantTec_logo_white.png" width="231" height="131" align="center"/>
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/humble/docs/USV_sticker.png" width="131" height="131" align="left"/>
</p>

# VantTec USV Main Repository

This is the main working repository for the USV (Unmanned Surface Vehicle) VantTec Platform.
Official documentation [here][vanttec-documentation].


### Packages
- **usv_comms**: ROS package that allows the USV software to interface with the Digi XBee Hardware for communication with the ground control station.
- **usv_control**: ROS package for the implementation of the control algorithms for the USV.
- **usv_missions**: ROS package where the algorithms to solve the different RoboBoat 2024 challenges are implemented.
- **usv_perception**: ROS package for the perception algorithms used in the USV.
### Submodules
- **sbg_driver**: ROS package that allows the USV to interface with SBG's IMU.
- **usv_libs**: Control library.
- **velodyne**: ROS package for the Velodyne LIDAR.
- **zed_ros_wrapper**: ROS package for the Stereolabs ZED Camera.


### Needed Dependencies
- Nvidia CUDA
- ZED SDK
- Gazebo Sim - Garden
- TensorRT
- The following dependencies:

```Shell
# To install dependencies automatically:
rosdep install --from-paths src -y --ignore-src

sudo apt-get install libpcap-dev libgeographic-dev ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-vision-opencv ros-humble-xacro ros-humble-tf-transformations libgz-sim7-dev libignition-transport12-dev libignition-msgs9-dev python3-sdformat13 ros-humble-diagnostic-updater ros-humble-geographic-msgs ros-humble-nmea-msgs ros-humble-robot-localization

sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
```


### How to start working?

Enter the following commands into your **Ubuntu 22** terminal:

```Shell
# Clone repository and its submodules
cd
git clone http://github.com/vanttec/vanttec_usv.git
cd vanttec_usv
git submodule update --init --recursive

# Build the usv_interfaces package
colcon build --packages-select usv_interfaces

# Set environment variables with install/setup.bash file
source install/setup.bash
```


## HOW TOs (Pending: Modify for updated launch files):
**Run mission #2:** 
```Shell
ros2 launch usv_control usv_control_sim_launch.py
ros2 launch usv_missions obstacle_launch.py
ros2 run usv_utils obstacle_viewer_node
ros2 run usv_control obstacle_avoidance_node
ros2 run usv_missions mission_handler_node 
```

#

**Move the boat around:** 
```Shell
ros2 launch usv_control usv_control_sim_launch.py
ros2 launch usv_control teleop_launch.py 
```

[vanttec-documentation]: https://vanttec-documentation.readthedocs.io/en/latest/usv_documentation.html
