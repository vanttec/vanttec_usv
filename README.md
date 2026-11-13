<p align="right">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/missions/docs/VantTec_logo_white.png" width="352" height="200" align="center"/>
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/missions/docs/VTEC S-IV.gif" width="200" height="200" align="left"/>
</p>

# VantTec USV Main Repository

This is the main working repository for the USV (Unmanned Surface Vehicle) VantTec Platform.
Official documentation [here][vanttec-documentation].


### Prerequisites
1. CUDA 12.8 Toolkit
2. CUDNN 8.9.7
3. TensorRT 10.3.0
4. ZED SDK
5. Install the following dependencies:

```Shell
sudo apt install libgtsam-dev libgtsam-unstable-dev 
sudo apt install ros-humble-xacro libpcap-dev ros-humble-robot-localization ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-vision-opencv ros-humble-tf-transformations ros-humble-foxglove-bridge ros-humble-nmea-msgs ros-humble-joy-teleop 

# Install other missing dependencies automatically if needed:
rosdep install --from-paths src -y --ignore-src
```

Note: This repository can be used without CUDA, CUDNN, TensorRT and ZED SDK if needed, but this requires skipping the visionsystemx package during building and will not allow for any vision functionalities.

### Downloading and building the workspace

```Shell
# Clone repository and its submodules
cd
git clone http://github.com/vanttec/vanttec_usv.git
cd vanttec_usv
git submodule update --init --recursive

# Build the usv_interfaces package first
colcon build --packages-select usv_interfaces

# Set environment variables with install/setup.bash file
source ./install/setup.bash

# Build the rest of the packages
colcon build
```
For use without cuda, the visionsystemx package can be skipped by using the following command instead: 
```
colcon build --packages-skip visionsystemx
```


Optional extra: 

- Install [ACADOS] for usv_control's MPC's functionalities
- Use [vanttec_usv_gz] for simulation of the USV

## Congratulations, you're ready to navigate the future @ VantTec!

<!-- ## HOW TOs (Pending: Modify for updated launch files):
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
``` -->

[vanttec-documentation]: https://vanttec-documentation.readthedocs.io/en/latest/usv_documentation.html
[ACADOS]: https://docs.acados.org/installation
[vanttec_usv_gz]: https://github.com/vanttec/vanttec_usv_gz