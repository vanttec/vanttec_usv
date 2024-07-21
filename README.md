<p align="right">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/humble/docs/VantTec_logo_white.png" width="231" height="131" align="center"/>
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/humble/docs/USV_sticker.png" width="131" height="131" align="left"/>
</p>

# VantTec USV Main Repository

This is the main working repository for the USV (Unmanned Surface Vehicle) VantTec Platforms.

### Packages
- **usv_comms**: ROS package that allows the USV software to interface with the Digi XBee Hardware for communication with the ground control station.
- **usv_control**: ROS package for the implementation of the control algorithms for the USV.
- **usv_master**: ROS package containing the master node for the USV ROS software.
- **usv_missions**: ROS package where the algorithms to solve the different RoboBoat 2024 challenges are implemented.
- **usv_perception**: ROS package for the perception algorithms used in the USV.
### Submodules
- **sbg_driver**: ROS package that allows the USV to interface with SBG's IMU.
- **usv_libs**: Control library.
- **velodyne**: ROS package for the Velodyne LIDAR.
- **zed_ros_wrapper**: ROS package for the Stereolabs ZED Camera.

### How to start working?

Enter the following commands into your **Ubuntu** terminal:

```Shell
# Clone repository and its submodules
cd
git clone --recurse-submodules http://github.com/vanttec/vanttec_usv.git

# Build docker container
cd ~/vanttec_usv
docker build -t vtec_rb25 .

# Run container
usv start 
```

**NOTE:** Add the *scripts* repository to the *${PATH}* variable in ~/.bashrc
```
export PATH="$PATH":~/vanttec_usv/scripts
```

### Requirements to build the workspace
- Nvidia CUDA
- ZED SDK
- Gazebo Sim Garden
- TensorRT
  - https://docs.nvidia.com/deeplearning/tensorrt/quick-start-guide/index.html

- Other dependencies specified in errors while building