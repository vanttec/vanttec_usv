<p align="right">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/humble/docs/VantTec_logo_white.png" width="231" height="131" align="center"/>
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/humble/docs/USV_sticker.png" width="131" height="131" align="left"/>
</p>

# VantTec USV Main Repository

This is the main working repository for the USV (Unmanned Surface Vehicle) VantTec Platform.

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

```Shell
# Build ROS 2 packages
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

# (Pull and) Run docker image ->
# Jetson TX2
sudo docker run --hostname vtec --name usv -v ~/vanttec_usv/:/ws/ --device=/dev/ttyUSB0 -it maxpacheco02/vanttec_usv:rb25

# Jetson Orin Nano
sudo docker run --hostname vtec --name usv -v ~/vanttec_usv/:/ws/ -it my_container:l4t-r36.3.0

sudo docker start usv
sudo docker attach usv
sudo docker exec -it usv bash
```


**NOTE:** Add the *scripts* repository to the *${PATH}* variable in ~/.bashrc
```Shell
export PATH="$PATH":~/vanttec_usv/scripts
```

### Requirements to build the workspace

```
# Go to repository root
cd {vanttec_usv path}

# Build usv_interfaces package
colcon build --packages-select usv_interfaces

# Set environment variables with install/setup.bash file
source install/setup.bash

# Create record about submodules in .git/config file in workspace root
git submodule init
git submodule update

# Change directory to usv_libs from usv_control package and create submodules
cd vanttec_usv/src/usv_control/libs/usv_libs
git submodule init
git submodule update

- Nvidia CUDA
- ZED SDK
- Gazebo Sim Garden
- TensorRT
  - https://docs.nvidia.com/deeplearning/tensorrt/quick-start-guide/index.html

- Other dependencies specified in errors while building

```

## TODO: Modify and do documentation of: ROS2_XBEE_BRIDGE's changes. Also, add to repo as submodule

1. Change config file to:
- ['xbee0', '0013A20041CF8FE6', 'STATION_XBEE']
- ['xbee1', '0013A20041CF8F96', 'BOAT_XBEE']

###

2. Do launch files for

**Network 1: (USV)**

export ROS_DOMAIN_ID=50; ros2 launch ros2_xbee_bridge xbee_bridge.launch.py dev:=/dev/ttyUSB0 namespace:=xbee0

export ROS_DOMAIN_ID=50; ros2 topic pub /xbee1/msg1 std_msgs/msg/String "data: 'Hello from id 50 huhh'"

**Network 2: (OGS)**

export ROS_DOMAIN_ID=51; ros2 launch ros2_xbee_bridge xbee_bridge.launch.py dev:=/dev/ttyUSB1 namespace:=xbee1

export ROS_DOMAIN_ID=51; ros2 topic echo /intra_comms/xbee0/msg1 std_msgs/msg/String

**NOTE:** If found similar issue to "xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">" while running topic echo:
Just restart ros daemon


**NOTE:** Extra dependencies: 
```Shell
# To install dependencies automatically:
rosdep install --from-paths src -y --ignore-src

sudo apt-get install libpcap-dev libgeographic-dev ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-vision-opencv ros-humble-xacro ros-humble-tf-transformations

sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## HOW TOs:
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
#
