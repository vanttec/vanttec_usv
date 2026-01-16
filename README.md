<p align="right">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/missions/docs/VantTec_logo_white.png" width="231" height="131" align="center"/>
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/missions/docs/VTEC S-IV.gif" width="131" height="131" align="left"/>
</p>

# VantTec USV Main Repository

This is the main working repository for the USV (Unmanned Surface Vehicle) VantTec Platform.
Official documentation [here][vanttec-documentation].


### Prerequisites
1. CUDA 12.8 Toolkit
2. CUDNN 8.9.7
3. TensorRT 10.3.0
4. ZED SDK
5. Gazebo Sim - Harmonic
6. Install the following dependencies:

```Shell
sudo add-apt-repository ppa:borglab/gtsam-release-5.1
sudo apt install libgtsam-dev libgtsam-unstable-dev ros-humble-xacro libpcap-dev ros-humble-robot-localization ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-vision-opencv ros-humble-tf-transformations ros-humble-foxglove-bridge ros-humble-nmea-msgs ros-humble-joy-teleop libgz-sim8 libgz-sim8-dev ros-humble-ros-gz ros-humble-ros-gzharmonic

# Install other missing dependencies automatically if needed:
rosdep install --from-paths src -y --ignore-src
```

7. Download the gz sim waves plugin
```Shell
# Install the plugin dependenciess
sudo apt-get update
sudo apt-get install libcgal-dev libfftw3-dev

# Create a ws for the plugin
cd
mkdir -p gz_ws/src

# Clone the repo
cd ~/gz_ws/src
git clone https://github.com/srmainwaring/asv_wave_sim.git

# Specify $GZ_VERSION before compiling
export GZ_VERSION=harmonic

# Compile it
colcon build --symlink-install --merge-install --cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DBUILD_TESTING=ON \
-DCMAKE_CXX_STANDARD=17

# Also build the GUI plugin
cd ~/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control 
mkdir build && cd build
cmake .. && make
```
8. Setup Gz Sim and the waves plugin, by adding these lines to the end of your ~/.bashrc file

```Shell
source ~/gz_ws/install/setup.bash

export GZ_VERSION=harmonic

export GZ_SIM_RESOURCE_PATH=:~/vanttec_usv/src/usv_description/models:$GZ_SIM_RESOURCE_PATH

# ensure the model and world files are found
export GZ_SIM_RESOURCE_PATH=\
$GZ_SIM_RESOURCE_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/worlds

# ensure the system plugins are found
export GZ_SIM_SYSTEM_PLUGIN_PATH=\
$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/gz_ws/install/lib

# ensure the gui plugin is found
export GZ_GUI_PLUGIN_PATH=\
$GZ_GUI_PLUGIN_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build
```

9. Download and build this workspace

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

10. Install fatrop for usv_control's MPC's functionalities
```Shell
# For the official, latest instructions, visit the fatrop repository

######

# Installing blasfeo from source 
cd
git clone https://github.com/giaf/blasfeo.git
cd blasfeo
mkdir build
cd build

# Configure project (if you have a different architecture, please refer to blasfeo's repository to view complete list of options)
cmake .. -DTARGET=X64_INTEL_CORE -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)

# Install blasfeo
sudo make install

######

# Installing fatrop
cd ~/vanttec_usv/src/usv_control/libs/fatrop/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install

######

# Install CASADI's dependency
sudo apt-get install swig

# Install CASADI
cd
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build
cd build
cmake .. \
    -DWITH_IPOPT=ON -DWITH_BUILD_IPOPT=ON \
    -DWITH_BUILD_MUMPS=ON -DWITH_BUILD_METIS=ON \
    -DWITH_FATROP=ON \
    -DWITH_PYTHON=ON -DWITH_PYTHON3=ON \
    -DPYTHON_PREFIX=$(python3 -c 'from distutils.sysconfig import get_python_lib; print(get_python_lib())') \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# Finally, go back to the fatrop dir. to build for CASADI
cd ~/vanttec_usv/src/usv_control/libs/fatrop/build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install

# Test the fatrop interface
cd ~/vanttec_usv/src/usv_control/libs/fatrop/examples/
python3 car_reference_tracking_example.py
```

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
