<p align="center">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/release_candidate_v1_0/docs/LogoNegro_Azul.png" width="400" height="240" align="center"/>

</p>

# VantTec USV Main Repository

**How to start working?**

Enter the following commands into your **Ubuntu 16.04** terminal:

```Shell
cd
git clone http://github.com/vanttec/vanttec_usv.git
cd vanttec_usv
./init_worskpace.sh
```

**Make sure to have ROS installed, as well as the following dependencies:**

-- TODO: Dependencies

This is the main working repository for the USV (Unmanned Surface Vehicle) VantTec Platforms. Each directory represents a ROS Package:

- **arduino_br**: ROS package that uses rosserial_python and rosserial_arduino to interface with the T-100 and the T-200 thrusters.
- **rb_missions**: ROS package where the algorithms to solve the different RoboBoat challenges are implemented.
- **usv_comms**: ROS package that allows the USV software to interface with the Digi X-Tend Hardware for communication with the main station.
- **usv_control**: ROS package for the implementation of the control algorithms for the USV.
- **usv_master**: ROS package containing the master node for the USV ROS software.
- **usv_perception**: ROS package for the perception algorithms used in the USV.
- **vectornav_ros**: ROS package that allows the USV to interface with Vectornav's IMUs.
- **velodyne**: ROS package for the Velodyne LIDARs.
- **zed_ros_wrapper**: ROS package for the Stereolabs ZED Camera.
