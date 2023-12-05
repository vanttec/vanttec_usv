FROM osrf/ros:humble-desktop

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    python3-pip -y \
    --no-install-recommends terminator \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    ros-humble-controller-interface \
    ros-humble-realtime-tools \
    ros-humble-controller-manager \
    ros-humble-ackermann-msgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-common \
    ros-humble-nav2-bringup \
    ros-humble-rqt-tf-tree \
    ros-humble-tf2-tools \
    ros-humble-ros2-control \
    ros-humble-robot-localization \
    ros-humble-foxglove-bridge \
    ros-humble-diagnostic-updater \
    espeak \ 
    alsa-utils \ 
    software-properties-common \
    ffmpeg \ 
    bluez \ 
    portaudio19-dev \
    pulseaudio-module-bluetooth \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get -y install libgl1-mesa-glx libgl1-mesa-dri mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip install python-can
RUN pip install setuptools==58.2.0

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc