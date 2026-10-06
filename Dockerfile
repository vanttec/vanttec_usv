FROM osrf/ros:humble-desktop

# -----------------------------------------------------------------------------
# Argumentos de build (se pueden sobreescribir desde docker-compose.yml)
# USER_UID/GID deben coincidir con tu usuario del host (`id -u`, `id -g`) para
# que build/, install/ y log/ queden con tu dueño en el volumen montado.
# -----------------------------------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG DEBIAN_FRONTEND=noninteractive

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    terminator \
    gdb \
    sudo \
    bash-completion \
    nano \
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
RUN pip install python-can setuptools==58.2.0

# -----------------------------------------------------------------------------
# Usuario no-root
# dialout: grupo dueño de /dev/ttyUSB* -> acceso al puerto serial del SBG.
# sudo sin contraseña: para instalar algo puntual sin rehacer la imagen.
# -----------------------------------------------------------------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    usermod -aG dialout ${USERNAME} && \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /ws

# .bashrc se lee en cada shell nuevo (incluido `docker compose exec`),
# a diferencia del ENTRYPOINT, que corre una sola vez al arrancar.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> ~/.bashrc && \
    echo "alias cb='colcon build --base-paths src src/deps --symlink-install'" >> ~/.bashrc && \
    echo "alias cclean='rm -rf /ws/build /ws/install /ws/log'" >> ~/.bashrc

CMD ["bash"]
