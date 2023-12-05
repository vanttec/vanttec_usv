#!/bin/bash

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Variables for forwarding ssh agent into docker container
SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

# Settings required for having intel integrated graphics acceleration inside the docker
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/dri:/dev/dri --device /dev/snd"

DOCKER_COMMAND="docker run"

xhost +

$DOCKER_COMMAND -it -d\
    $DOCKER_USER_ARGS \
    $DOCKER_GPU_ARGS \
    $DOCKER_SSH_AUTH_ARGS \
    $DOCKER_NETWORK_ARGS \
    --privileged \
    -v /dev/bus/usb/:/dev/bus/usb \
    -v /dev:/dev \
    -v "$PWD/src:/ws/src" \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /run/dbus:/run/dbus:ro \
    -v /etc/asound.conf:/etc/asound.conf:ro \
    -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
    --name=usv \
    usv\
    bash

# -v "/media/saveasmtz/pacman/rosbags:/ws/src/tests" \
#-v /usr/lib/x86_64-linux-gnu/alsa-lib:/usr/lib/x86_64-linux-gnu/alsa-lib:ro \