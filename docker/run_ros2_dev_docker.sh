#!/bin/bash

COMMAND=$1

isRunning=`docker ps -f name=ros2_vive_controller | grep -c "ros2_vive_controller"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm ros2_vive_controller
    docker run  \
        --name ros2_vive_controller  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --ipc host \
        --pid host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v /run/udev:/run/udev \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        -v `pwd`/../ROS2/:/ros2_ws/src/ros2_vive_controller \
        -v `pwd`/../config/:/ros2_ws/src/ros2_vive_controller/config \
        -v `pwd`/../openvr_class/:/ros2_ws/src/ros2_vive_controller/ros2_vive_controller/openvr_class \
        -v `pwd`/../steamvr_config/openvrpaths.vrpath:/root/.config/openvr/openvrpaths.vrpath \
        -v `pwd`/../steamvr_config/default_driver_null:/home/steam/Steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings \
        -v `pwd`/../steamvr_config/default_resources:/home/steam/Steam/steamapps/common/SteamVR/resources/settings/default.vrsettings \
        -w /ros2_ws \
        ros2_vive_controller:steam $COMMAND

else
    echo "Docker already running."
    if [ -z "$COMMAND" ]; then
        docker exec -it ros2_vive_controller /bin/bash
    else
        docker exec -it ros2_vive_controller /ros_entrypoint.sh $COMMAND
    fi
fi