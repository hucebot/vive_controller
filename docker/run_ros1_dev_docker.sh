
isRunning=`docker ps -f name=ros1_vive_controller | grep -c "ros1_vive_controller"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm ros1_vive_controller
    docker run  \
        --name ros1_vive_controller  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v /run/udev:/run/udev \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        -v `pwd`/../ROS1/:/ros_ws/src/ros1_vive_controller \
        -v `pwd`/../config/:/ros_ws/src/ros1_vive_controller/config \
        -v `pwd`/../openvr_class/:/ros_ws/src/ros1_vive_controller/src/openvr_class \
        -v "$HOME/.steam/debian-installation/steamapps/common/SteamVR":"$HOME/.steam/debian-installation/steamapps/common/SteamVR" \
        -v "$HOME/.config/openvr":"/root/.config/openvr" \
        -w /ros_ws \
        ros1_vive_controller:latest

else
    echo "Docker already running."
    docker exec -it ros1_vive_controller /bin/bash
fi