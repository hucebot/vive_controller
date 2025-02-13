FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

###### Set environment variable
ENV DISPLAY=:0
ENV ROS_DISTRO=noetic
ENV LIBGL_ALWAYS_INDIRECT=0

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"

###### 1) Install prerequisites (lsb-release, curl, gnupg2)
RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    gnupg2

###### 2) Add ROS repository
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

###### 3) Add ROS keys and install ROS
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full

RUN apt-get install -yy \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

RUN rosdep init
RUN rosdep update

###### Install necessary packages
RUN apt-get update && apt-get install -y \
    python-is-python3 \
    python3-pip \
    git \
    gedit \
    terminator \
    python3-catkin-tools

###### Install necessary ROS packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-xacro

###### Upgrade pip & Install python packages
RUN python3 -m pip install --upgrade pip
RUN pip install openvr

###### Source the ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

###### Create and build a workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

###### Set ROS environment variables: Tiago wired
RUN echo "#export ROS_MASTER_URI=http://10.68.0.1:11311" >> /root/.bashrc
RUN echo "#export ROS_IP=10.68.0.128" >> /root/.bashrc
