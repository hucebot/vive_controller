FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
ENV ROS_DISTRO=humble

ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

LABEL maintainer="clemente.donoso@inria.fr"

RUN apt-get update && apt-get upgrade -y

###### Install Python Dependencies
RUN apt-get install -y \
    python-is-python3 \
    python3-gst-1.0 \
    python3-pip

RUN pip install\
    v4l2py \
    opencv-contrib-python \
    pyserial \
    scipy \
    pyside6 \
    pyglet \
    moderngl \
    moderngl-window \
    pyglm \
    glfw \
    pillow \
    pygame \
    pynput \
    pyrr \
    tqdm \
    urdfpy \
    streamdeck \
    pillow \
    usd-core \
    trimesh \
    ping3 \
    pyqtgraph \
    mss

RUN pip install --upgrade networkx
RUN pip install --upgrade numpy==1.23.5

###### Install ROS2
RUN apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN apt install ros-humble-desktop-full -y
RUN apt install ros-dev-tools -y
RUN rosdep init && rosdep update

###### Install ROS Dependencies
RUN apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-imu-plugin

####### Install OpenVR
RUN git clone https://github.com/ChristophHaag/openvr.git
WORKDIR /openvr
RUN mkdir build
WORKDIR /openvr/build
RUN cmake -DCMAKE_BUILD_TYPE=Release ../
RUN make

###### Update Buffer
RUN sysctl net.ipv4.ipfrag_time=3
RUN sysctl net.ipv4.ipfrag_high_thresh=134217728

RUN echo "export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=1MB&soets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50" >> ~/.bashrc

###### Source ROS2
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
