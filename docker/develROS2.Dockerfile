FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
ENV ROS_DISTRO=humble

ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

ARG STEAM_USER
ARG STEAM_PASSWORD

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
    ros-${ROS_DISTRO}-rviz2

###### Upgrade pip & Install python packages
RUN python3 -m pip install --upgrade pip
RUN pip install openvr
RUN pip install scipy
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  libpipewire-0.3-0 libpipewire-0.3-modules libspa-0.2-modules pipewire \
  dbus-x11 \
  libxkbcommon0 libxrandr2 libxinerama1 libxcursor1 libxi6 \
  libnss3 libatk1.0-0 libgtk-3-0

###### Update Buffer
RUN sysctl net.ipv4.ipfrag_time=3
RUN sysctl net.ipv4.ipfrag_high_thresh=134217728

RUN echo "export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=1MB&soets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50" >> ~/.bashrc

###### Install streamcmd
RUN echo "steam steam/question select I AGREE" | debconf-set-selections
RUN echo "steam steam/license note ''" | debconf-set-selections

RUN apt-get update && \
    apt-get install -y software-properties-common

RUN add-apt-repository -y multiverse
RUN dpkg --add-architecture i386

RUN apt-get update && \
    apt-get install -y steamcmd lib32gcc-s1 && \
    ln -sf /usr/games/steamcmd /usr/bin/steamcmd


RUN useradd -m -s /bin/bash steam || true
RUN mkdir -p /home/steam/.steam /home/steam/.local/share/Steam /home/steam/Steam
RUN chown -R steam:steam /home/steam

###### Install SteamVR
USER steam
RUN steamcmd +login ${STEAM_USER} ${STEAM_PASSWORD} +app_update 250820 validate +quit || true
USER root

RUN pip install OneEuroFilter --upgrade
RUN pip install transformations
RUN apt-get install terminator -y


RUN echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc

###### Source ROS2
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc