# --- STAGE 1: Base (Nvidia + ROS2 + SteamCMD) ---
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04 AS base

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash

# 1. Core System & ROS 2 Repo
RUN apt-get update && apt-get install -y \
    curl ca-certificates gnupg2 lsb-release software-properties-common python3-pip \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# 2. Install ROS 2 Desktop & Build Tools
RUN apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-dev-tools \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 3. Setup SteamCMD (Requires i386)
RUN add-apt-repository -y multiverse && \
    dpkg --add-architecture i386 && \
    apt-get update && \
    echo "steam steam/question select I AGREE" | debconf-set-selections && \
    apt-get install -y steamcmd lib32gcc-s1 && \
    ln -sf /usr/games/steamcmd /usr/bin/steamcmd

# --- STAGE 2: Dependencies (Python + SteamVR) ---
FROM base AS dep

# 1. Install heavy Python dependencies
RUN pip install --no-cache-dir \
    numpy==1.23.5 scipy openvr OneEuroFilter transformations \
    opencv-contrib-python pyserial pyside6 moderngl moderngl-window \
    pyglm glfw pillow pygame pynput pyrr tqdm urdfpy trimesh \
    ping3 pyqtgraph mss networkx

# 2. Install System UI/OpenVR dependencies
RUN apt-get update && apt-get install -y \
    libpipewire-0.3-0 libpipewire-0.3-modules libspa-0.2-modules pipewire \
    libnss3 libatk1.0-0 libgtk-3-0 dbus-x11 terminator \
    libxkbcommon0 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

# 3. Install SteamVR (Requires ARG for build-time login)
ARG STEAM_USER
ARG STEAM_PASSWORD
RUN useradd -m steam && \
    chown -R steam:steam /home/steam
USER steam
RUN steamcmd +login ${STEAM_USER} ${STEAM_PASSWORD} +app_update 250820 validate +quit || true

USER root
# --- STAGE 3: Dev (The workspace for coding) ---
FROM dep AS dev
# --- CRITICAL FIX: Switch to Bash for this stage onwards ---
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws
# CycloneDDS & Networking
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY .ci/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Environment Setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# This stage keeps the container open for dev work (mounting handled by run_docker.py)
CMD ["/bin/bash"]

# --- STAGE 4: App (Standalone Deployment) ---
FROM dev AS app

# Copy local source into the container
COPY . /ros2_ws/src/ros2_vive_controller
WORKDIR /ros2_ws

# --- FIX: Ensure the resource marker exists and is a file ---
# This prevents the "can't copy: doesn't exist" error in setup.py
RUN mkdir -p /ros2_ws/src/ros2_vive_controller/resource && \
    touch /ros2_ws/src/ros2_vive_controller/resource/ros2_vive_controller

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

# Entrypoint is inherited from 'dev', so we just set the CMD
CMD ["ros2", "launch", "ros2_vive_controller", "vive_teleop.launch.py"]