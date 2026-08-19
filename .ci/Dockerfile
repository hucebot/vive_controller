# --- STAGE 1: Base (Nvidia + ROS2 + SteamCMD) ---
FROM registry.gitlab.inria.fr/eurobin-horizon/code/vive_controller_tiago/vive-controller:latest as dep
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