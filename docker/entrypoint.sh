#!/bin/bash
set -e

# --- Build the ROS2 workspace ---
cd /ros2_ws
echo "🔧 Building workspace..."
colcon build --symlink-install --packages-select ros2_vive_controller

# Source ROS2 environment + workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# --- Run selected node based on argument ---
if [ "$1" = "calibrate" ]; then
    echo "🚀 Running calibrate_workspace..."
    exec ros2 run ros2_vive_controller calibrate_workspace
elif [ "$1" = "joystick" ]; then
    echo "🎮 Running joystick_controller..."
    exec ros2 run ros2_vive_controller joystick_controller
else
    echo "⚙️ No valid argument provided. Starting bash shell..."
    exec bash
fi
