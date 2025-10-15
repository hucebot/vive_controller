#!/bin/bash
set -e

# --- Build the ROS2 workspace ---
cd /ros2_ws
source /opt/ros/humble/setup.bash
echo "🔧 Building workspace..."
colcon build

# Source ROS2 environment + workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# --- Run selected node based on argument ---
if [ "$1" = "calibrate" ]; then
    echo "🚀 Running calibrate_workspace..."
    exec ros2 run ros2_vive_controller calibrate_workspace

elif [ "$1" = "visualize" ]; then
    echo "🔍 Running visualize_workspace..."
    exec python3 /ros2_ws/src/ros2_vive_controller/ros2_vive_controller/tools/visualize_workspace.py

elif [ "$1" = "joystick" ]; then
    echo "🎮 Running joystick_controller..."
    exec ros2 run ros2_vive_controller joystick_node
    
else
    echo "⚙️ No valid argument provided. Starting bash shell..."
    exec bash
fi
