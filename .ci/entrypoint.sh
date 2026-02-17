#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# 1. Source the base ROS 2 Humble installation
# This is always required for any ROS 2 container.
source "/opt/ros/humble/setup.bash"

# 2. Source the local workspace if it exists.
# We check the /install folder which is created after 'colcon build'.
# This works for both the built 'app' image and the 'dev' mount.
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
    # Optional: Log that the workspace was found
    # echo "✅ Workspace sourced."
fi

# 3. Environment Fallbacks
# We set these here as defaults, but the values passed by
# 'docker run -e ...' in your run_docker.py will OVERWRITE these.
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# 4. Execute the command passed to the container.
# Using 'exec' is critical: it makes the ROS process (like a launch file)
# become PID 1. This allows it to receive signals like SIGINT (Ctrl+C)
# so your save_and_exit() functions actually run.
# '$@' represents all arguments passed to the script.
exec "$@"