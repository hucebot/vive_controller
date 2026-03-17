#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

source "/opt/ros/humble/setup.bash"


if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
    # Optional: Log that the workspace was found
    # echo "✅ Workspace sourced."
fi

exec "$@"