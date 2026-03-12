#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# 1. Source the base ROS 2 Humble installation
# This is always required for any ROS 2 container.
source "/opt/ros/humble/setup.bash"

# 2. Rebuild workspace if setup.py has changed (dev mode with volume mounts)
SETUP_FILE="/ros2_ws/src/ros2_vive_controller/setup.py"
SETUP_HASH_FILE="/ros2_ws/.setup_py_hash"
if [ -f "$SETUP_FILE" ]; then
    CURRENT_HASH=$(md5sum "$SETUP_FILE" | awk '{print $1}')
    STORED_HASH=""
    if [ -f "$SETUP_HASH_FILE" ]; then
        STORED_HASH=$(cat "$SETUP_HASH_FILE")
    fi
    if [ "$CURRENT_HASH" != "$STORED_HASH" ]; then
        echo "Detected setup.py change, rebuilding workspace..."
        cd /ros2_ws
        colcon build --symlink-install --packages-select ros2_vive_controller
        echo "$CURRENT_HASH" > "$SETUP_HASH_FILE"
        cd /
    fi
fi

# 3. Source the local workspace if it exists.
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

# 4. Setup steamclient symlinks and start vrserver (needed for Vive Trackers)
mkdir -p /root/.steam/sdk64 /root/.steam/sdk32
ln -sf /home/steam/.local/share/Steam/steamcmd/linux64/steamclient.so /root/.steam/sdk64/steamclient.so 2>/dev/null || true
ln -sf /home/steam/.local/share/Steam/steamcmd/linux32/steamclient.so /root/.steam/sdk32/steamclient.so 2>/dev/null || true
STEAMVR_PATH="/home/steam/Steam/steamapps/common/SteamVR"
if [ -d "$STEAMVR_PATH" ] && ! pgrep -x vrserver > /dev/null 2>&1; then
    export LD_LIBRARY_PATH="$STEAMVR_PATH/bin/linux64:${LD_LIBRARY_PATH}"
    echo "Starting SteamVR vrserver..."
    "$STEAMVR_PATH/bin/linux64/vrserver" --keepalive &
    sleep 3
    echo "SteamVR vrserver started."
fi

# 5. Environment Fallbacks
# We set these here as defaults, but the values passed by
# 'docker run -e ...' in your run_docker.py will OVERWRITE these.
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# 6. Execute the command passed to the container.
# Using 'exec' is critical: it makes the ROS process (like a launch file)
# become PID 1. This allows it to receive signals like SIGINT (Ctrl+C)
# so your save_and_exit() functions actually run.
# '$@' represents all arguments passed to the script.
exec "$@"