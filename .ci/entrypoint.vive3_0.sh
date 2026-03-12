#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

# Rebuild ros2_vive_controller if setup.py has changed (e.g. new entry points)
SETUP_FILE="/ros2_ws/src/ros2_vive_controller/setup.py"
SETUP_HASH_FILE="/ros2_ws/.setup_py_hash"
if [ -f "$SETUP_FILE" ]; then
    CURRENT_HASH=$(md5sum "$SETUP_FILE" | awk '{print $1}')
    STORED_HASH=""
    [ -f "$SETUP_HASH_FILE" ] && STORED_HASH=$(cat "$SETUP_HASH_FILE")
    if [ "$CURRENT_HASH" != "$STORED_HASH" ]; then
        echo "Detected setup.py change, rebuilding ros2_vive_controller..."
        cd /ros2_ws
        colcon build --symlink-install --packages-select ros2_vive_controller
        echo "$CURRENT_HASH" > "$SETUP_HASH_FILE"
    fi
fi

if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# Setup steamclient symlinks
mkdir -p /root/.steam/sdk64 /root/.steam/sdk32
ln -sf /home/steam/.local/share/Steam/steamcmd/linux64/steamclient.so /root/.steam/sdk64/steamclient.so
ln -sf /home/steam/.local/share/Steam/steamcmd/linux32/steamclient.so /root/.steam/sdk32/steamclient.so

# Start vrserver directly (no compositor/monitor needed for tracker-only use)
STEAMVR_PATH="/home/steam/Steam/steamapps/common/SteamVR"
export LD_LIBRARY_PATH="$STEAMVR_PATH/bin/linux64:${LD_LIBRARY_PATH}"

if [ -d "$STEAMVR_PATH" ]; then
    echo "Starting SteamVR vrserver..."
    "$STEAMVR_PATH/bin/linux64/vrserver" --keepalive &
    sleep 3
    echo "SteamVR vrserver started."
fi

exec "$@"
