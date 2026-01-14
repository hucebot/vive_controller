#!/bin/sh
set -eu

STEAM_USER=""
STEAM_PASSWORD=""

# Parse --steam_user=... and --steam_password=...
while [ $# -gt 0 ]; do
  case "$1" in
    --steam_user=*)
      STEAM_USER="${1#*=}"
      ;;
    --steam_password=*)
      STEAM_PASSWORD="${1#*=}"
      ;;
    *)
      echo "Unknown argument: $1" >&2
      ;;
  esac
  shift
done

# Validate
if [ -z "$STEAM_USER" ] || [ -z "$STEAM_PASSWORD" ]; then
  echo "Error: You must pass both arguments:" >&2
  echo "  sh build_ros2.sh --steam_user=<username> --steam_password=<password>" >&2
  exit 1
fi

echo "Building Docker image with Steam user: $STEAM_USER"

docker build \
  --build-arg STEAM_USER="$STEAM_USER" \
  --build-arg STEAM_PASSWORD="$STEAM_PASSWORD" \
  -t ros1_vive_controller:steam \
  -f develROS1.Dockerfile .