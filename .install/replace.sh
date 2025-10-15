#!/bin/bash
# -------------------------------------------------------------
# Copy custom SteamVR configuration files
#
# Usage:
#   ./copy_steamvr_configs.sh /path/to/steam
# Example:
#   ./copy_steamvr_configs.sh "$HOME/.steam/debian-installation"
# -------------------------------------------------------------

# Check argument
if [ -z "$1" ]; then
  echo "❌  Error: You must specify the base Steam directory."
  echo "Example: ./copy_steamvr_configs.sh \$HOME/.steam/debian-installation"
  exit 1
fi

STEAM_DIR="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/../steamvr_config"

# Source files
FILE_NULL="$SRC_DIR/default_driver_null"
FILE_RES="$SRC_DIR/default_resources"

# Destination paths
DEST_NULL="$STEAM_DIR/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings"
DEST_RES="$STEAM_DIR/steamapps/common/SteamVR/resources/settings/default.vrsettings"

# Validations
if [ ! -f "$FILE_NULL" ]; then
  echo "❌  Missing file: $FILE_NULL"
  exit 1
fi

if [ ! -f "$FILE_RES" ]; then
  echo "❌  Missing file: $FILE_RES"
  exit 1
fi

if [ ! -d "$STEAM_DIR/steamapps/common/SteamVR" ]; then
  echo "❌  SteamVR directory not found at: $STEAM_DIR/steamapps/common/SteamVR"
  exit 1
fi

# Create destination directories if needed
mkdir -p "$(dirname "$DEST_NULL")"
mkdir -p "$(dirname "$DEST_RES")"

# Copy files
cp -f "$FILE_NULL" "$DEST_NULL"
cp -f "$FILE_RES" "$DEST_RES"

# Confirmation
echo "✅ Files copied successfully:"
echo " - $FILE_NULL → $DEST_NULL"
echo " - $FILE_RES → $DEST_RES"
