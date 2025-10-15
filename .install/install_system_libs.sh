#!/bin/bash
set -e

echo "=============================="
echo " Installing System Libraries "
echo "=============================="

export DEBIAN_FRONTEND=noninteractive
export TZ="Europe/Paris"

echo ">>> Updating system packages..."
sudo apt-get update && sudo apt-get upgrade -y

echo ">>> Installing base system dependencies..."
sudo apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python-is-python3 \
    python3-gst-1.0 \
    dbus-x11 \
    libxkbcommon0 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    libnss3 \
    libatk1.0-0 \
    libgtk-3-0 \
    libpipewire-0.3-0 \
    libpipewire-0.3-modules \
    libspa-0.2-modules \
    pipewire \
    terminator \
    git \
    build-essential \
    cmake \
    pkg-config \
    unzip

echo ">>> Adjusting network buffer parameters..."
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

echo ">>> System libraries installed successfully!"
