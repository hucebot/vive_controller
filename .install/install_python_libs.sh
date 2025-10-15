#!/bin/bash
set -e

python3 -m pip install --upgrade pip

echo "==== Installing Python libraries ===="
pip install \
    v4l2py \
    opencv-contrib-python \
    pyserial \
    scipy \
    pyside6 \
    pyglet \
    moderngl \
    moderngl-window \
    pyglm \
    glfw \
    pillow \
    pygame \
    pynput \
    pyrr \
    tqdm \
    urdfpy \
    streamdeck \
    pillow \
    usd-core \
    trimesh \
    ping3 \
    pyqtgraph \
    mss \
    OneEuroFilter \
    transformations

pip install --upgrade networkx
pip install numpy==1.23.5
pip install openvr

echo "==== Python libraries installed ===="
