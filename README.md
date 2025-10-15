# 3d_teleoperation_interface

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-green)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)
[![GitHub Stars](https://img.shields.io/github/stars/hucebot/vive_controller?style=social)](https://github.com/hucebot/vive_controller/stargazers)

| **Control Cartesio - RVIZ**                                | **Control Real Robot**                     | **Workspace**                   |
|-------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------|
| <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Cartesio" width="240"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Real Robot" width="240"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/workspace.gif" alt="Control Real Robot" width="240"> |


## Table of Contents
- [Get Started](#get-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
    - [From Docker](#from-docker)
- [Usage](#usage)
  - [Reference Frames](#reference-frames)
  - [Steam VR Instructions](#steam-vr-instructions)
    - [Install Steam](#install-steam)
    - [Connect Devices](#connect-devices)
  - [ROS](#ros)
    - [ROS1](#ros1)
      - [Calibrate Workspace](#calibrate-workspace)
      - [Visualize and generate the workspace](#visualize-and-generate-the-workspace)
      - [Run Joystick Node](#run-joystick-node)
    - [ROS2](#ros2)
      - [Calibrate Workspace](#calibrate-workspace-1)
      - [Visualize and generate the workspace](#visualize-and-generate-the-workspace-1)
      - [Run Joystick Node](#run-joystick-node-1)
      - [Tracker Node](#tracker-node)

# Get Started

## Prerequisites

The following dependencies are required to use this package:
- Docker
- NVIDIA Container Toolkit (if you are using an NVIDIA GPU)
- Steam account

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build_ros1.sh
```

```bash
sh build_ros2.sh --steam_user=USERNAME --steam_password=PASSWORD
```

To run the development container, use the following command:

```bash
sh run_ros1_dev_docker.sh
```

We implemented 3 entrypoints to make it easier to use the package. You can run the following commands:

- For calibrating the workspace:
```bash
sh run_ros2_dev_docker.sh calibrate
```

- For visualizing and generating the workspace:
```bash
sh run_ros2_dev_docker.sh visualize
```

- For running the joystick node:
```bash
sh run_ros2_dev_docker.sh joystick
```

### Using without Docker
If you plan to use the package without Docker, please follow the instructions in the [INSTRUCTIONS_WITHOUT_DOCKER.md](INSTRUCTIONS_WITHOUT_DOCKER.md) file.

## Usage

### Reference Frames
<img src="https://github.com/hucebot/vive_controller/blob/main/images/vive_axis.png" alt="Vive Controller Frames" width="700">

TODO

### Verify orientation of the controller

TODO