# 3d_teleoperation_interface

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-green)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)

| Control Cartesio - RVIZ                                | Control Real Robot                     |
|-------------------------------------------------------|-----------------------------------------------------|
| <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Cartesio" width="400"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Real Robot" width="400"> |


## Table of Contents
- [Get Started](#get-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
    - [From Docker](#from-docker)
- [Usage](#usage)


# Get Started

## Prerequisites

The following dependencies are required to use this package:
- Docker
- NVIDIA Container Toolkit (if you are using an NVIDIA GPU)
- SteamVR
- Vive Tracker
- Vive Joystick
- Vive Base Station

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build_ros1.sh
```

```bash
sh build_ros2.sh
```

To run the development container, use the following command:

```bash
sh run_ros1_dev_docker.sh
```
```bash
sh run_ros2_dev_docker.sh
```

# Usage

## Reference Frames
<img src="https://github.com/hucebot/vive_controller/blob/main/images/vive_axis.png" alt="Vive Controller Frames" width="700">
TODO

## Steam VR Instructions
TODO

## Connect to the Vive Joystick
TODO

## Connect to the Vive Tracker
TODO

## ROS LAUNCH

### ROS1
TODO
```bash

```

### ROS2
TODO
```bash

```

