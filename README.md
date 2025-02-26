# 3d_teleoperation_interface

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-green)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-red)](
https://docs.ros.org/en/humble/index.html)

| Control Cartesio - RVIZ                                | Control Real Robot                     | Workspace                    |
|-------------------------------------------------------|-----------------------------------------------------|
| <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Cartesio" width="250"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Real Robot" width="250"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/workspace.gif" alt="Control Real Robot" width="250"> |


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
  - [Python API](#python-api)
  - [ROS](#ros)
    - [ROS1](#ros1)
    <!---- [ROS2](#ros2) -->


# Get Started

## Prerequisites

The following dependencies are required to use this package:
- Docker
- NVIDIA Container Toolkit (if you are using an NVIDIA GPU)
- SteamVR
- **Hardware**:
  - Vive Tracker (optional)
  - Vive Joystick (optional)
  - Vive Base Station (mandatory, at least one)

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build_ros1.sh
```

<!--- TODO
``bash
sh build_ros2.sh
```
-->
To run the development container, use the following command:

```bash
sh run_ros1_dev_docker.sh
```
<!--- TODO
```bash
sh run_ros2_dev_docker.sh
```
-->

# Usage

## Reference Frames
<img src="https://github.com/hucebot/vive_controller/blob/main/images/vive_axis.png" alt="Vive Controller Frames" width="700">
TODO

## Steam VR Instructions
### Install Steam
The easiest way to install Steam is to install through a command line. To do this, run the following command:
```bash
sudo apt install steam
```

Once Steam is installed, open it and log in with your account. If you don't have an account, you can create one for free. Then, go to the Steam Store and search for SteamVR. Install it.

### Connect Devices
Once SteamVR is installed, open it and go to the settings. In the settings, go the paired devices and pair your Vive Tracker and Vive Joystick. You can also pair your Vive Base Station if you have one.
Once the devices are paired, you will see a small window with the devices and their status. Make sure that all devices are connected. Finally, you can close SteamVR.


## Python API
To try the Python API, you can go to the python folder and run the following command:
```bash
python3 test_vive_controller.py
```
This will show the pose of the Vive Joystick in the terminal.

## ROS

### ROS1
TODO
```bash

```
<!--- TODO
### ROS2
TODO
```bash

```
-->

