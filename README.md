# 3d_teleoperation_interface

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-green)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-yellow)](
https://docs.ros.org/en/humble/index.html)

| **Control Cartesio - RVIZ**                                | **Control Real Robot**                     | **Workspace**                   |
|-------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------|
| <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Cartesio" width="240"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/test_rviz.gif" alt="Control Real Robot" width="240"> | <img src="https://github.com/hucebot/vive_controller/blob/main/images/workspace.gif" alt="Control Real Robot" width="240"> |


## Table of Contents
- [3d\_teleoperation\_interface](#3d_teleoperation_interface)
  - [Table of Contents](#table-of-contents)
- [Get Started](#get-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
    - [From Docker](#from-docker)
- [Usage](#usage)
  - [Reference Frames](#reference-frames)
  - [Steam VR Instructions](#steam-vr-instructions)
    - [Install Steam](#install-steam)
    - [Using without headset](#using-without-headset)
    - [Connect Devices](#connect-devices)
  - [ROS](#ros)
    - [ROS1](#ros1)
      - [Calibrate Workspace](#calibrate-workspace)
      - [Visualize and generate the workspace](#visualize-and-generate-the-workspace)
      - [Run Joystick Node](#run-joystick-node)
- [ROS 2 Usage](#ros-2-usage)
    - [1. Workspace Calibration](#1-workspace-calibration)
    - [2. Teleoperation](#2-teleoperation)
      - [SETUP DEFAULT DDS CONFIGURATION](#setup-default-dds-configuration)



# Get Started

## Prerequisites

The following dependencies are required to use this package:
- Docker
- NVIDIA Container Toolkit (if you are using an NVIDIA GPU)
- SteamVR (optional, you can use the joystick + cable)
- OpenVR
- **Hardware**:
  - Vive Tracker (optional)
  - Vive Joystick (optional)
  - Vive Base Station (mandatory, at least one)

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build_ros1.sh --steam_user=USERNAME --steam_password=PASSWORD
```

```bash
sh build_ros2.sh --steam_user=USERNAME --steam_password=PASSWORD
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
### Install Steam
The easiest way to install Steam is to install through a command line. To do this, run the following command:
```bash
sudo apt install steam
```

Once Steam is installed, open it and log in with your account. If you don't have an account, you can create one for free. Then, go to the Steam Store and search for SteamVR. Install it.
Then you need to RUN the BETA Version.

### Using without headset

Basically you need to edit the following files:
- **default.vrsettings** located in **Steam Directory**/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings
  - On the null driver secction replace: **"enable": false** to **"enable": true**
- **default.vrsettings** located in **Steam Directory**/steamapps/common/SteamVR/resources/settings/default.vrsettings
  - On the steamvr section replace: **"requireHmd": true** to **"requireHmd": false**
  - On the steamvr section replace: **"forcedDriver": ""** to **"forcedDriver": "null"**
  - On the steamvr section replace: **"activateMultipleDrivers": false** to **"activateMultipleDrivers": true**

We prepare command that already do this. Move to the folder `Docker` and the run the following command:
```bash
sh replace.sh path-where-steam-is-installed #For example: sh replace.sh ~/.steam/debian-installation
```

### Connect Devices
Once SteamVR is installed, open it and go to the settings. In the settings, go the paired devices and pair your Vive Tracker and Vive Joystick. Once the devices are paired, you will see a small window with the devices and their status. Make sure that all devices are connected. Finally, you can close SteamVR.

## ROS

### ROS1

#### Calibrate Workspace

It is important to calibrate the workspace before using the joystick. By calibrating the workspace, you define the limites of the tracking area. To calibrate the workspace, run the following command:

```bash
rosrun ros1_vive_controller calibrate_workspace.py
```

The idea is to move the joystick in every direction, with this you will create a PointCloud that represents the workspace. The recomendations it's to use RVIZ to visualize
this point cloud over the topic `/workspace_pointcloud`, in this way it will be more easly to see the limits of the workspace.


#### Visualize and generate the workspace

Once the workspace is calibrated, you can visualize the representation of it, the convexhull and the bounding box representing the workspace ( After removing the outliers). If you feel that you should remove more outliers you can change the parameter `z_threshold` on the config file.

To visualize and generate the workspace, run the following command:
```bash
cd /ros_ws/src/ros1_vive_controller/tools && python visualize_workspace.py
```

#### Run Joystick Node

Finally, you can run the joystick node. With the workspace calibrated, you will fell a vibration if you are near the limits of the workspace (By default is setup to 20 cm, but can be modified on the config file). To run the joystick node, run the following command:

```bash
rosrun ros1_vive_controller joystick_node.py
```

# ROS 2 Usage

The ROS 2 package `ros2_vive_controller` features a modern, unified workflow.

### 1. Workspace Calibration

Before teleoperating, you must define the "Safe Workspace". This prevents the robot from crashing into physical walls by limiting the VR controller's effective range.

We use a **Unified Calibration Node** that visualizes the process live in RViz.

**Launch:**

```bash
ros2 launch ros2_vive_controller calibration.launch.py

```

**Instructions:**

1. **Check Tracking:** Look at RViz. You should see a **RED Cube** following your controller.
2. **Paint Bounds:** Walk to the corners of your safe area. Hold the **Trigger** to record points.
* *Visual:* The cursor turns **GREEN**, and a point cloud appears.
* *Feedback:* A green bounding box will grow in real-time to encompass your points.


3. **Save:** Press `Ctrl+C` in the terminal.
* The node automatically filters outliers (z-score), calculates the new limits, and updates `config/config.yaml`.



### 2. Teleoperation

Once calibrated, run the main joystick node to control the robot.

**Run:**

```bash
ros2 run ros2_vive_controller joystick_node

```

**Features:**

* **Deadman Switch:** Hold the Trigger to engage movement (Clutch). Release to reposition your hand without moving the robot.
* **Safety Walls:** If you reach the edge of the calibrated workspace, the controller vibrates and motion stops.
* **Smoothing:** Position data is filtered using a OneEuro filter to remove jitter.

#### SETUP DEFAULT DDS CONFIGURATION
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp




