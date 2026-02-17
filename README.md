# ROS 2 Vive Controller

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-yellow)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)

A unified ROS 2 driver and teleoperation bridge for HTC Vive hardware (Trackers, Controllers, and Base Stations). This package provides a standalone Dockerized workflow for workspace calibration and robot teleoperation.

|                        **Teleoperation**                         |                       **Workspace Calibration**                       |
| :--------------------------------------------------------------: | :-------------------------------------------------------------------: |
| <img src="assets/images/test_rviz.gif" alt="Teleop" width="300"> | <img src="assets/images/workspace.gif" alt="Calibration" width="300"> |

---

## Table of Contents
- [ROS 2 Vive Controller](#ros-2-vive-controller)
  - [Table of Contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
    - [Hardware](#hardware)
    - [Software](#software)
  - [Installation \& Docker](#installation--docker)
    - [1. Building the Image](#1-building-the-image)
    - [2. Running the Container](#2-running-the-container)
  - [⚙️ The Vive Driver Node](#️-the-vive-driver-node)
    - [Key Features](#key-features)
  - [🚀 Launching the Hardware](#-launching-the-hardware)
    - [Usage](#usage)
    - [Advanced Launch Configuration](#advanced-launch-configuration)
    - [📊 Data Outputs](#-data-outputs)
      - [1. Tracking Data](#1-tracking-data)
      - [2. Button Mapping (`joint_states`)](#2-button-mapping-joint_states)
  - [🛠️ The Teleop Bridge Node](#️-the-teleop-bridge-node)
    - [Core Logic: Hybrid Relative/Absolute Positioning](#core-logic-hybrid-relativeabsolute-positioning)
    - [Reference Frames \& Alignment](#reference-frames--alignment)
    - [Key Features](#key-features-1)
  - [🤖 Robot Launch Configurations](#-robot-launch-configurations)
    - [1. TIAGo (Mobile Manipulator)](#1-tiago-mobile-manipulator)
    - [2. Unitree G1 (Humanoid)](#2-unitree-g1-humanoid)
  - [⌨️ Usage](#️-usage)

---

## Prerequisites

### Hardware
* **HTC Vive Base Stations** (Lighthouse): At least one is mandatory for tracking.
* **HTC Vive Controller** or **Tracker**: For teleoperation and calibration.
* **SteamVR Compatible Dongle** (or HMD): Required to connect the devices to the PC.

### Software
* **Linux** (Ubuntu 22.04 recommended)
* **Docker** & **NVIDIA Container Toolkit**
* **Python 3** (for build scripts)
* **Steam Account** (Username and Password required for headless SteamVR installation)

---

## Installation & Docker

This project uses a **multi-stage Docker build** to separate the heavy SteamVR dependencies from your daily development code. You do not need to install ROS 2 or SteamVR on your host machine.

### 1. Building the Image
We provide a Python script to handle the multi-stage build arguments automatically.

1. **Export your Steam Credentials** (Required to download the headless SteamVR server):
  ```bash
   export STEAM_USER="your_username"
   export STEAM_PASSWORD="your_password"
  ```
2. **Run the Build Script**:
  ```bash
  # Builds the development image (mounts local code)
  python3 scripts/build_docker.py --target dev
  ```



### 2. Running the Container

The `run_docker.py` script handles GPU passthrough, USB device mapping, and ROS 2 network configuration (`ROS_DOMAIN_ID`). It also provides **aliases** for common tasks.

**Basic Usage:**

```bash
python3 scripts/run_docker.py [COMMAND] --domain [ID]

```

**Available Commands:**
| Command     | Description                                               |
| :---------- | :-------------------------------------------------------- |
| `calibrate` | Launches the workspace calibration tool with RViz.        |
| `teleop`    | Launches the main teleoperation bridge (driver + bridge). |
| `g1`        | Launches the G1 Humanoid dual-arm teleop configuration.   |
| `tiago`     | Launches the Tiago dual-arm teleop configuration.         |
| *(empty)*   | Starts a generic bash shell inside the container.         |

**Example:**

```bash
# Run teleoperation on Domain ID 1
python3 scripts/run_docker.py teleop --domain 1

```

---

## ⚙️ The Vive Driver Node

The driver handles three critical tasks:

1. **Haptic Safety (The Virtual Fence):** It monitors the controller's position relative to the calibrated `workspace` parameters. If the controller enters the "padding" zone near a wall, it triggers a haptic vibration pulse (2ms) to warn the user.
2. **Jitter Reduction:** It uses a **OneEuro Filter** to smooth out the tracking data, significantly reducing high-frequency jitter in the robot's motion.
3. **Haptic & Logic Separation:** It treats the hardware as a `JointState` source (buttons) and a `PoseStamped` source (tracking).

### Key Features

* **OneEuro Filter:** Balances low-latency response with high-speed smoothing using three parameters: `mincutoff`, `beta`, and `dcutoff`.
* **Workspace Markers:** Publishes a semi-transparent red cube to RViz representing the "Safe Zone" defined during calibration.
* **Serialized Hardware:** Binds to specific controllers using their unique hardware serial numbers (e.g., `LHR-4BB3817E`).

---

## 🚀 Launching the Hardware

The driver is typically launched via `vive_teleop.launch.py`. This launch file is designed to be robust against OpenVR initialization race conditions.

### Usage

```bash
# Run with default serials
python3 scripts/run_docker.py teleop

# Run with specific hardware serials
python3 scripts/run_docker.py teleop --serial_right "LHR-12345678"

```

### Advanced Launch Configuration

The launch file includes a **2-second TimerAction delay** between starting the left and right controller drivers. This prevents Inter-Process Communication (IPC) conflicts within the OpenVR runtime when multiple nodes attempt to initialize the driver simultaneously.

| Argument             | Default        | Description                                           |
| -------------------- | -------------- | ----------------------------------------------------- |
| `rviz`               | `true`         | Automatically launches RViz with a predefined config. |
| `serial_left`        | `LHR-97752221` | Hardware ID for the left-hand controller.             |
| `serial_right`       | `LHR-4BB3817E` | Hardware ID for the right-hand controller.            |
| `tracking_reference` | `LHB-DFA5BD2C` | The Base Station used as the world origin.            |
I have added a detailed breakdown of the `joint_states` topic to the **Data Outputs** section. This explains exactly which index corresponds to which physical button, matching the code you provided.


---

### 📊 Data Outputs

Each driver node (Left/Right) publishes to its own namespace.

#### 1. Tracking Data

| Topic                        | Type                        | Description                              |
| ---------------------------- | --------------------------- | ---------------------------------------- |
| `vive/left/pose`             | `geometry_msgs/PoseStamped` | Filtered 6-DOF position and orientation. |
| `vive/left/workspace_marker` | `visualization_msgs/Marker` | The visual boundary box in RViz.         |

#### 2. Button Mapping (`joint_states`)

The driver publishes button inputs as a `sensor_msgs/JointState` message to `vive/left/joint_states`. The `position` array contains the values for the following keys:

| Index | Name               | Type    | Range          | Description                                        |
| ----- | ------------------ | ------- | -------------- | -------------------------------------------------- |
| **0** | `trigger`          | Analog  | `0.0` - `1.0`  | The index finger trigger. Used for the **Clutch**. |
| **1** | `trackpad_x`       | Analog  | `-1.0` - `1.0` | Horizontal touch position on the round pad.        |
| **2** | `trackpad_y`       | Analog  | `-1.0` - `1.0` | Vertical touch position on the round pad.          |
| **3** | `grip`             | Digital | `0.0` / `1.0`  | The side grip buttons (squeezing the handle).      |
| **4** | `menu`             | Digital | `0.0` / `1.0`  | The small button above the trackpad.               |
| **5** | `trackpad_touched` | Digital | `0.0` / `1.0`  | True if the thumb is touching the pad.             |
| **6** | `trackpad_pressed` | Digital | `0.0` / `1.0`  | True if the trackpad is physically clicked down.   |



| Vive Button Map                                                           |
| -------------------------------------------------------------------------------- |
| <img src="assets/images/vive_button_map.png" alt="Vive Controller Frames" width="200"> |

> **Note:** Digital buttons are published as floats (`0.0` for False, `1.0` for True) to maintain consistency within the `JointState` message standard.

---
## 🛠️ The Teleop Bridge Node

This node implements a **Clutch Mechanism** (Deadman Switch) to allow for safe, intuitive teleoperation by separating translation and rotation logic.

### Core Logic: Hybrid Relative/Absolute Positioning

To provide the most intuitive experience for the operator, the bridge treats position and orientation differently:

1. **Relative Translation (The Mouse Metaphor):** Position is calculated as a delta () from the moment the clutch is engaged. This allows you to "ratchet" the robot's position, moving it large distances through multiple small controller strokes.
2. **Absolute Orientation (The Mirror Metaphor):** Rotation is **not relative**. For intuitive control, the robot's end-effector orientation is mapped to match the controller's orientation directly. This ensures that if you tilt the controller 45°, the robot hand tilts 45°, maintaining a consistent mental map for the operator.

### Reference Frames & Alignment

To ensure the robot moves in the direction you expect, the controller's internal axes must be understood. The image below shows the coordinate system of the Vive Controller used by the driver:

| Vive Coordinate System                                                           |
| -------------------------------------------------------------------------------- |
| <img src="assets/images/vive_axis.png" alt="Vive Controller Frames" width="200"> |

* **Z-axis:** Points "out" from the controller tip.
* **X-axis:** Points to the right side of the controller.
* **Y-axis:** Points "up" through the trackpad.

**Teleoperation Workflow:**

1. **Idle State:** The bridge ignores controller movement.
2. **Activation (Clutch):** Press the `trigger`. The node saves the current EE position as a reference point.
3. **Execution:** Move the controller. The robot translates based on your hand's displacement and rotates to mirror your hand's orientation.
4. **Repositioning:** Release the trigger. Your hand can move freely while the robot stays locked in its current pose.

### Key Features

* **PointStamped Button Mapping:** All analog and digital button data is published as `geometry_msgs/PointStamped` (with the value in the `.x` field). This allows for high-compatibility with diverse robot control stacks.
* **TF2 Integration:** Uses standard ROS 2 transform lookups to anchor movement to the robot's coordinate system (e.g., `base_link` or `pelvis`).
* **Frequency Control:** Can be set to a fixed rate (e.g., 30 Hz) or event-driven mode (`-1.0`) where it publishes only when new data arrives.

---

## 🤖 Robot Launch Configurations

The package includes pre-configured launch files for complex platforms, demonstrating how to remap topics and frames for specific hardware.

### 1. TIAGo (Mobile Manipulator)

Designed for the dual-arm TIAGo robot. It maps VR buttons to the TIAGo gripper actions.

* **Reference Frame:** `base_link`
* **Target Frames:** `gripper_left_grasping_frame` / `gripper_right_grasping_frame`
* **Logic:** Maps the `menu` button directly to the gripper command topics.

### 2. Unitree G1 (Humanoid)

A specialized setup for high-speed humanoid teleoperation.

* **Reference Frame:** `pelvis`
* **Target Frames:** `left_hand_point_contact` / `right_hand_point_contact`
* **Logic:** Uses **Event-Driven publishing** (Frequency: `-1.0`) to minimize latency for the humanoid's whole-body controller.
* **Custom Serials:** Overrides the default hardware IDs directly in the launch file to match specific lab hardware.

---

## ⌨️ Usage

To launch a specific robot configuration inside the Docker container:

```bash
# For TIAGo
python3 scripts/run_docker.py tiago

# For Unitree G1
python3 scripts/run_docker.py g1

```

