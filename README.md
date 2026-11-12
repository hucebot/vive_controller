# ROS 2 Vive Controller
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

Teleoperate robot arms with HTC Vive controllers, in Docker. SteamVR runs headless in the container: no
headset, and no ROS 2 or SteamVR to install on the host. Used at HuCeBot for the Franka Panda, TIAGo,
TIAGo Pro and Unitree G1.

| | |
| :---: | :---: |
| <img src="assets/images/vive_controller_logo.png" alt="Logo" width="200"> | <img src="assets/images/test_rviz.gif" alt="Teleoperation in RViz" width="400"> |

## Contents
- [How it works](#how-it-works)
- [Requirements](#requirements)
- [First-time setup](#first-time-setup)
- [Daily use](#daily-use)
- [Configuration reference](#configuration-reference)
- [The driver (vive_node)](#the-driver-vive_node)
- [The teleop bridge (teleop_bridge_node)](#the-teleop-bridge-teleop_bridge_node)
- [Adding a robot](#adding-a-robot)
- [Vive Trackers](#vive-trackers)
- [Troubleshooting](#troubleshooting)
- [Tests](#tests)
- [Maintainers](#maintainers)

## How it works

```
SteamVR ──> vive_node (one per hand) ──> /vive/<side>/pose, joint_states ──> teleop_bridge_node ──> robot command topic
             filters, origin frame,                                          clutch on the trigger,
             safety fence, haptics                                           TF of the end effector
```

- **`vive_node`** reads one controller from SteamVR and publishes its pose and buttons. It can express poses relative to
  a base station, keeps the controller inside a painted workspace fence, and vibrates it.
- **`teleop_bridge_node`** turns the controller pose into an end-effector target for the robot while the trigger is held
  (the clutch), and republishes the buttons on the topics the robot expects.
- **Robot profiles** (`config/robots/<robot>.yaml`) describe each robot: which arms, which topics and frames.
- **The station file** (`config/station.yaml`, one per machine, written by `make setup`) says which controller is in
  which hand, which base station is the origin, which robot this machine drives, and each arm's fence.

## Requirements

- At least one HTC Vive base station (Lighthouse), and Vive controllers (or Vive Trackers 3.0, see [Vive Trackers](#vive-trackers)).
- Micro USB cables if the controllers are used wired.
- Docker and `make`.
- To build the image yourself: a Steam account (SteamVR is downloaded during the build). Lab members can pull the
  pre-built image instead.

## First-time setup

### 1. `.env`

```bash
cp .env.template .env
```

Fill in at least the Steam credentials (only needed to build the image) and `ROS_DOMAIN_ID`. Everything else is optional:

| Variable | Use |
| :--- | :--- |
| `STEAM_USER`, `STEAM_PASSWORD` | Building the image (`make build`) |
| `ROS_DOMAIN_ID` | ROS 2 domain shared with the robot |
| `LINEAR_SCALE` | Hand-to-robot translation scale. Unset: the robot profile's value |
| `CLUTCH_REFERENCE` | `tf` (default) or `last_command`, see [the bridge](#the-teleop-bridge-teleop_bridge_node) |
| `SERIAL_RIGHT`, `SERIAL_LEFT`, `REFERENCE_LIGHTHOUSE_SERIAL` | Fallback serials, used until `make setup` has saved its own |
| `SETUP_PORT` | Port of the setup web app (default `8800`) |

### 2. DDS network interface

Each robot uses its own Cyclone DDS file in `config/`. Put the name of the network interface that reaches the robot
(`ip link`) in it:

| Robot | File |
| :--- | :--- |
| Franka | `franka_cyclonedds.xml` |
| TIAGo, TIAGo Pro | `tiago_cyclonedds.xml` |
| G1, drivers only | `cyclonedds.xml` |

```xml
<NetworkInterface name="enx607d09178f25" />
```

The Franka file uses multicast for discovery only (`<AllowMulticast>spdp</AllowMulticast>`). With full multicast, data
from the robot was lost, so its TF arrived late. See [Troubleshooting](#troubleshooting).

### 3. Get the image

```bash
make pull     # lab members: pre-built image from the registry
make build    # or build it (needs STEAM_USER / STEAM_PASSWORD)
```

### 4. Set up the station

```bash
make setup
```

This starts a small web app next to SteamVR and prints its address. Open it on the PC, or on a tablet or phone on the
same network:

1. **Robot**: the robot this machine drives.
2. **Hands**: press *Assign*, then pull the trigger of the controller in that hand. *Buzz* vibrates a controller.
3. **Origin**: the base station used as the origin frame (only for robots whose profile uses one, i.e. Franka).
4. **Workspace fence**: press *Paint*, hold that hand's trigger and sweep the space the hand should reach, then *Done*.
   The 3D view shows the points, the fence, its warning band and the live controllers. Adjust the margin if needed.

*Save* writes `config/station.yaml` (local to the machine, not in git). `make setup-demo` runs the same app on simulated
controllers, to try it without hardware.

Base stations only show up once a controller has seen them, which can take up to 30 s after SteamVR starts. Hold a
controller up towards them.

## Daily use

```bash
make teleop      # start the robot saved in make setup
make franka      # or start a robot explicitly: franka, tiago, tiago_pro, g1
make stop        # stop the containers
```

Hold the trigger to move the arm, release it to stop. Teleop is relative: each stroke moves the arm by how much your hand
moved, so you can ratchet over long distances. The gripper follows the controller's orientation.

Other commands:

| Command | What it does |
| :--- | :--- |
| `make setup` | Setup web app (also while a robot runs: it then shares its SteamVR, but trigger actions are disabled) |
| `make setup-demo` | Setup app with simulated controllers |
| `make identify` | List the devices SteamVR sees (serial, battery, role, tracking) and vibrate each controller |
| `make drivers` | Only the Vive drivers and RViz, no robot |
| `make calibrate` | Paint the right arm's fence in RViz instead of the web app (`SIDE=left` for the left arm) |
| `make pull` / `build` / `push` | Image management |
| `make clean` | Remove the containers |

At startup the launch log lists the profile, the scale and where each serial came from:

```
Robot profile 'franka': Franka Panda, single arm (Cartesian impedance controller) (arms: right, linear scale 2.0)
  Right controller: LHR-21C1BC92 (config/station.yaml)
  Origin base station: LHB-4B20F6CF (config/station.yaml)
  Right arm fence from config/station.yaml: on (frame 'lighthouse_origin')
```

## Configuration reference

### Where each value comes from

| Value | Order (first one set wins) |
| :--- | :--- |
| Controller serials, origin base station | launch argument, then `config/station.yaml`, then `.env` |
| Workspace fence (per arm) | `config/station.yaml`, then the old `config/workspace.yaml` (both arms), then none |
| Scale, clutch reference | `.env` (`LINEAR_SCALE`, `CLUTCH_REFERENCE`), then the robot profile, then the bridge default |

A device listed in `config/station.yaml` wins even when it is empty, so choosing "no origin base station" in the app is
not undone by an old `.env` value.

### `config/station.yaml` (written by `make setup`)

```yaml
robot: franka
devices:
  right: LHR-21C1BC92
  left: ''
  origin_lighthouse: LHB-4B20F6CF
workspace:
  right: {enabled: true, frame_id: lighthouse_origin, x_min: -0.45, x_max: 0.40, y_min: 0.30, y_max: 1.05, z_min: -0.35, z_max: 0.30}
```

A `.bak` of the previous version is kept next to it.

### Robot profiles (`config/robots/<robot>.yaml`)

See [Adding a robot](#adding-a-robot). `franka.yaml` documents every key.

### Driver defaults (`config/vive.params.yaml`)

Filter settings (`filter.mincutoff`, `filter.beta`, `filter.dcutoff`), the frame without an origin base station
(`frame_id: vive_world`) and the fence's warning band (`workspace.padding`, 0.15 m).

## The driver (vive_node)

One node per hand, in the `/vive/<side>` namespace.

| Topic | Type | Content |
| :--- | :--- | :--- |
| `/vive/<side>/pose` | `geometry_msgs/PoseStamped` | Filtered, scaled controller pose |
| `/vive/<side>/joint_states` | `sensor_msgs/JointState` | Buttons (see below), always published |
| `/vive/<side>/workspace_marker` | `visualization_msgs/Marker` | The fence, for RViz |
| `/vive/<side>/identify` | `std_srvs/Trigger` (service) | Vibrates the controller for 3 s |

TF: `vive_world` → `lighthouse_origin` (with an origin base station) → `vive_<side>_link`.

- **Origin frame.** SteamVR's world origin is arbitrary. With an origin base station, poses are relative to it, turned
  180° about its z axis, in `lighthouse_origin`. Only robots whose profile sets `use_origin_lighthouse: true` use it
  (Franka). Enabling it for another robot changes the axes the arm moves along, so its `rotation_offset` may need
  re-tuning. Once known, the base station's pose is kept, so the frame never switches back mid-session. Base Station 2.0
  units can report their pose as invalid: the node then stays in `vive_world` and says so (`allow_invalid_lighthouse_pose:=true`
  uses the stored pose anyway).
- **Reliable poses only.** A partly covered controller is often still flagged valid while SteamVR only dead-reckons from
  the IMU, and those poses jump. The driver only publishes poses SteamVR tracks optically (`Running_OK`) and needs 5 good
  samples in a row (`tracking_ok_samples`) before publishing again. The log says why tracking was lost, e.g.
  `Tracking LOST for right (Calibrating_OutOfRange)`. `require_tracking_ok:=false` turns this off.
- **Workspace fence.** An axis-aligned box per arm, in the frame it was painted in. Within `workspace.padding` of a face,
  the controller vibrates and the pose is still published. Outside, it vibrates and the pose is held back, so the bridge
  stops the arm. A fence painted in another frame than the one the driver publishes in is ignored, with a warning.
- **Reconnects.** Controllers are looked up by serial on every cycle. A wireless controller switched off and on again
  is picked up again, and the other hand keeps its own controller. Wired controllers: see [Troubleshooting](#troubleshooting).
- **Filtering.** A OneEuro filter on the position removes hand jitter.

### Buttons (`joint_states`)

| Index | Name | Range | |
| :---: | :--- | :--- | :--- |
| 0 | `trigger` | 0.0 to 1.0 | The clutch |
| 1 | `trackpad_x` | -1.0 to 1.0 | |
| 2 | `trackpad_y` | -1.0 to 1.0 | |
| 3 | `grip` | 0 / 1 | Side buttons |
| 4 | `menu` | 0 / 1 | Button above the trackpad |
| 5 | `trackpad_touched` | 0 / 1 | |
| 6 | `trackpad_pressed` | 0 / 1 | Trackpad clicked |

<div align="center">
  <img src="assets/images/vive_button_map.png" alt="Vive controller buttons" width="450">
</div>

## The teleop bridge (teleop_bridge_node)

One node per arm, between the driver and the robot's Cartesian controller.

- **Clutch.** The arm follows the hand while the trigger is fully pulled (`clutch_engage_threshold`,
  `clutch_release_threshold`, both 1.0). For a worn trigger that never reaches 1.0, set e.g. 0.95 / 0.90.
- **Relative translation.** Each stroke moves the end effector by the hand's displacement (times `linear_scale`) from
  where the stroke started.
  - **Stroke start.** By default (`clutch_reference: tf`) a stroke starts from the end effector's TF.
    `last_command` starts from the last command instead, so a late robot TF cannot make the arm jump back when you
    clutch again. Set it with `CLUTCH_REFERENCE` in `.env`.
- **Absolute orientation.** The gripper takes the controller's orientation, corrected by `rotation_offset`
  (`[roll, pitch, yaw]` in degrees, applied in the controller frame). This is the intended design.
- **Lost pose.** If no pose arrives for `pose_timeout` s (0.3, 0 to disable) while clutched, because tracking was lost
  or the hand left the fence, the bridge lets go and holds the last command. Release and pull the trigger again to
  continue. The arm never jumps to where the controller reappears.
- **Buttons.** Each button is republished as `geometry_msgs/PointStamped` (value in `point.x`) on the topic the profile
  gives it, e.g. `menu` to a gripper command. With `trackpad_pressed_required: true`, `trackpad_x`/`trackpad_y` are only
  sent while the trackpad is clicked (used on G1, where the trackpad drives the hand).

Controller axes, for `rotation_offset`: z points out of the tip, x to the right, y up through the trackpad.

<div align="center">
  <img src="assets/images/vive_axis.png" alt="Vive controller axes" width="300">
</div>

## Adding a robot

A robot is a YAML profile; no Python or launch file to write.

```yaml
# config/robots/my_robot.yaml
description: My robot, dual arm
rviz: true
linear_scale: 1.0              # LINEAR_SCALE in .env overrides it
use_origin_lighthouse: false   # true: poses relative to the origin base station
arms:                          # left and/or right, one teleop bridge each
  right:
    output_topic: /my_robot/right_arm/target_pose   # the Cartesian controller's input
    reference_frame: base_link                      # robot root frame
    target_frame: right_gripper_link                # end effector frame
    publish_frequency: 30.0
    rotation_offset: [180.0, 0.0, 0.0]
    buttons:                                        # unlisted buttons go to /vive/right/<button>
      menu: /my_robot/right_gripper/toggle
  left:
    output_topic: /my_robot/left_arm/target_pose
    reference_frame: base_link
    target_frame: left_gripper_link
```

Any other `teleop_bridge_node` parameter can go under an arm (`trackpad_pressed_required`, `clutch_engage_threshold`,
`clutch_release_threshold`, `pose_timeout`, `clutch_reference`). A mistyped key stops the launch and lists the valid
ones.

Run it with:

```bash
ros2 launch ros2_vive_controller teleop.launch.py robot:=my_robot
```

Images built before the profiles existed don't have `teleop.launch.py` installed yet. Use its path instead,
`ros2 launch /ros2_ws/src/ros2_vive_controller/launch/teleop.launch.py robot:=my_robot`, until the next `make build`.

To get `make my_robot` and see the robot in `make setup`:
1. In `docker-compose.yml`, copy a robot service (e.g. `tiago:`), rename it and its profile, and set its command to
   `ros2 launch /ros2_ws/src/ros2_vive_controller/launch/teleop.launch.py robot:=my_robot linear_scale:=${LINEAR_SCALE:-} clutch_reference:=${CLUTCH_REFERENCE:-}`.
2. In the `Makefile`, add `my_robot: gui-perms` with `docker compose --profile my_robot up`.

## Vive Trackers

For Vive Trackers 3.0 (e.g. strapped to the hands), `tracker_3_0_node` and `bimanual_tracker_3_0_node` publish
`/vive_<name>/pose` and `/vive_<name>/pose_hand` (the pose shifted by `hand_offset`). They run from their own image:

```bash
docker compose --profile bimanual up   # SERIAL_LEFT / SERIAL_RIGHT, HAND_OFFSET_LEFT / HAND_OFFSET_RIGHT in .env
docker compose --profile simple up     # SERIAL, TRACKER_NAME, HAND_OFFSET in .env
```

## Troubleshooting

**The arm stops and the log says `Tracking LOST for right (...)`.** The controller was covered or out of view, and the
driver stopped publishing rather than send IMU guesses. Bring it back in view, release the trigger and pull it again.

**A base station is missing in `make setup` or `make identify`.** SteamVR lists a base station only once a controller has
seen it. Switch a controller on and hold it up towards the base stations for a few seconds.

**A wired controller is not found again after unplugging it or switching it off.** SteamVR inside the container does not
re-detect a USB controller (its log shows the disconnect, then nothing). Restart the robot (`make <robot>`). Wireless
controllers through a dongle are not affected.

**The arm jumps back to an old position when you clutch again.** The bridge started the stroke from a stale end-effector
TF. Check that the robot's data arrives at its normal rate (`ros2 topic hz /tf`). If it is slow or bursty, check the DDS
file: on Franka, full multicast lost the robot's data (fixed with `AllowMulticast spdp`). `CLUTCH_REFERENCE=last_command`
also avoids it.

**`Workspace fence was calibrated in 'X' but poses are in 'Y': fence disabled`.** The fence was painted with another
origin than the one in use (another robot, or another origin base station). Paint it again in `make setup`.

**`make setup` says the port is in use.** The app is already running somewhere, or another program uses port 8800. Stop
it, or set `SETUP_PORT` in `.env`.

**`steamclient.so: cannot open shared object file` in the log.** Harmless: SteamVR runs without the Steam client.

**Which controller is which?** `make identify` vibrates each one in turn, or press *Buzz* in `make setup`.

## Tests

The tests need no SteamVR or hardware: `ros2_vive_controller/testing/fake_openvr.py` stands in for OpenVR (the setup
app's demo mode uses it too). From the repository root:

```bash
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 --entrypoint bash \
  -v "$PWD":/work -w /work registry.gitlab.inria.fr/eurobin-horizon/code/ros2-vive-controller/vive-controller:latest \
  -lc "source /opt/ros/humble/setup.bash && python3 -m pytest -q -p no:cacheprovider test"
```

## Maintainers

Developed by the [HuCeBot team](https://github.com/hucebot) at Inria / Loria.

- Maintainers: [Dionis Totsila](https://github.com/dtotsila), [Jean-Baptiste Mouret](https://github.com/jbmouret)
- Active contributors: [Hippolyte Henry](https://github.com/hippolyte-bleu)
- Other contributors: [Clemente Donoso](https://github.com/cdonoso)
