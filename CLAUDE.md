# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ROS 2 (Humble) package for teleoperating robot arms (Franka, TIAGo, TIAGo Pro, Unitree G1) with HTC Vive controllers,
run in Docker with headless SteamVR. README.md has the user-facing setup and commands.

## Environment

- Nothing runs on the host: no ROS, numpy or SteamVR there. Use the image
  `registry.gitlab.inria.fr/eurobin-horizon/code/ros2-vive-controller/vive-controller:latest`; pytest and flake8 are in it.
- Development machines usually have no Vive hardware. The tests use a fake OpenVR, so anything that depends on real
  SteamVR or robot behaviour (tracking, haptics, reconnects, base stations, DDS) must be checked by a person in the lab.
  Say so and give them a concrete checklist (`make` targets, what to look at in the log or in RViz).

## Commands

```bash
# All tests (from the repo root)
docker run --rm --entrypoint bash -v "$PWD":/work -w /work \
  registry.gitlab.inria.fr/eurobin-horizon/code/ros2-vive-controller/vive-controller:latest \
  -lc "source /opt/ros/humble/setup.bash && python3 -m pytest -q test"
# One test: same, with  python3 -m pytest -q test/test_station.py::test_round_trip_with_backup
# Lint: same, with      python3 -m flake8 --max-line-length 120 <files>   (several older files have pre-existing warnings)

make setup-demo   # the setup web app on simulated controllers (no hardware); port 8800
make <robot>      # franka | tiago | tiago_pro | g1, needs the hardware; make help lists the rest
```

To check launch files in the image, mount the source like compose does, so the new robot profiles are found:
`-v "$PWD/config":/ros2_ws/src/ros2_vive_controller/config -v "$PWD/launch":/ros2_ws/src/ros2_vive_controller/launch
-v "$PWD/ros2_vive_controller":/ros2_ws/src/ros2_vive_controller/ros2_vive_controller`, then
`source /ros2_ws/install/setup.bash && ros2 launch ros2_vive_controller franka_single.launch.py --show-args`.

## Architecture

**Data flow.** `vive_node` (one per hand, namespace `/vive/<side>`) reads one controller from SteamVR and publishes
`pose` and `joint_states`. `teleop_bridge_node` (one per arm) applies the clutch (trigger) and turns them into the
robot's end-effector command plus per-button `PointStamped` topics. Pure logic lives in `teleop_core.py` (bridge),
`workspace.py` (fence), `frames.py` (origin frame), `station.py` and `launch_config.py`, and is unit tested directly.

**Configuration layers.**
- `config/robots/<robot>.yaml`: robot profiles (arms, topics, frames, `rotation_offset`, scale, whether the origin base
  station is used). Validated in `launch_config.py`; unknown keys are errors.
- `config/station.yaml`: per-machine, gitignored, written by the setup app. Holds the robot, serials per role and a
  fence per arm. `station.py` documents the format.
- `.env`: Steam credentials, `ROS_DOMAIN_ID`, `LINEAR_SCALE`, `CLUTCH_REFERENCE`, and fallback serials.
- `config/vive.params.yaml`: driver defaults. The old `config/workspace.yaml` (one fence for both arms) still loads.
- Serials resolve as launch argument, then station file, then `.env` (`station.resolve_device`). A device present in the
  station file wins even when it is `''`. For this to work, compose passes the `.env` serials as environment variables
  (`x-vive-env`), never as launch arguments.
- Parameter precedence relies on rclpy applying parameter files in order, last one wins. The station fence is passed as
  a dict parameter, which launch_ros writes to a temp file after the YAML files, so it overrides `workspace.yaml`
  (`test_params_precedence.py`).

**Launch.** `robot_launch.py` builds plain node specs (`robot_nodes()`, `drivers_nodes()`) from a profile and the station
file, then converts them to launch actions in an `OpaqueFunction`. The robot launch files (`franka_single`,
`tiago_dual`, `tiago_pro`, `g1_dual`) are 3-line wrappers that call it. They are kept because the image is built with
`--symlink-install`: the Python package directory is a symlink to the mounted source, so new modules work without a
rebuild, but `share/` only contains the launch and config files that existed at build time. Prefer adding Python code
over new launch files. `test_profiles.py` pins every robot's exact node parameters; update it deliberately when a
profile changes.

**Frames.** `openvr_class.convert_to_quaternion` already maps OpenVR's y-up coordinates (x, y, z) to z-up (z, x, y), so
every published pose and fence uses the converted frame. In tests, fake devices are placed in raw OpenVR coordinates
(see `_put()` in `test_setup_app.py`). With an origin base station, poses are relative to it turned 180° about z, in
`lighthouse_origin` (`frames.py`, shared by the driver and the setup app so fences are painted in the frame the driver
checks). Fences store their `frame_id`; the driver ignores a fence from another frame.

**SteamVR access.**
- `openvr_class.triad_openvr` is the device registry. Nodes look devices up by serial on every cycle and never keep
  names: names are never reused after a reconnect, and `sync_connected_devices()` catches devices SteamVR re-activates
  without an event.
- Two SteamVR servers cannot share the USB devices. `scripts/vive_run.sh` (`make setup`, `make identify`) therefore
  execs into a running `ros2_vive_*` container if there is one, otherwise starts a one-off `drivers` container.
- `vive_node` only publishes poses with `eTrackingResult == Running_OK`; partly covered controllers otherwise produce
  IMU-only poses that jump.

**Setup app** (`ros2_vive_controller/setup_app/`, stdlib only).
- `core.SetupApp` owns OpenVR from a single loop thread (`step()`, about 100 Hz). HTTP handler threads only read
  snapshots and queue commands (`call()`), and a command is answered once the snapshot showing its result is published.
- `static/index.html` is one self-contained page (no CDN), read from disk on every request.
- When the app runs inside a container where teleop is running (`VIVE_SETUP_ATTACHED_TO`), trigger-based actions are
  refused, because a trigger pull would also move the robot.
- `--demo` swaps in the fake runtime.

**Tests.**
- `ros2_vive_controller/testing/fake_openvr.py` is the fake runtime (also used by `--demo`). `test/conftest.py` installs
  it as `sys.modules['openvr']`, and the `vr_system` fixture resets it.
- ROS tests create real rclpy nodes and call their callbacks directly. They do not spin.

**Vive Trackers.** `tracker_3_0_node.py`, `bimanual_tracker_3_0_node.py` and `base_tracker_node.py` serve another
project (Bleu) through the `bimanual`/`simple` compose profiles and their own image. Keep their topics and parameters
unchanged.

## Design rules

- Orientation is absolute by design (the gripper takes the controller's orientation, plus `rotation_offset`); translation
  is relative to the clutch stroke. A relative-orientation mode may only be added as an opt-in, off by default.
- Never silently change how hand motion maps to robot motion (frames, orientation, scale, which origin a robot uses).
  Such changes must be opt-in and logged.
- Safety behaviour stays fail-safe: a stale pose (`pose_timeout`), lost tracking or leaving the fence makes the bridge let
  go and hold the last command; re-engaging needs a fresh trigger pull.
- The setup page and its messages use a plain lab-tool style: flat, dense, terse text, no decorative cards or pills.
