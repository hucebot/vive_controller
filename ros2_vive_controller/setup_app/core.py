"""Setup app logic: device roles, robot, origin base station and workspace fences.

The result is saved to config/station.yaml (see station.py), which the launch files read.

SetupApp owns the OpenVR connection. Everything that touches OpenVR or the app state runs in one
loop (step()), on its own thread when the app is served: the web server only reads snapshots
(state(), points_state()) and queues commands (call()). Tests call step() and execute() directly.
"""
import copy
import math
import os
import queue
import threading
import time
import traceback

import yaml

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import station as st
from ros2_vive_controller.frames import LIGHTHOUSE_FRAME, WORLD_FRAME, pose_in_origin
from ros2_vive_controller.openvr_class.openvr_class import tracking_ok, tracking_result_name
from ros2_vive_controller.workspace import AXES, bounds_from_points, classify

LOOP_PERIOD = 0.01          # s. Also the haptic period: one 3.9 ms pulse every 10 ms, as identify does.
PULSE_MICROS = 3900
STATE_PERIOD = 0.05         # s between two snapshots for the web page
ASSIGN_PRESSED = 0.9        # trigger value that counts as a pull when assigning a hand...
ASSIGN_RELEASED = 0.5       # ...after the trigger was seen below this
PAINT_TRIGGER = 0.5         # painting while the trigger is above this (as calibration_node)
PAINT_SPACING = 0.02        # m between two painted points (as calibration_node)
MIN_POINTS = 10
DEFAULT_MARGIN = 0.05
DEFAULT_PADDING = 0.15
SIDES = st.SIDES
DRIVERS_CONTAINER = 'ros2_vive_drivers'   # drivers only, no teleop bridge: triggers move nothing

COMMANDS = ('buzz', 'assign', 'cancel', 'set_device', 'select_robot', 'select_origin', 'paint_start',
            'paint_stop', 'paint_clear', 'set_margin', 'set_fence_enabled', 'save', 'revert')


class CommandError(ValueError):
    """A request the app refuses; the message is shown to the user."""


def _safe(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


def _distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class SetupApp:
    def __init__(self, vr, config_dir, environ=None, clock=time.monotonic, attached_to='', on_step=None):
        """vr: a triad_openvr. attached_to: name of the container the app runs in, when that
        container also runs teleop (then the triggers drive the robot, so trigger actions are off)."""
        self.vr = vr
        self.config_dir = config_dir
        self.station_path = st.path_in(config_dir)
        self.environ = os.environ if environ is None else environ
        self.clock = clock
        self.on_step = on_step
        self.teleop_running = attached_to if attached_to and attached_to != DRIVERS_CONTAINER else ''

        self.profiles = self._load_profiles()
        self.padding = self._padding()
        self.legacy_fence = self._legacy_fence()
        self.saved = st.load(self.station_path)
        self.station_exists = os.path.isfile(self.station_path)
        self.draft = self._draft_from(self.saved)

        self.assigning = None           # side waiting for a trigger pull
        self.painting = None            # side being painted
        self.points = {side: [] for side in SIDES}
        self.points_frame = {side: None for side in SIDES}
        self.points_version = 0
        self.margin = DEFAULT_MARGIN
        self.message = None

        self._armed = set()
        self._triggers = {}
        self._models = {}
        self._buzz_until = {}
        self._lighthouse_poses = {}     # serial -> last known pose (base stations do not move)
        self._last_sync = None
        self._last_state = None
        self._commands = queue.Queue()
        self._lock = threading.Lock()
        self._state = {}
        self._points_state = {'version': 0, 'points': {side: [] for side in SIDES}}
        self._thread = None
        self._stop = threading.Event()

    # --- configuration files ----------------------------------------------------------------

    def _load_profiles(self):
        profiles = {}
        for name in lc.list_profiles(self.config_dir):
            try:
                profiles[name] = lc.load_profile(name, self.config_dir)
            except lc.ProfileError as error:
                profiles[name] = {'name': name, 'error': str(error)}
        return profiles

    def _read_parameters(self, file_name):
        try:
            with open(os.path.join(self.config_dir, file_name)) as f:
                return yaml.safe_load(f)['/**']['ros__parameters']['workspace']
        except Exception:
            return None

    def _padding(self):
        params = self._read_parameters('vive.params.yaml') or {}
        return float(params.get('padding', DEFAULT_PADDING))

    def _legacy_fence(self):
        """The old-style fence (config/workspace.yaml), which applies to arms without a station fence."""
        params = self._read_parameters('workspace.yaml')
        return st.normalize_fence(params) if isinstance(params, dict) else None

    def _draft_from(self, saved):
        """What the page edits: the station file, with unset devices taken from .env as teleop would."""
        draft = copy.deepcopy(saved)
        for role in st.ROLES:
            if draft['devices'][role] is None:
                draft['devices'][role] = st.resolve_device(role, '', saved, self.environ)[0]
        return draft

    # --- frames ---------------------------------------------------------------------------------

    def profile(self):
        profile = self.profiles.get(self.draft['robot'])
        return profile if profile and 'error' not in profile else None

    def uses_origin_lighthouse(self):
        profile = self.profile()
        # No robot chosen: like `make drivers` and `make calibrate`, use it when there is one.
        return profile['use_origin_lighthouse'] if profile else True

    def target_frame(self):
        """(frame the robot's poses will be in, origin base station serial or '')."""
        serial = self.draft['devices']['origin_lighthouse'] if self.uses_origin_lighthouse() else ''
        return (LIGHTHOUSE_FRAME if serial else WORLD_FRAME), serial

    def _lighthouse_pose(self, serial, poses):
        device = self.vr.get_device_by_serial(serial)
        pose = device.get_pose_quaternion(poses) if device is not None else None
        if pose is not None:
            self._lighthouse_poses[serial] = pose
        return self._lighthouse_poses.get(serial)

    # --- the loop -------------------------------------------------------------------------------

    def start(self):
        self._thread = threading.Thread(target=self._loop, name='vive-setup-loop', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _loop(self):
        next_time = time.monotonic()
        while not self._stop.is_set():
            try:
                self.step()
            except Exception:
                traceback.print_exc()
                self._stop.wait(0.5)
            next_time += LOOP_PERIOD
            delay = next_time - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_time = time.monotonic()

    def step(self, now=None):
        now = self.clock() if now is None else now
        if self.on_step is not None:
            self.on_step(now)
        finished = self._run_commands(now)
        try:
            self._update(now, force_state=bool(finished))
        finally:
            # Answer the web page only once the snapshot shows the command's result.
            for done in finished:
                done.set()

    def _update(self, now, force_state=False):
        self.vr.poll_vr_events()
        if self._last_sync is None or now - self._last_sync >= 1.0:
            self._last_sync = now
            self.vr.sync_connected_devices()

        poses = self.vr.get_pose()
        frame, lighthouse = self.target_frame()
        origin = self._lighthouse_pose(lighthouse, poses) if lighthouse else None
        frame_ready = not lighthouse or origin is not None
        if self.painting and self.points_frame[self.painting] not in (None, frame):
            self._set_message(f'Frame changed to {frame}: painting stopped.', 'warn', now)
            self.points[self.painting] = []
            self.points_version += 1
            self.painting = None

        devices = []
        for device in list(self.vr.devices.values()):
            serial = device.get_serial()
            if device.device_class == 'HMD' or not serial:
                continue
            devices.append(self._device_state(device, serial, poses, origin, frame, frame_ready, now))

        if force_state or self._last_state is None or now - self._last_state >= STATE_PERIOD:
            self._last_state = now
            self._publish(devices, frame, lighthouse, frame_ready)

    def _device_state(self, device, serial, poses, origin, frame, frame_ready, now):
        if serial not in self._models:
            self._models[serial] = _safe(device.get_model, '')
        info = {'serial': serial, 'class': device.device_class, 'model': self._models[serial],
                'role': self._role_of(serial)}
        if device.device_class == 'Tracking Reference':
            info['mode'] = _safe(device.get_mode, '')
            info['pose_known'] = (serial in self._lighthouse_poses
                                  or device.get_pose_quaternion(poses) is not None)
            return info

        battery = _safe(device.get_battery_percent)
        info['battery'] = None if battery is None else round(battery * 100)
        info['charging'] = bool(_safe(device.is_charging, False))
        result = device.get_tracking_result(poses)
        pose = device.get_pose_quaternion(poses)
        info['tracking'] = tracking_result_name(result)
        info['tracked'] = pose is not None and tracking_ok(result)
        position = None
        if info['tracked'] and frame_ready:
            position = pose[:3] if origin is None else pose_in_origin(pose, origin)[:3]
            position = [round(float(v), 4) for v in position]
        info['position'] = position
        if device.device_class != 'Controller':
            return info

        inputs = _safe(device.get_controller_inputs, {}) or {}
        trigger = float(inputs.get('trigger', 0.0))
        info['trigger'] = round(trigger, 3)
        self._handle_trigger(serial, trigger, position, frame, now)
        if self._buzz_until.get(serial, 0.0) > now:
            device.trigger_haptic_pulse(PULSE_MICROS)
        side = info['role'] if info['role'] in SIDES else None
        info['fence'] = self._fence_state(side, position, frame)
        return info

    def _handle_trigger(self, serial, trigger, position, frame, now):
        if self.assigning:
            if trigger < ASSIGN_RELEASED:
                self._armed.add(serial)
            elif trigger >= ASSIGN_PRESSED and serial in self._armed:
                side, self.assigning = self.assigning, None
                self._assign(side, serial)
                self._buzz(serial, 0.5, now)
                self._set_message(f'{side.capitalize()} hand: {serial}.', 'ok', now)
        side = self.painting
        if side and serial == self.draft['devices'][side] and trigger > PAINT_TRIGGER and position is not None:
            points = self.points[side]
            if not points or _distance(position, points[-1]) > PAINT_SPACING:
                points.append(position)
                self.points_frame[side] = frame
                self.points_version += 1

    def _role_of(self, serial):
        for role in st.ROLES:
            if serial and self.draft['devices'][role] == serial:
                return role
        return ''

    def _assign(self, side, serial):
        devices = self.draft['devices']
        other = 'left' if side == 'right' else 'right'
        if devices[other] == serial:        # swap hands
            devices[other] = devices[side] if devices[side] != serial else ''
        devices[side] = serial

    def _buzz(self, serial, seconds, now):
        self._buzz_until[serial] = max(self._buzz_until.get(serial, 0.0), now + seconds)

    def _fence_state(self, side, position, frame):
        fence = self.draft['workspace'].get(side) if side else None
        if not fence or not fence['enabled'] or fence['frame_id'] != frame or position is None:
            return None
        return classify(position, fence, self.padding)

    def _set_message(self, text, level, now):
        self.message = {'text': text, 'level': level, 'time': now}

    # --- commands -------------------------------------------------------------------------------

    def call(self, name, timeout=2.0, **kwargs):
        """Thread-safe: run a command on the loop thread and return its message."""
        if name not in COMMANDS:
            raise CommandError(f'Unknown command: {name}')
        if self._thread is None:
            return self.execute(name, **kwargs)
        done, result = threading.Event(), {}
        self._commands.put((name, kwargs, done, result))
        if not done.wait(timeout):
            raise CommandError('The app is busy, try again.')
        if 'error' in result:
            raise CommandError(result['error'])
        return result['message']

    def _run_commands(self, now):
        """Run the queued commands; returns their events, to be set once the state is published."""
        finished = []
        while True:
            try:
                name, kwargs, done, result = self._commands.get_nowait()
            except queue.Empty:
                return finished
            try:
                result['message'] = self.execute(name, now=now, **kwargs)
            except CommandError as error:
                result['error'] = str(error)
            except Exception as error:
                traceback.print_exc()
                result['error'] = f'Internal error: {error}'
            finished.append(done)

    def execute(self, name, now=None, **kwargs):
        """Run a command now (on the loop thread). Returns a message for the user."""
        if name not in COMMANDS:
            raise CommandError(f'Unknown command: {name}')
        now = self.clock() if now is None else now
        try:
            message = getattr(self, f'_cmd_{name}')(now, **kwargs)
        except TypeError as error:
            raise CommandError(f'Bad request for {name}: {error}')
        except CommandError as error:
            self._set_message(str(error), 'error', now)
            raise
        if message:
            self._set_message(message, 'ok', now)
        return message

    def _controller(self, serial):
        device = self.vr.get_device_by_serial(serial) if serial else None
        return device if device is not None and device.device_class == 'Controller' else None

    def _no_trigger_actions(self):
        if self.teleop_running:
            raise CommandError(f'Teleop running in {self.teleop_running}: a trigger pull would move the robot. '
                               f'Stop it first (make stop) or use the lists.')

    def _cmd_buzz(self, now, serial, seconds=1.5):
        if self._controller(serial) is None:
            raise CommandError(f'Controller {serial} is not connected.')
        self._buzz(serial, min(float(seconds), 5.0), now)
        return ''

    def _cmd_assign(self, now, side):
        if side not in SIDES:
            raise CommandError(f'Unknown hand: {side}')
        self._no_trigger_actions()
        if not any(d.device_class == 'Controller' for d in self.vr.devices.values()):
            raise CommandError('No controller is connected.')
        self.painting = None
        self.assigning = side
        self._armed = set()
        return f'Pull the trigger in your {side.upper()} hand.'

    def _cmd_cancel(self, now):
        self.assigning = None
        if self.painting:
            self.painting = None
            return 'Painting cancelled.'
        return ''

    def _cmd_set_device(self, now, role, serial):
        if role not in st.ROLES:
            raise CommandError(f'Unknown role: {role}')
        serial = st.clean_value(serial)
        wanted = 'Tracking Reference' if role == 'origin_lighthouse' else 'Controller'
        known = {d.get_serial() for d in self.vr.devices.values() if d.device_class == wanted}
        if serial and serial not in known and serial != self.draft['devices'][role]:
            raise CommandError(f'{serial} is not a connected {wanted.lower()}.')
        if role in SIDES and serial:
            self._assign(role, serial)
        else:
            self.draft['devices'][role] = serial
        return ''

    def _cmd_select_robot(self, now, robot):
        if robot and robot not in self.profiles:
            raise CommandError(f'Unknown robot: {robot}')
        if robot and 'error' in self.profiles[robot]:
            raise CommandError(self.profiles[robot]['error'])
        self.draft['robot'] = robot
        return ''

    def _cmd_select_origin(self, now, serial):
        return self._cmd_set_device(now, 'origin_lighthouse', serial)

    def _cmd_paint_start(self, now, side):
        if side not in SIDES:
            raise CommandError(f'Unknown arm: {side}')
        self._no_trigger_actions()
        serial = self.draft['devices'][side]
        if self._controller(serial) is None:
            raise CommandError(f'Assign a connected controller to the {side} hand first.')
        frame, lighthouse = self.target_frame()
        if lighthouse and lighthouse not in self._lighthouse_poses:
            raise CommandError(f'Base station {lighthouse}: position not known yet (point a controller at it).')
        self.assigning = None
        self.painting = side
        self.points[side] = []
        self.points_frame[side] = frame
        self.points_version += 1
        return f'Painting {side} arm: hold the {side.upper()} trigger and sweep, then Done.'

    def _fence_from_points(self, side):
        bounds = bounds_from_points(self.points[side], margin=self.margin)
        if bounds is None:
            raise CommandError('All points rejected as outliers. Paint again.')
        self.draft = st.set_fence(self.draft, side, bounds, self.points_frame[side])

    def _cmd_paint_stop(self, now):
        side, self.painting = self.painting, None
        if not side:
            return ''
        count = len(self.points[side])
        if count < MIN_POINTS:
            raise CommandError(f'Only {count} points (need {MIN_POINTS}, 2 cm apart): hold the trigger while sweeping.')
        self._fence_from_points(side)
        return f'{side.capitalize()} fence: {count} points. Save to keep it.'

    def _cmd_paint_clear(self, now, side):
        if side not in SIDES:
            raise CommandError(f'Unknown arm: {side}')
        if self.painting == side:
            self.painting = None
        self.points[side] = []
        self.points_version += 1
        self.draft['workspace'].pop(side, None)
        return f'{side.capitalize()} fence removed.'

    def _cmd_set_margin(self, now, margin):
        margin = float(margin)
        if not 0.0 <= margin <= 0.3:
            raise CommandError('The margin must be between 0 and 0.3 m.')
        self.margin = margin
        for side in SIDES:
            if len(self.points[side]) >= MIN_POINTS and self.painting != side and side in self.draft['workspace']:
                self._fence_from_points(side)
        return ''

    def _cmd_set_fence_enabled(self, now, side, enabled):
        fence = self.draft['workspace'].get(side)
        if fence is None or not all(key in fence for key in st.BOUND_KEYS):
            raise CommandError(f'No {side} fence yet.')
        fence['enabled'] = bool(enabled)
        return ''

    def _cmd_save(self, now):
        self.painting = None
        self.assigning = None
        st.save(self.station_path, self.draft)
        self.saved = st.load(self.station_path)
        self.station_exists = True
        robot = self.draft['robot']
        restart = f'make {robot} (or make teleop)' if robot else 'the robot'
        return f'Saved config/{st.FILE_NAME}. Restart {restart} to use it.'

    def _cmd_revert(self, now):
        self.saved = st.load(self.station_path)
        self.draft = self._draft_from(self.saved)
        self.painting = None
        self.assigning = None
        return f'Reverted to config/{st.FILE_NAME}.'

    # --- snapshots for the web page ---------------------------------------------------------

    def _robot_summary(self, name, profile):
        if 'error' in profile:
            return {'error': profile['error']}
        arms = {side: {key: arm.get(key) for key in ('output_topic', 'reference_frame', 'target_frame')}
                for side, arm in profile['arms'].items()}
        return {'description': profile['description'], 'arms': arms, 'linear_scale': profile['linear_scale'],
                'use_origin_lighthouse': profile['use_origin_lighthouse']}

    def _warnings(self, devices, frame, lighthouse, frame_ready):
        warnings = []
        connected = {d['serial'] for d in devices}
        profile = self.profile()
        if self.teleop_running:
            warnings.append(f'Teleop running in {self.teleop_running}: trigger assign and painting are disabled '
                            f'(a pull would move the robot).')
        if not profile:
            warnings.append('No robot selected.')
        sides = list(profile['arms']) if profile else [s for s in SIDES if self.draft['devices'][s]]
        for side in sides:
            serial = self.draft['devices'][side]
            if not serial:
                warnings.append(f'No controller for the {side} hand.')
            elif serial not in connected:
                warnings.append(f'{side.capitalize()} controller {serial} not connected.')
        if profile and profile['use_origin_lighthouse'] and not lighthouse:
            warnings.append(f"{profile['name']} expects an origin base station; without one, poses are in "
                            f"{WORLD_FRAME}.")
        if lighthouse and not frame_ready:
            warnings.append(f'Base station {lighthouse}: position not known yet (point a controller at it).')
        for side in sides:
            fence = self.draft['workspace'].get(side)
            if fence and fence['enabled']:
                if fence['frame_id'] != frame:
                    warnings.append(f"{side.capitalize()} fence is in '{fence['frame_id']}', the robot runs in "
                                    f"'{frame}': the driver ignores it. Repaint.")
                thin = [a for a in AXES if fence[f'{a}_max'] - fence[f'{a}_min'] < 2 * self.padding]
                if thin:
                    warnings.append(f"{side.capitalize()} fence is under 2 x the warning band ({self.padding} m) "
                                    f"along {', '.join(thin)}: it vibrates everywhere.")
            elif fence is None and self.legacy_fence and self.legacy_fence['enabled']:
                warnings.append(f"{side.capitalize()} arm: the old config/workspace.yaml fence applies "
                                f"(frame '{self.legacy_fence['frame_id']}').")
        return warnings

    def _publish(self, devices, frame, lighthouse, frame_ready):
        draft = copy.deepcopy(self.draft)
        state = {
            'devices': devices,
            'draft': draft,
            'dirty': not self.station_exists or st.normalize(draft) != self.saved,
            'station_exists': self.station_exists,
            'robots': {name: self._robot_summary(name, p) for name, p in self.profiles.items()},
            'frame': {'id': frame, 'lighthouse': lighthouse, 'ready': frame_ready,
                      'uses_origin': self.uses_origin_lighthouse()},
            'assigning': self.assigning,
            'painting': self.painting,
            'points_count': {side: len(self.points[side]) for side in SIDES},
            'points_version': self.points_version,
            'margin': self.margin,
            'padding': self.padding,
            'legacy_fence': self.legacy_fence,
            'teleop_running': self.teleop_running,
            'warnings': self._warnings(devices, frame, lighthouse, frame_ready),
            'message': self.message,
        }
        with self._lock:
            self._state = state
            if self._points_state['version'] != self.points_version:
                self._points_state = {'version': self.points_version,
                                      'points': {side: list(self.points[side]) for side in SIDES}}

    def state(self):
        with self._lock:
            return self._state

    def points_state(self):
        with self._lock:
            return self._points_state
