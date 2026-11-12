"""Robot profiles (config/robots/<robot>.yaml) and the station file (config/station.yaml) turned
into node parameters for the launch files.

Plain Python, no launch imports, so it can be tested directly. robot_launch.py builds the nodes.
"""
import os

import yaml

from ros2_vive_controller import station as st
from ros2_vive_controller.station import clean_value  # noqa: F401  (re-exported)

PACKAGE = 'ros2_vive_controller'
# Compose mounts ./config here. Files created after the image was built (a new profile, the fence)
# only exist in the source tree: with --symlink-install, share/ only links the files that existed
# at build time.
SOURCE_CONFIG_DIR = f'/ros2_ws/src/{PACKAGE}/config'

SIDES = ('left', 'right')
BUTTONS = ('trigger', 'trackpad_x', 'trackpad_y', 'grip', 'menu', 'trackpad_touched', 'trackpad_pressed')

# teleop_bridge_node parameters an arm of a profile may set.
BRIDGE_PARAMETERS = (
    'output_topic', 'publish_frequency', 'target_frame', 'reference_frame', 'rotation_offset',
    'trackpad_pressed_required', 'clutch_engage_threshold', 'clutch_release_threshold',
    'pose_timeout', 'clutch_reference',
)
REQUIRED_ARM_KEYS = ('output_topic', 'target_frame', 'reference_frame')
ARM_KEYS = BRIDGE_PARAMETERS + ('bridge_name', 'buttons')
PROFILE_KEYS = ('description', 'rviz', 'linear_scale', 'use_origin_lighthouse', 'arms')
PROFILE_DEFAULTS = {'description': '', 'rviz': True, 'linear_scale': 1.0, 'use_origin_lighthouse': False}


class ProfileError(ValueError):
    """A robot profile that does not exist or is not valid."""


def config_dir(share_dir=None, source_dir=None):
    """The config directory: the source tree when it is there (in the containers), else share/."""
    source_dir = SOURCE_CONFIG_DIR if source_dir is None else source_dir
    if source_dir and os.path.isdir(source_dir):
        return source_dir
    if share_dir is None:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(PACKAGE)
    return os.path.join(share_dir, 'config')


def profiles_dir(config):
    return os.path.join(config, 'robots')


def list_profiles(config):
    """Names of the robot profiles in config/robots/."""
    try:
        files = os.listdir(profiles_dir(config))
    except FileNotFoundError:
        return []
    return sorted(f[:-len('.yaml')] for f in files if f.endswith('.yaml'))


def load_profile(name, config):
    """The validated profile, with defaults filled in."""
    path = os.path.join(profiles_dir(config), f'{name}.yaml')
    if not name or not os.path.isfile(path):
        available = ', '.join(list_profiles(config)) or 'none'
        raise ProfileError(f"Unknown robot '{name}'. Profiles in {profiles_dir(config)}: {available}")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return validate_profile(name, data)


def _unknown_keys(where, data, allowed):
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ProfileError(f"{where}: unknown key(s) {', '.join(unknown)}. Allowed: {', '.join(allowed)}")


def validate_profile(name, data):
    if not isinstance(data, dict):
        raise ProfileError(f"Robot profile '{name}' must be a mapping")
    _unknown_keys(f"Robot profile '{name}'", data, PROFILE_KEYS)
    arms = data.get('arms')
    if not isinstance(arms, dict) or not arms:
        raise ProfileError(f"Robot profile '{name}': 'arms' must list at least one of {', '.join(SIDES)}")
    profile = dict(PROFILE_DEFAULTS)
    profile.update({k: v for k, v in data.items() if k != 'arms'})
    profile['name'] = name
    profile['arms'] = {}
    for side, arm in arms.items():
        where = f"Robot profile '{name}', arm '{side}'"
        if side not in SIDES:
            raise ProfileError(f"{where}: the arm must be one of {', '.join(SIDES)}")
        if not isinstance(arm, dict):
            raise ProfileError(f"{where} must be a mapping")
        _unknown_keys(where, arm, ARM_KEYS)
        missing = [k for k in REQUIRED_ARM_KEYS if k not in arm]
        if missing:
            raise ProfileError(f"{where}: missing {', '.join(missing)}")
        buttons = arm.get('buttons') or {}
        _unknown_keys(f'{where}, buttons', buttons, BUTTONS)
        profile['arms'][side] = dict(arm, buttons=buttons)
    return profile


def bridge_name(side, arm):
    return arm.get('bridge_name') or f'teleop_bridge_{side}'


def bridge_parameters(side, arm, clutch_reference=''):
    """Parameters of the teleop_bridge_node for one arm of a profile."""
    params = {
        'pose_topic': f'/vive/{side}/pose',
        'button_state_topic': f'/vive/{side}/joint_states',
    }
    params.update({k: arm[k] for k in BRIDGE_PARAMETERS if k in arm})
    if clutch_reference:
        params['clutch_reference'] = clutch_reference
    for button in BUTTONS:
        params[f'{button}_topic'] = arm['buttons'].get(button, f'/vive/{side}/{button}')
    return params


def driver_parameters(side, serial, lighthouse_serial, linear_scale, fence=None):
    """Per-side parameters of vive_node (the defaults come from config/vive.params.yaml).

    fence: this arm's fence from the station file; it overrides config/workspace.yaml.
    """
    params = {
        'side': side,
        'linear_scale': float(linear_scale),
        'serial': serial,
        'reference_lighthouse_serial': lighthouse_serial,
    }
    params.update(st.fence_parameters(fence))
    return params


def driver_parameter_files(config):
    """Defaults, then the fence written by `make calibrate` (if any), which overrides them."""
    files = [os.path.join(config, 'vive.params.yaml')]
    workspace = os.path.join(config, 'workspace.yaml')
    if os.path.isfile(workspace):
        files.append(workspace)
    return files


# Launch argument -> station role
DEVICE_ROLES = (('serial_left', 'left'), ('serial_right', 'right'),
                ('reference_lighthouse_serial', 'origin_lighthouse'))


def resolve_devices(arguments, station, environ, use_origin_lighthouse=True):
    """Serials by launch argument name, and where each came from (for the launch log).

    Order: launch argument, station file, .env. A robot whose profile does not use the origin base
    station gets none, unless it is given as a launch argument.
    """
    devices, sources = {}, {}
    for name, role in DEVICE_ROLES:
        devices[name], sources[name] = st.resolve_device(role, arguments.get(name, ''), station, environ)
    if not use_origin_lighthouse and sources['reference_lighthouse_serial'] != st.SOURCE_ARGUMENT:
        devices['reference_lighthouse_serial'] = ''
        sources['reference_lighthouse_serial'] = 'not used by this robot'
    return devices, sources
