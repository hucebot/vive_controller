"""This machine's setup: config/station.yaml, written by the setup app (`make setup`).

    robot: franka
    devices: {right: LHR-21C1BC92, left: '', origin_lighthouse: LHB-4B20F6CF}
    workspace:
      right: {enabled: true, frame_id: lighthouse_origin, x_min: ..., x_max: ..., ...}

The controllers and the origin base station are looked up in this order: launch argument, this
file, then the .env variables (SERIAL_RIGHT, SERIAL_LEFT, REFERENCE_LIGHTHOUSE_SERIAL). A device
listed here wins even when it is empty (''), so "no left controller" or "no origin base station"
chosen in the app is not undone by an old .env value. The file is local to each machine (not in git).
"""
import datetime
import os
import shutil

import yaml

from ros2_vive_controller.workspace import AXES

FILE_NAME = 'station.yaml'
SIDES = ('right', 'left')
ROLES = ('right', 'left', 'origin_lighthouse')
ENV_VARIABLES = {'right': 'SERIAL_RIGHT', 'left': 'SERIAL_LEFT', 'origin_lighthouse': 'REFERENCE_LIGHTHOUSE_SERIAL'}
BOUND_KEYS = tuple(f'{axis}_{end}' for axis in AXES for end in ('min', 'max'))

SOURCE_ARGUMENT = 'launch argument'
SOURCE_STATION = 'config/station.yaml'
SOURCE_ENV = '.env'
NOT_SET = 'not set'


def clean_value(value):
    """A .env value without its inline comment ('LHR-1234 # right hand' -> 'LHR-1234')."""
    return ('' if value is None else str(value)).split('#')[0].strip()


def path_in(config_dir):
    return os.path.join(config_dir, FILE_NAME)


def empty():
    """No station file: nothing chosen yet (devices None = fall back to .env)."""
    return {'robot': '', 'devices': {role: None for role in ROLES}, 'workspace': {}}


def normalize_fence(fence):
    out = {'enabled': bool(fence.get('enabled', False)), 'frame_id': clean_value(fence.get('frame_id'))}
    for key in BOUND_KEYS:
        if fence.get(key) is not None:
            out[key] = float(fence[key])
    if out['enabled'] and not all(key in out for key in BOUND_KEYS):
        out['enabled'] = False      # never enable a fence with made-up bounds
    return out


def normalize(data):
    station = empty()
    if not isinstance(data, dict):
        return station
    station['robot'] = clean_value(data.get('robot'))
    devices = data.get('devices') if isinstance(data.get('devices'), dict) else {}
    for role in ROLES:
        if devices.get(role) is not None:       # None/absent: not chosen, '' : chosen to be none
            station['devices'][role] = clean_value(devices[role])
    workspace = data.get('workspace') if isinstance(data.get('workspace'), dict) else {}
    for side in SIDES:
        if isinstance(workspace.get(side), dict):
            station['workspace'][side] = normalize_fence(workspace[side])
    return station


def load(path):
    """The station file, or empty() when there is none."""
    try:
        with open(path) as f:
            return normalize(yaml.safe_load(f))
    except FileNotFoundError:
        return empty()


def _to_yaml(station):
    station = normalize(station)
    data = {'robot': station['robot']}
    devices = {role: value for role, value in station['devices'].items() if value is not None}
    if devices:
        data['devices'] = devices
    if station['workspace']:
        data['workspace'] = station['workspace']
    return data


def _match_owner(paths, directory):
    """Files written from the container are root-owned; give them the config directory's owner."""
    try:
        st = os.stat(directory)
        for path in paths:
            if os.path.exists(path):
                os.chown(path, st.st_uid, st.st_gid)
    except OSError:
        pass


def save(path, station, writer='the setup app (make setup)'):
    """Write the station file, keeping a .bak of the previous one. Returns the backup path or None."""
    backup = None
    if os.path.exists(path):
        backup = path + '.bak'
        shutil.copy(path, backup)
    header = (
        f'# This machine\'s Vive setup, written by {writer} on {datetime.datetime.now():%Y-%m-%d %H:%M}.\n'
        '# Local to this machine (not tracked by git). Values here win over the serials in .env.\n'
    )
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(header)
        yaml.safe_dump(_to_yaml(station), f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, path)
    _match_owner([path, backup] if backup else [path], os.path.dirname(os.path.abspath(path)))
    return backup


def set_fence(station, side, bounds, frame_id, enabled=True):
    """A copy of the station with this arm's fence replaced."""
    station = normalize(station)
    fence = {'enabled': enabled, 'frame_id': frame_id}
    fence.update(bounds or {})
    station['workspace'][side] = normalize_fence(fence)
    return station


def resolve_device(role, argument, station, environ):
    """(serial, where it came from) for a role: launch argument, station file, then .env."""
    value = clean_value(argument)
    if value:
        return value, SOURCE_ARGUMENT
    value = station['devices'].get(role)
    if value is not None:
        return value, SOURCE_STATION
    value = clean_value(environ.get(ENV_VARIABLES[role]))
    if value:
        return value, SOURCE_ENV
    return '', NOT_SET


def fence_parameters(fence):
    """vive_node parameters for a station fence ({} when the arm has none)."""
    if not fence:
        return {}
    params = {'workspace.enabled': fence['enabled'], 'workspace.frame_id': fence['frame_id']}
    params.update({f'workspace.{key}': fence[key] for key in BOUND_KEYS if key in fence})
    return params
