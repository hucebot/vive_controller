#!/usr/bin/env python3
"""List the Vive devices SteamVR sees and vibrate each controller in turn.

Standalone: it needs SteamVR (the Docker image) but no ROS nodes and no robot. `make identify`
runs it inside the running vive container, or in a one-off container when none is running.

The roles shown come from config/station.yaml (written by `make setup`), falling back to the
SERIAL_RIGHT, SERIAL_LEFT and REFERENCE_LIGHTHOUSE_SERIAL values of .env, as for teleop.
"""
import argparse
import os
import sys
import time

from ros2_vive_controller import station as st

NOT_CONFIGURED = 'no role'


def current_roles(station, environ=os.environ):
    """{serial: 'right hand (config/station.yaml)', ...}, resolved as the launch files do."""
    labels = {'right': 'right hand', 'left': 'left hand', 'origin_lighthouse': 'origin base station'}
    roles = {}
    for role in st.ROLES:
        serial, source = st.resolve_device(role, '', station, environ)
        if serial:
            roles[serial] = f'{labels[role]} ({source})'
    return roles


def wait_for_devices(vr, expected, min_wait, max_wait, sleep=time.sleep, clock=time.monotonic):
    """Poll SteamVR for at least min_wait s, and up to max_wait s until all expected serials are seen."""
    start = clock()
    while True:
        vr.poll_vr_events()
        elapsed = clock() - start
        found_all = all(vr.get_device_by_serial(serial) is not None for serial in expected)
        if (found_all and elapsed >= min_wait) or elapsed >= max_wait:
            return
        sleep(0.1)


def _has_base_station(vr):
    return any(d.device_class == 'Tracking Reference' for d in vr.devices.values())


def wait_for_base_stations(vr, timeout, out=print, sleep=time.sleep, clock=time.monotonic):
    """SteamVR lists a base station only once a controller or tracker has seen it, which can take
    a while after SteamVR starts. Wait for the first one (and a moment for the others)."""
    if timeout <= 0 or _has_base_station(vr):
        return
    out('No base station yet: SteamVR lists them once a controller has seen them. Hold a controller '
        f'up towards them (waiting up to {timeout:.0f} s)...', flush=True)
    end = clock() + timeout
    while clock() < end:
        vr.poll_vr_events()
        if _has_base_station(vr):
            settle = clock() + 1.0
            while clock() < settle:
                vr.poll_vr_events()
                sleep(0.1)
            return
        sleep(0.1)


def _battery(device):
    try:
        return f'battery {device.get_battery_percent() * 100:3.0f}%'
    except Exception:
        return ''


def describe(vr, roles):
    """One line per device (the null driver's virtual headset is skipped), then missing serials."""
    lines = []
    devices = [d for d in vr.devices.values() if d.device_class != 'HMD']
    for device in sorted(devices, key=lambda d: (d.device_class, d.get_serial())):
        serial = device.get_serial()
        if device.device_class in ('Controller', 'Tracker'):
            battery = _battery(device)
            state = 'tracked' if device.get_pose_quaternion() is not None else 'not tracked'
        else:
            battery, state = '', ''
        lines.append(f'  {device.device_class:<19} {serial:<14} {battery:<13} {state:<12} '
                     f'{roles.get(serial, NOT_CONFIGURED)}')
    if not devices:
        lines.append('  (no device found: are the controllers switched on and the base stations powered?)')
    for serial, role in roles.items():
        if vr.get_device_by_serial(serial) is None:
            lines.append(f'  ! {serial}, the {role}, is not connected')
    return lines


def buzz(device, seconds, sleep=time.sleep, clock=time.monotonic):
    """Vibrate continuously: one 3.9 ms pulse every 10 ms."""
    end = clock() + seconds
    while clock() < end:
        device.trigger_haptic_pulse(3900)
        sleep(0.01)


def controllers_to_buzz(vr, roles, only_serial=None):
    """Controllers in order: right, left, then the others."""
    def rank(device):
        role = roles.get(device.get_serial(), '')
        return (0 if role.startswith('right') else 1 if role.startswith('left') else 2, device.get_serial())

    controllers = [d for d in vr.devices.values() if d.device_class == 'Controller']
    if only_serial:
        controllers = [d for d in controllers if d.get_serial() == only_serial]
    return sorted(controllers, key=rank)


def run(vr, roles, only_serial=None, seconds=1.5, min_wait=2.0, max_wait=8.0, lighthouse_wait=30.0, out=print):
    out('Looking for Vive devices...', flush=True)
    wait_for_devices(vr, list(roles), min_wait, max_wait)
    wait_for_base_stations(vr, lighthouse_wait, out)
    out('SteamVR sees:')
    for line in describe(vr, roles):
        out(line)

    controllers = controllers_to_buzz(vr, roles, only_serial)
    if not controllers:
        out('No controller to vibrate.' if not only_serial else f'Controller {only_serial} is not connected.')
        return 1
    out(f'Vibrating each controller for {seconds:.1f} s:', flush=True)
    for device in controllers:
        serial = device.get_serial()
        out(f'  -> {serial}  ({roles.get(serial, NOT_CONFIGURED)})', flush=True)
        buzz(device, seconds)
        time.sleep(0.5)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='List the Vive devices and vibrate each controller in turn.')
    parser.add_argument('--serial', help='only vibrate this controller')
    parser.add_argument('--seconds', type=float, default=1.5, help='vibration time per controller (s)')
    parser.add_argument('--wait', type=float, default=8.0, help='max time to wait for devices (s)')
    parser.add_argument('--lighthouse-wait', type=float, default=30.0,
                        help='max time to wait for a base station to show up (s, 0 to skip)')
    args = parser.parse_args(argv)

    from ros2_vive_controller.launch_config import config_dir
    from ros2_vive_controller.openvr_class.openvr_class import triad_openvr
    roles = current_roles(st.load(st.path_in(config_dir())))
    vr = triad_openvr()
    return run(vr, roles, args.serial, args.seconds, max_wait=args.wait,
               lighthouse_wait=args.lighthouse_wait)


if __name__ == '__main__':
    sys.exit(main())
