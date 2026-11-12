#!/usr/bin/env python3
"""List the Vive devices SteamVR sees and vibrate each controller in turn.

Standalone: it needs SteamVR (the Docker image) but no ROS nodes and no robot. `make identify`
runs it inside the running vive container, or in a one-off container when none is running.

The roles shown come from the SERIAL_RIGHT, SERIAL_LEFT and REFERENCE_LIGHTHOUSE_SERIAL
environment variables (the .env values, passed in by the Makefile).
"""
import argparse
import os
import sys
import time

ROLE_VARIABLES = ('SERIAL_RIGHT', 'SERIAL_LEFT', 'REFERENCE_LIGHTHOUSE_SERIAL')
NOT_CONFIGURED = 'not in .env'


def roles_from_env(environ=os.environ):
    """{serial: variable name} for the serials set in .env (inline comments ignored)."""
    roles = {}
    for name in ROLE_VARIABLES:
        value = environ.get(name, '').split('#')[0].strip()
        if value:
            roles[value] = name
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
        battery = _battery(device) if device.device_class in ('Controller', 'Tracker') else ''
        lines.append(f'  {device.device_class:<19} {serial:<14} {battery:<13} {roles.get(serial, NOT_CONFIGURED)}')
    if not devices:
        lines.append('  (no device found: are the controllers switched on and the base stations powered?)')
    for serial, name in roles.items():
        if vr.get_device_by_serial(serial) is None:
            lines.append(f'  ! {name}={serial} is not connected')
    return lines


def buzz(device, seconds, sleep=time.sleep, clock=time.monotonic):
    """Vibrate continuously: one 3.9 ms pulse every 10 ms."""
    end = clock() + seconds
    while clock() < end:
        device.trigger_haptic_pulse(3900)
        sleep(0.01)


def controllers_to_buzz(vr, roles, only_serial=None):
    """Controllers in order: right, left, then the others."""
    order = {'SERIAL_RIGHT': 0, 'SERIAL_LEFT': 1}
    controllers = [d for d in vr.devices.values() if d.device_class == 'Controller']
    if only_serial:
        controllers = [d for d in controllers if d.get_serial() == only_serial]
    return sorted(controllers, key=lambda d: (order.get(roles.get(d.get_serial()), 2), d.get_serial()))


def run(vr, roles, only_serial=None, seconds=1.5, min_wait=2.0, max_wait=8.0, out=print):
    out('Looking for Vive devices...', flush=True)
    wait_for_devices(vr, list(roles), min_wait, max_wait)
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
    args = parser.parse_args(argv)

    from ros2_vive_controller.openvr_class.openvr_class import triad_openvr
    vr = triad_openvr()
    return run(vr, roles_from_env(), args.serial, args.seconds, max_wait=args.wait)


if __name__ == '__main__':
    sys.exit(main())
