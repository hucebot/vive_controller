"""Launch descriptions built from a robot profile (config/robots/<robot>.yaml).

The robot launch files (franka_single.launch.py, ...) and teleop.launch.py call generate();
vive_teleop.launch.py (`make drivers`: drivers and RViz, no robot) calls generate_drivers().

The nodes are first described as plain dicts (robot_nodes(), drivers_nodes()) so tests can check
them, then turned into launch actions.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import station as st

# The right driver starts a little after the left one, as it always has.
RIGHT_DRIVER_DELAY = 3.0

DEVICE_ARGUMENTS = (
    ('serial_left', 'Left controller serial (empty: config/station.yaml, then SERIAL_LEFT in .env)'),
    ('serial_right', 'Right controller serial (empty: config/station.yaml, then SERIAL_RIGHT in .env)'),
    ('reference_lighthouse_serial',
     'Base station used as the origin frame (empty: config/station.yaml, then REFERENCE_LIGHTHOUSE_SERIAL)'),
)
DEVICE_LABELS = {'serial_left': 'Left controller', 'serial_right': 'Right controller',
                 'reference_lighthouse_serial': 'Origin base station'}


def _driver_specs(sides, devices, linear_scale, config, fences):
    files = lc.driver_parameter_files(config)
    specs = []
    for side in lc.SIDES:
        if side in sides:
            params = lc.driver_parameters(side, devices.get(f'serial_{side}', ''),
                                          devices.get('reference_lighthouse_serial', ''), linear_scale,
                                          fences.get(side))
            specs.append({'package': lc.PACKAGE, 'executable': 'vive_node', 'name': f'driver_{side}',
                          'namespace': f'vive/{side}', 'parameters': [*files, params],
                          'delay': RIGHT_DRIVER_DELAY if side == 'right' else 0.0})
    return specs


def _rviz_spec(share_dir):
    return {'package': 'rviz2', 'executable': 'rviz2', 'name': 'rviz2',
            'arguments': ['-d', os.path.join(share_dir, 'rviz', 'view_vive.rviz')]}


def robot_nodes(profile, devices, config, share_dir, linear_scale='', clutch_reference='', rviz=None,
                fences=None):
    """Drivers, RViz and one teleop bridge per arm of the profile.

    devices: serials by launch argument name (see launch_config.resolve_devices).
    fences: workspace fence per side from the station file.
    """
    specs = _driver_specs(profile['arms'], devices, linear_scale or profile['linear_scale'], config, fences or {})
    if profile['rviz'] if rviz is None else rviz:
        specs.append(_rviz_spec(share_dir))
    for side in lc.SIDES:
        if side in profile['arms']:
            arm = profile['arms'][side]
            specs.append({'package': lc.PACKAGE, 'executable': 'teleop_bridge_node',
                          'name': lc.bridge_name(side, arm),
                          'parameters': [lc.bridge_parameters(side, arm, clutch_reference)]})
    return specs


def drivers_nodes(sides, devices, config, share_dir, linear_scale=1.0, rviz=True, fences=None):
    specs = _driver_specs(sides, devices, linear_scale, config, fences or {})
    if rviz:
        specs.append(_rviz_spec(share_dir))
    return specs


def to_actions(specs):
    actions = []
    for spec in specs:
        spec = dict(spec)
        delay = spec.pop('delay', 0.0)
        node = Node(output='screen', **spec)
        actions.append(TimerAction(period=delay, actions=[node]) if delay else node)
    return actions


# --- Launch plumbing ---------------------------------------------------------------------------

def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _optional_bool(value):
    """None for an empty argument (use the default), else the boolean."""
    if value == '':
        return None
    return value.lower() in ('1', 'true', 'yes', 'on')


def _share_dir():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(lc.PACKAGE)


def _station_and_devices(context, config, use_origin_lighthouse=True):
    station = st.load(st.path_in(config))
    arguments = {name: _arg(context, name) for name, _ in DEVICE_ARGUMENTS}
    devices, sources = lc.resolve_devices(arguments, station, os.environ, use_origin_lighthouse)
    return station, devices, sources


def _device_log(devices, sources, sides):
    lines = []
    for name, _ in DEVICE_ARGUMENTS:
        side = name[len('serial_'):] if name.startswith('serial_') else None
        if side and side not in sides:
            continue
        lines.append(f"  {DEVICE_LABELS[name]}: {devices[name] or 'none'} ({sources[name]})")
    return lines


def _fence_log(station, sides):
    return [f"  {side.capitalize()} arm fence from config/station.yaml: "
            f"{'on' if station['workspace'][side]['enabled'] else 'off'}"
            f" (frame '{station['workspace'][side]['frame_id']}')"
            for side in lc.SIDES if side in sides and side in station['workspace']]


def _robot_setup(context):
    share_dir = _share_dir()
    config = lc.config_dir(share_dir)
    profile = lc.load_profile(_arg(context, 'robot'), config)
    station, devices, sources = _station_and_devices(context, config, profile['use_origin_lighthouse'])
    linear_scale = _arg(context, 'linear_scale')
    specs = robot_nodes(profile, devices, config, share_dir, linear_scale=linear_scale,
                        clutch_reference=_arg(context, 'clutch_reference'), rviz=_optional_bool(_arg(context, 'rviz')),
                        fences=station['workspace'])
    summary = '\n'.join([
        f"Robot profile '{profile['name']}': {profile['description']} "
        f"(arms: {', '.join(profile['arms'])}, linear scale {float(linear_scale or profile['linear_scale'])})",
        *_device_log(devices, sources, profile['arms']),
        *_fence_log(station, profile['arms']),
    ])
    return [LogInfo(msg=summary), *to_actions(specs)]


def generate(robot=''):
    """Drivers, teleop bridges and RViz for a robot profile."""
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value=robot, description='Robot profile in config/robots/'),
        *[DeclareLaunchArgument(name, default_value='', description=text) for name, text in DEVICE_ARGUMENTS],
        DeclareLaunchArgument('linear_scale', default_value='',
                              description="Translation scale (empty: the profile's linear_scale)"),
        DeclareLaunchArgument('clutch_reference', default_value='',
                              description="'tf' or 'last_command' (empty: the bridge default, tf)"),
        DeclareLaunchArgument('rviz', default_value='', description="Start RViz (empty: the profile's rviz)"),
        OpaqueFunction(function=_robot_setup),
    ])


def _drivers_setup(context):
    share_dir = _share_dir()
    config = lc.config_dir(share_dir)
    station, devices, sources = _station_and_devices(context, config)
    sides = ('right',) if _optional_bool(_arg(context, 'only_right')) else lc.SIDES
    rviz = _optional_bool(_arg(context, 'rviz'))
    specs = drivers_nodes(sides, devices, config, share_dir, linear_scale=_arg(context, 'linear_scale') or 1.0,
                          rviz=rviz is not False, fences=station['workspace'])
    summary = '\n'.join(['Vive drivers only (no robot):', *_device_log(devices, sources, sides),
                         *_fence_log(station, sides)])
    return [LogInfo(msg=summary), *to_actions(specs)]


def generate_drivers():
    """Only the Vive drivers (and RViz): no robot, no teleop bridge."""
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        *[DeclareLaunchArgument(name, default_value='', description=text) for name, text in DEVICE_ARGUMENTS],
        DeclareLaunchArgument('only_right', default_value='false',
                              description='If true, the left controller node is skipped.'),
        DeclareLaunchArgument('linear_scale', default_value='1.0', description='Translation scaling factor.'),
        OpaqueFunction(function=_drivers_setup),
    ])


def calibration_nodes(side, devices, config, share_dir):
    """Driver for one controller, calibration_node and RViz (`make calibrate`)."""
    params = lc.driver_parameters(side, devices[f'serial_{side}'], devices['reference_lighthouse_serial'], 1.0)
    return [
        {'package': lc.PACKAGE, 'executable': 'vive_node', 'name': f'calibration_driver_{side}',
         'namespace': f'vive/{side}', 'parameters': [os.path.join(config, 'vive.params.yaml'), params]},
        {'package': lc.PACKAGE, 'executable': 'calibration_node', 'name': 'calibration_node',
         'parameters': [{'station_file': st.path_in(config), 'side': side}]},
        {'package': 'rviz2', 'executable': 'rviz2', 'name': 'rviz2',
         'arguments': ['-d', os.path.join(share_dir, 'rviz', 'calibration.rviz')]},
    ]


def _calibration_setup(context):
    share_dir = _share_dir()
    config = lc.config_dir(share_dir)
    side = _arg(context, 'side')
    if side not in lc.SIDES:
        raise ValueError(f"side must be one of {', '.join(lc.SIDES)}, not '{side}'")
    station = st.load(st.path_in(config))
    # Record in the frame the robot will run in: its profile decides on the origin base station.
    robot = _arg(context, 'robot') or station['robot']
    use_origin = lc.load_profile(robot, config)['use_origin_lighthouse'] if robot else True
    arguments = {name: _arg(context, name) for name, _ in DEVICE_ARGUMENTS}
    arguments[f'serial_{side}'] = _arg(context, 'serial')
    devices, sources = lc.resolve_devices(arguments, station, os.environ, use_origin)
    summary = '\n'.join([f"Painting the {side} arm's workspace" + (f" for robot '{robot}'" if robot else '') + ':',
                         *_device_log(devices, sources, (side,))])
    return [LogInfo(msg=summary), *to_actions(calibration_nodes(side, devices, config, share_dir))]


def generate_calibration():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right', description='Arm whose workspace is painted'),
        DeclareLaunchArgument('serial', default_value='',
                              description="That arm's controller (empty: config/station.yaml, then .env)"),
        DeclareLaunchArgument('reference_lighthouse_serial', default_value='',
                              description='Origin base station (empty: config/station.yaml, then .env)'),
        DeclareLaunchArgument('robot', default_value='',
                              description="Robot profile, which decides on the origin frame (empty: the station's)"),
        OpaqueFunction(function=_calibration_setup),
    ])
