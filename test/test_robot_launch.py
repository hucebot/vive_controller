"""The launch files run their OpaqueFunction without errors and produce the expected actions."""
import importlib.util
import os
import shutil

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import robot_launch
from ros2_vive_controller import station as st

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(autouse=True)
def config(monkeypatch, tmp_path):
    """A copy of the repository config without this machine's station file or fence."""
    config = tmp_path / 'config'
    shutil.copytree(os.path.join(REPO, 'config', 'robots'), config / 'robots')
    shutil.copy(os.path.join(REPO, 'config', 'vive.params.yaml'), config)
    monkeypatch.setattr(lc, 'SOURCE_CONFIG_DIR', str(config))
    monkeypatch.setattr(robot_launch, '_share_dir', lambda: REPO)
    for variable in st.ENV_VARIABLES.values():
        monkeypatch.delenv(variable, raising=False)
    return config


def _run(launch_file, **arguments):
    """Execute the launch description's arguments and OpaqueFunction; return the actions."""
    spec = importlib.util.spec_from_file_location('launch_file', os.path.join(REPO, 'launch', launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    actions = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            actions += entity.execute(context)
    return actions


def _shape(actions):
    shape = []
    for action in actions:
        if isinstance(action, TimerAction):
            assert [type(a) for a in action.actions] == [Node]
            shape.append('delayed node')
        else:
            shape.append({LogInfo: 'log', Node: 'node'}[type(action)])
    return shape


@pytest.mark.parametrize('launch_file, shape', [
    # log, [left driver], right driver (delayed), [rviz], bridge(s)
    ('franka_single.launch.py', ['log', 'delayed node', 'node', 'node']),
    ('tiago_dual.launch.py', ['log', 'node', 'delayed node', 'node', 'node', 'node']),
    ('tiago_pro.launch.py', ['log', 'node', 'delayed node', 'node', 'node', 'node']),
    ('g1_dual.launch.py', ['log', 'node', 'delayed node', 'node', 'node']),
    ('vive_teleop.launch.py', ['log', 'node', 'delayed node', 'node']),
])
def test_launch_files_build_their_nodes(launch_file, shape):
    assert _shape(_run(launch_file)) == shape


def test_generic_launch_needs_a_known_robot():
    assert _shape(_run('teleop.launch.py', robot='franka', rviz='false')) == ['log', 'delayed node', 'node']
    with pytest.raises(lc.ProfileError, match="Unknown robot 'nobot'"):
        _run('teleop.launch.py', robot='nobot')


def _log(actions):
    return [a for a in actions if isinstance(a, LogInfo)][0].msg[0].text


def test_launch_log_says_where_each_serial_came_from(config, monkeypatch):
    monkeypatch.setenv('SERIAL_RIGHT', 'LHR-ENV')
    monkeypatch.setenv('REFERENCE_LIGHTHOUSE_SERIAL', 'LHB-ENV')
    text = _log(_run('franka_single.launch.py'))
    assert 'Right controller: LHR-ENV (.env)' in text
    assert 'Origin base station: LHB-ENV (.env)' in text
    assert 'Left controller' not in text           # Franka has no left arm

    st.save(st.path_in(str(config)), st.set_fence(
        {'robot': 'franka', 'devices': {'right': 'LHR-APP', 'left': '', 'origin_lighthouse': ''}},
        'right', {'x_min': -1, 'x_max': 1, 'y_min': -1, 'y_max': 1, 'z_min': 0, 'z_max': 1}, 'vive_world'))
    text = _log(_run('franka_single.launch.py'))
    assert 'Right controller: LHR-APP (config/station.yaml)' in text
    assert 'Origin base station: none (config/station.yaml)' in text
    assert "Right arm fence from config/station.yaml: on (frame 'vive_world')" in text

    text = _log(_run('tiago_dual.launch.py'))
    assert 'Origin base station: none (not used by this robot)' in text
