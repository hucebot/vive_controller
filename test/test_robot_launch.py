"""The launch files run their OpaqueFunction without errors and produce the expected actions."""
import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import robot_launch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(autouse=True)
def repo_config(monkeypatch):
    monkeypatch.setattr(lc, 'SOURCE_CONFIG_DIR', os.path.join(REPO, 'config'))
    monkeypatch.setattr(robot_launch, '_share_dir', lambda: REPO)


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
    ('vive_teleop.launch.py', ['node', 'delayed node', 'node']),
])
def test_launch_files_build_their_nodes(launch_file, shape):
    assert _shape(_run(launch_file)) == shape


def test_generic_launch_needs_a_known_robot():
    assert _shape(_run('teleop.launch.py', robot='franka', rviz='false')) == ['log', 'delayed node', 'node']
    with pytest.raises(lc.ProfileError, match="Unknown robot 'nobot'"):
        _run('teleop.launch.py', robot='nobot')
