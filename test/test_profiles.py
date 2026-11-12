"""Robot profiles must start exactly the nodes the hand-written robot launch files started.

EXPECTED_BRIDGES is copied from franka_single/tiago_dual/tiago_pro/g1_dual.launch.py as they were
before the profiles (default argument values, as `make <robot>` ran them).
"""
import os

import pytest

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import station as st
from ros2_vive_controller.robot_launch import RIGHT_DRIVER_DELAY, drivers_nodes, robot_nodes

REPO_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
SHARE = '/share'


def _buttons(side, **overrides):
    topics = {f'{b}_topic': f'/vive/{side}/{b}' for b in lc.BUTTONS}
    topics.update({f'{k}_topic': v for k, v in overrides.items()})
    return topics


def _bridge(side, **params):
    return dict({'pose_topic': f'/vive/{side}/pose', 'button_state_topic': f'/vive/{side}/joint_states'}, **params)


EXPECTED_BRIDGES = {
    'franka': {
        'teleop_bridge_franka_tcp': _bridge(
            'right', output_topic='/cartesian_impedance/equilibrium_pose', publish_frequency=30.0,
            target_frame='panda_hand_tcp', reference_frame='panda_link0', rotation_offset=[180.0, 0.0, 0.0],
            **_buttons('right', grip='/vive/right/grip_button', menu='/panda_gripper/gripper_command')),
    },
    'tiago': {
        f'teleop_bridge_{s}': _bridge(
            s, output_topic=f'/vive/{s}/output_pose', publish_frequency=30.0,
            target_frame=f'opensot/gripper_{s}_grasping_frame', reference_frame='opensot/base_link',
            rotation_offset=[0.0, 0.0, 0.0], **_buttons(s, menu=f'/vive/{s}/gripper'))
        for s in ('left', 'right')
    },
    'tiago_pro': {
        f'teleop_bridge_{s}': _bridge(
            s, output_topic=f'/vive/{s}/output_pose', publish_frequency=30.0,
            target_frame=f'opensot/gripper_{s}_grasping_link', reference_frame='opensot/base_link',
            rotation_offset=[180.0, 0.0, 0.0], **_buttons(s, menu=f'/vive/{s}/gripper'))
        for s in ('left', 'right')
    },
    'g1': {
        f'teleop_bridge_{s}': _bridge(
            s, output_topic=f'/g1pilot/{s}_hand/pose_ref', publish_frequency=-1.0,
            target_frame=f'{s}_hand_point_contact', reference_frame='world',
            trackpad_pressed_required=True,   # was the string 'true'; the bridge accepts both
            **_buttons(s, trackpad_x=f'/g1pilot/{s}_hand/dx3/action'))
        for s in ('left', 'right')
    },
}
# Scale under `make <robot>` (compose defaults) and whether RViz was started.
EXPECTED_SCALE = {'franka': 2.0, 'tiago': 1.0, 'tiago_pro': 1.0, 'g1': 1.0}
EXPECTED_RVIZ = {'franka': True, 'tiago': True, 'tiago_pro': True, 'g1': False}
# Only Franka was given the reference lighthouse.
USES_LIGHTHOUSE = {'franka': True, 'tiago': False, 'tiago_pro': False, 'g1': False}

DEVICES = {'serial_left': 'LHR-L', 'serial_right': 'LHR-R', 'reference_lighthouse_serial': 'LHB-O'}
# What `make <robot>` passes when there is no station file: the .env serials.
ENV = {'SERIAL_LEFT': 'LHR-L', 'SERIAL_RIGHT': 'LHR-R # comment', 'REFERENCE_LIGHTHOUSE_SERIAL': 'LHB-O'}


def _nodes(robot, **kwargs):
    profile = lc.load_profile(robot, REPO_CONFIG)
    devices, _ = lc.resolve_devices({}, st.empty(), ENV, profile['use_origin_lighthouse'])
    return robot_nodes(profile, devices, REPO_CONFIG, SHARE, **kwargs)


def _by_name(specs):
    return {spec['name']: spec for spec in specs}


def test_every_compose_robot_has_a_profile():
    assert lc.list_profiles(REPO_CONFIG) == sorted(EXPECTED_BRIDGES)


@pytest.mark.parametrize('robot', sorted(EXPECTED_BRIDGES))
def test_bridges_match_the_old_launch_files(robot):
    specs = _nodes(robot)
    bridges = {s['name']: s['parameters'] for s in specs if s['executable'] == 'teleop_bridge_node'}
    assert bridges == {name: [params] for name, params in EXPECTED_BRIDGES[robot].items()}


@pytest.mark.parametrize('robot', sorted(EXPECTED_BRIDGES))
def test_drivers_match_the_old_launch_files(robot):
    specs = _by_name(_nodes(robot))
    sides = [s for s in ('left', 'right') if f'driver_{s}' in specs]
    assert sides == (['right'] if robot == 'franka' else ['left', 'right'])
    for side in sides:
        driver = specs[f'driver_{side}']
        assert driver['executable'] == 'vive_node'
        assert driver['namespace'] == f'vive/{side}'
        assert driver['delay'] == (RIGHT_DRIVER_DELAY if side == 'right' else 0.0)
        assert driver['parameters'][0] == os.path.join(REPO_CONFIG, 'vive.params.yaml')
        assert driver['parameters'][-1] == {
            'side': side, 'linear_scale': EXPECTED_SCALE[robot], 'serial': DEVICES[f'serial_{side}'],
            'reference_lighthouse_serial': 'LHB-O' if USES_LIGHTHOUSE[robot] else ''}
    assert ('rviz2' in specs) == EXPECTED_RVIZ[robot]


def test_command_line_values_override_the_profile():
    specs = _by_name(_nodes('franka', linear_scale='1.5', clutch_reference='last_command', rviz=False))
    assert specs['driver_right']['parameters'][-1]['linear_scale'] == 1.5
    assert specs['teleop_bridge_franka_tcp']['parameters'][0]['clutch_reference'] == 'last_command'
    assert 'rviz2' not in specs


def test_drivers_only_launch():
    specs = _by_name(drivers_nodes(('left', 'right'), DEVICES, REPO_CONFIG, SHARE))
    assert sorted(specs) == ['driver_left', 'driver_right', 'rviz2']
    assert specs['driver_left']['parameters'][-1]['reference_lighthouse_serial'] == 'LHB-O'
    specs = _by_name(drivers_nodes(('right',), DEVICES, REPO_CONFIG, SHARE, rviz=False))
    assert sorted(specs) == ['driver_right']


def test_calibrated_fence_is_loaded_after_the_defaults(tmp_path):
    (tmp_path / 'vive.params.yaml').write_text('{}')
    assert lc.driver_parameter_files(str(tmp_path)) == [str(tmp_path / 'vive.params.yaml')]
    (tmp_path / 'workspace.yaml').write_text('{}')
    assert lc.driver_parameter_files(str(tmp_path))[-1] == str(tmp_path / 'workspace.yaml')


# --- validation ---------------------------------------------------------------------------------

def _write_profile(tmp_path, name, text):
    (tmp_path / 'robots').mkdir(exist_ok=True)
    (tmp_path / 'robots' / f'{name}.yaml').write_text(text)


ARM = 'output_topic: /out\n    reference_frame: base\n    target_frame: tool\n'


def test_unknown_robot_lists_the_profiles(tmp_path):
    _write_profile(tmp_path, 'mybot', f'arms:\n  right:\n    {ARM}')
    with pytest.raises(lc.ProfileError, match="Unknown robot 'nobot'.*mybot"):
        lc.load_profile('nobot', str(tmp_path))


@pytest.mark.parametrize('text, message', [
    (f'arms:\n  right:\n    {ARM}    target_fram: x\n', 'unknown key.*target_fram'),
    (f'arms:\n  middle:\n    {ARM}', 'must be one of'),
    ('arms:\n  right:\n    output_topic: /out\n', 'missing target_frame, reference_frame'),
    (f'arms:\n  right:\n    {ARM}    buttons:\n      trigerr: /t\n', 'buttons: unknown key.*trigerr'),
    (f'rvis: true\narms:\n  right:\n    {ARM}', 'unknown key.*rvis'),
    ('rviz: true\n', "'arms' must list"),
])
def test_profile_mistakes_are_reported(tmp_path, text, message):
    _write_profile(tmp_path, 'mybot', text)
    with pytest.raises(lc.ProfileError, match=message):
        lc.load_profile('mybot', str(tmp_path))


def test_minimal_profile_gets_defaults(tmp_path):
    _write_profile(tmp_path, 'mybot', f'arms:\n  left:\n    {ARM}')
    profile = lc.load_profile('mybot', str(tmp_path))
    assert profile['rviz'] is True and profile['linear_scale'] == 1.0
    assert profile['use_origin_lighthouse'] is False
    specs = _by_name(robot_nodes(profile, DEVICES, str(tmp_path), SHARE))
    assert specs['teleop_bridge_left']['parameters'][0]['grip_topic'] == '/vive/left/grip'


def test_inline_env_comments_are_dropped():
    assert lc.clean_value('LHR-1234 # right hand') == 'LHR-1234'
    assert lc.clean_value(None) == ''
