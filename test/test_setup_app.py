"""Setup app: assigning hands by trigger, painting fences in the robot's frame, saving the station."""
import json
import os
import shutil
import threading
import urllib.error
import urllib.request

import pytest
import yaml

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import station as st
from ros2_vive_controller.frames import pose_in_origin
from ros2_vive_controller.openvr_class.openvr_class import convert_to_quaternion, triad_openvr
from ros2_vive_controller.setup_app.core import CommandError, SetupApp
from ros2_vive_controller.setup_app.server import make_server
from ros2_vive_controller.testing.fake_openvr import (CONTROLLER, LIGHTHOUSE, FakeDevice,
                                                      TrackingResult_Running_OutOfRange, translation_matrix)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENV = {'SERIAL_RIGHT': 'LHR-A # right', 'REFERENCE_LIGHTHOUSE_SERIAL': 'LHB-O'}


@pytest.fixture
def config(tmp_path):
    shutil.copytree(os.path.join(REPO, 'config', 'robots'), tmp_path / 'robots')
    shutil.copy(os.path.join(REPO, 'config', 'vive.params.yaml'), tmp_path)
    return str(tmp_path)


class Rig:
    """Fake SteamVR with two controllers (A in slot 1, B in slot 2) and a base station (slot 3)."""

    def __init__(self, vr_system, config, **kwargs):
        self.system = vr_system
        self.a = vr_system.plug(1, FakeDevice(CONTROLLER, 'LHR-A'), announce=False)
        self.b = vr_system.plug(2, FakeDevice(CONTROLLER, 'LHR-B'), announce=False)
        self.lighthouse = vr_system.plug(3, FakeDevice(LIGHTHOUSE, 'LHB-O', position=(0.0, 2.0, -1.0)),
                                         announce=False)
        self.now = 100.0
        self.app = SetupApp(triad_openvr(), config, environ=kwargs.pop('environ', ENV),
                            clock=lambda: self.now, **kwargs)

    def step(self, n=1):
        for _ in range(n):
            self.now += 0.06          # > STATE_PERIOD, so every step publishes a snapshot
            self.app.step()

    def do(self, command, **kwargs):
        return self.app.execute(command, **kwargs)


@pytest.fixture
def rig(vr_system, config):
    return Rig(vr_system, config)


def test_first_start_is_prefilled_from_env(rig):
    rig.step()
    state = rig.app.state()
    assert state['draft']['devices'] == {'right': 'LHR-A', 'left': '', 'origin_lighthouse': 'LHB-O'}
    assert state['dirty'] and not state['station_exists']
    assert 'No robot selected.' in state['warnings']
    roles = {d['serial']: d['role'] for d in state['devices']}
    assert roles == {'LHR-A': 'right', 'LHR-B': '', 'LHB-O': 'origin_lighthouse'}
    assert sorted(state['robots']) == ['franka', 'g1', 'tiago', 'tiago_pro']


def test_assign_a_hand_by_pulling_its_trigger(rig):
    rig.do('assign', side='left')
    rig.step()                                    # both triggers seen released: armed
    rig.b.trigger = 1.0
    rig.step()
    assert rig.app.draft['devices']['left'] == 'LHR-B'
    assert rig.app.assigning is None
    assert rig.system.pulses_for(2)               # confirmation buzz
    assert not rig.system.pulses_for(1)


def test_a_trigger_already_held_does_not_count(rig):
    rig.b.trigger = 1.0
    rig.do('assign', side='left')
    rig.step(3)
    assert rig.app.draft['devices']['left'] == ''
    rig.b.trigger = 0.0
    rig.step()
    rig.b.trigger = 1.0
    rig.step()
    assert rig.app.draft['devices']['left'] == 'LHR-B'


def test_assigning_the_other_hands_controller_swaps_them(rig):
    rig.do('set_device', role='left', serial='LHR-B')
    rig.do('assign', side='right')
    rig.step()
    rig.b.trigger = 1.0
    rig.step()
    assert rig.app.draft['devices']['right'] == 'LHR-B'
    assert rig.app.draft['devices']['left'] == 'LHR-A'


def _driver_pose(raw_position):
    """The pose the driver publishes for a device at this raw OpenVR (y-up) position."""
    return convert_to_quaternion(translation_matrix(*raw_position))


def _put(device, position):
    """Move a device so that the driver sees it at this (z-up) position."""
    x, y, z = position
    device.move_to(y, z, x)
    assert _driver_pose((y, z, x))[:3] == pytest.approx(list(position))


def _paint(rig, device, positions):
    for position in positions:
        _put(device, position)
        rig.step()


BOX_POINTS = [(x, y, z) for x in (-0.2, 0.0, 0.2) for y in (-0.1, 0.1) for z in (0.8, 1.0, 1.2)]


def test_paint_a_fence_in_the_origin_frame(rig):
    rig.do('select_robot', robot='franka')        # Franka uses the origin base station
    rig.step()
    rig.do('paint_start', side='right')
    rig.a.trigger = 1.0
    _paint(rig, rig.a, BOX_POINTS)
    rig.a.trigger = 0.0
    _paint(rig, rig.a, [(5.0, 5.0, 5.0)])         # trigger released: not recorded
    rig.a.trigger = 1.0
    rig.a.tracking_result = TrackingResult_Running_OutOfRange
    _paint(rig, rig.a, [(-5.0, -5.0, -5.0)])      # tracking lost: not recorded
    assert len(rig.app.points['right']) == len(BOX_POINTS)

    rig.do('paint_stop')
    fence = rig.app.draft['workspace']['right']
    assert fence['enabled'] and fence['frame_id'] == 'lighthouse_origin'
    lighthouse = _driver_pose((0.0, 2.0, -1.0))
    in_origin = [pose_in_origin(_driver_pose((y, z, x)), lighthouse)[:3] for x, y, z in BOX_POINTS]
    for i, axis in enumerate('xyz'):
        assert fence[f'{axis}_min'] == pytest.approx(min(p[i] for p in in_origin) - 0.05, abs=1e-3)
        assert fence[f'{axis}_max'] == pytest.approx(max(p[i] for p in in_origin) + 0.05, abs=1e-3)

    rig.do('set_margin', margin=0.1)
    assert rig.app.draft['workspace']['right']['x_max'] == pytest.approx(fence['x_max'] + 0.05, abs=1e-3)


def test_robot_without_origin_paints_in_the_world_frame(rig):
    rig.do('select_robot', robot='tiago')
    rig.do('set_device', role='left', serial='LHR-B')
    rig.step()
    rig.do('paint_start', side='left')
    rig.b.trigger = 1.0
    _paint(rig, rig.b, BOX_POINTS)
    rig.do('paint_stop')
    fence = rig.app.draft['workspace']['left']
    assert fence['frame_id'] == 'vive_world'
    assert fence['x_min'] == pytest.approx(-0.25) and fence['z_max'] == pytest.approx(1.25)


def test_controller_reports_its_position_against_the_fence(rig):
    rig.do('select_robot', robot='tiago')
    rig.app.draft = st.set_fence(rig.app.draft, 'right', {'x_min': -1, 'x_max': 1, 'y_min': -1, 'y_max': 1,
                                                          'z_min': 0, 'z_max': 2}, 'vive_world')
    for position, expected in (((0, 0, 1), 'inside'), ((0.9, 0, 1), 'warning'), ((3, 0, 1), 'outside')):
        _put(rig.a, position)
        rig.step()
        assert [d['fence'] for d in rig.app.state()['devices'] if d['serial'] == 'LHR-A'] == [expected]


def test_too_few_points_is_refused(rig):
    rig.step()
    rig.do('paint_start', side='right')
    rig.a.trigger = 1.0
    _paint(rig, rig.a, BOX_POINTS[:3])
    with pytest.raises(CommandError, match='Only 3 points'):
        rig.do('paint_stop')
    assert 'right' not in rig.app.draft['workspace']


def test_painting_needs_a_connected_controller(rig):
    rig.step()
    with pytest.raises(CommandError, match='Assign a connected controller to the left hand'):
        rig.do('paint_start', side='left')


def test_painting_waits_for_the_origin_base_station(vr_system, config):
    rig = Rig(vr_system, config)
    rig.lighthouse.pose_valid = False
    rig.step()
    with pytest.raises(CommandError, match='LHB-O: position not known yet'):
        rig.do('paint_start', side='right')
    assert any('LHB-O: position not known yet' in w for w in rig.app.state()['warnings'])


def test_trigger_actions_are_off_while_teleop_runs(vr_system, config):
    rig = Rig(vr_system, config, attached_to='ros2_vive_franka')
    rig.step()
    for command, kwargs in (('assign', {'side': 'left'}), ('paint_start', {'side': 'right'})):
        with pytest.raises(CommandError, match='Teleop running in ros2_vive_franka'):
            rig.do(command, **kwargs)
    rig.do('set_device', role='left', serial='LHR-B')             # drop-downs still work
    assert rig.app.draft['devices']['left'] == 'LHR-B'
    assert Rig(vr_system, config, attached_to='ros2_vive_drivers').app.teleop_running == ''


def test_save_writes_the_station_file_the_launch_reads(rig, config):
    rig.do('select_robot', robot='franka')
    rig.do('select_origin', serial='')              # no origin, although .env names one
    rig.do('save')
    rig.step()
    assert not rig.app.state()['dirty']
    station = st.load(st.path_in(config))
    assert station['robot'] == 'franka'
    devices, sources = lc.resolve_devices({}, station, ENV)
    assert devices['serial_right'] == 'LHR-A' and sources['serial_right'] == st.SOURCE_STATION
    assert devices['reference_lighthouse_serial'] == ''     # the app's choice wins over .env

    rig.do('set_device', role='left', serial='LHR-B')
    rig.step()
    assert rig.app.state()['dirty']
    rig.do('revert')
    assert rig.app.draft['devices']['left'] == ''


def test_warnings_for_a_fence_in_the_wrong_frame_and_a_thin_fence(rig):
    rig.do('select_robot', robot='franka')
    rig.app.draft = st.set_fence(rig.app.draft, 'right', {'x_min': 0, 'x_max': 0.2, 'y_min': 0, 'y_max': 1,
                                                          'z_min': 0, 'z_max': 1}, 'vive_world')
    rig.step()
    warnings = ' '.join(rig.app.state()['warnings'])
    assert "Right fence is in 'vive_world', the robot runs in 'lighthouse_origin'" in warnings
    assert 'under 2 x the warning band (0.15 m) along x' in warnings


def test_old_workspace_file_is_mentioned(vr_system, config):
    with open(os.path.join(config, 'workspace.yaml'), 'w') as f:
        yaml.safe_dump({'/**': {'ros__parameters': {'workspace': {
            'enabled': True, 'frame_id': 'lighthouse_origin', 'x_min': -1, 'x_max': 1, 'y_min': -1,
            'y_max': 1, 'z_min': 0, 'z_max': 2}}}}, f)
    rig = Rig(vr_system, config)
    rig.do('select_robot', robot='franka')
    rig.step()
    assert rig.app.state()['legacy_fence']['frame_id'] == 'lighthouse_origin'
    assert any('old config/workspace.yaml fence' in w for w in rig.app.state()['warnings'])


def test_buzz_and_bad_requests(rig):
    rig.do('buzz', serial='LHR-B', seconds=0.1)
    rig.step()
    assert rig.system.pulses_for(2)
    with pytest.raises(CommandError, match='not connected'):
        rig.do('buzz', serial='LHR-NONE')
    with pytest.raises(CommandError, match='Bad request'):
        rig.do('buzz', serial='LHR-B', volume=11)
    with pytest.raises(CommandError, match='Unknown command'):
        rig.do('reboot')
    with pytest.raises(CommandError, match='not a connected tracking reference'):
        rig.do('select_origin', serial='LHR-A')


# --- HTTP ---------------------------------------------------------------------------------------

@pytest.fixture
def served(rig):
    rig.app.clock = __import__('time').monotonic
    rig.app.start()
    server = make_server(rig.app, '127.0.0.1', 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield rig, f'http://127.0.0.1:{server.server_address[1]}'
    server.shutdown()
    server.server_close()
    rig.app.stop()


def _request(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_http_api(served):
    rig, base = served
    status, _ = _request(f'{base}/api/select/robot', {'robot': 'tiago'})
    assert status == 200
    status, reply = _request(f'{base}/api/assign', {'side': 'right'})
    assert status == 200 and 'RIGHT hand' in reply['message']
    status, reply = _request(f'{base}/api/paint/start', {'side': 'banana'})
    assert status == 400 and 'Unknown arm' in reply['message']
    status, reply = _request(f'{base}/api/selfdestruct', {})
    assert status == 400
    status, state = _request(f'{base}/api/state')
    assert status == 200 and state['draft']['robot'] == 'tiago' and state['assigning'] == 'right'
    status, points = _request(f'{base}/api/points')
    assert status == 200 and set(points['points']) == {'left', 'right'}
    status, reply = _request(f'{base}/api/nothing-here')
    assert status == 404
    with urllib.request.urlopen(f'{base}/', timeout=5) as response:
        page = response.read().decode()
    assert response.headers['Content-Type'].startswith('text/html')
    assert 'VIVE SETUP' in page and "fetch('/api/state'" in page
