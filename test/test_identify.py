"""Standalone identify tool: device listing with roles, and vibrating each controller."""
from ros2_vive_controller.testing.fake_openvr import CONTROLLER, LIGHTHOUSE, TRACKER, FakeDevice

from ros2_vive_controller import station as st
from ros2_vive_controller.identify import controllers_to_buzz, current_roles, describe, run
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr

ENV = {'SERIAL_RIGHT': 'LHR-RIGHT # Replace with your right controller ',
       'SERIAL_LEFT': 'LHR-ABSENT',
       'REFERENCE_LIGHTHOUSE_SERIAL': 'LHB-ORIGIN  '}


def _setup(vr_system):
    vr_system.plug(1, FakeDevice(CONTROLLER, 'LHR-OTHER'), announce=False)
    vr_system.plug(2, FakeDevice(CONTROLLER, 'LHR-RIGHT'), announce=False)
    vr_system.plug(3, FakeDevice(LIGHTHOUSE, 'LHB-ORIGIN'), announce=False)
    vr_system.plug(4, FakeDevice(TRACKER, 'LHR-TRK'), announce=False)
    return triad_openvr()


def roles_from_env(environ):
    return current_roles(st.empty(), environ)


def test_roles_ignore_inline_comments_and_spaces():
    assert roles_from_env(ENV) == {'LHR-RIGHT': 'right hand (.env)', 'LHR-ABSENT': 'left hand (.env)',
                                   'LHB-ORIGIN': 'origin base station (.env)'}
    assert roles_from_env({'SERIAL_LEFT': ''}) == {}


def test_station_roles_win_over_env():
    station = st.normalize({'devices': {'right': 'LHR-OTHER', 'left': ''}})
    assert current_roles(station, ENV) == {'LHR-OTHER': 'right hand (config/station.yaml)',
                                           'LHB-ORIGIN': 'origin base station (.env)'}


def test_listing_shows_roles_battery_and_missing_serials(vr_system):
    vr = _setup(vr_system)
    text = '\n'.join(describe(vr, roles_from_env(ENV)))
    assert 'LHR-RIGHT' in text and 'right hand (.env)' in text and 'battery  80%' in text
    assert 'LHR-OTHER' in text and 'no role' in text
    assert 'LHB-ORIGIN' in text and 'origin base station' in text
    assert '! LHR-ABSENT, the left hand (.env), is not connected' in text


def test_right_controller_buzzes_first(vr_system):
    vr = _setup(vr_system)
    assert [d.get_serial() for d in controllers_to_buzz(vr, roles_from_env(ENV))] == ['LHR-RIGHT', 'LHR-OTHER']


def test_run_vibrates_every_controller_and_nothing_else(vr_system):
    vr = _setup(vr_system)
    lines = []
    assert run(vr, roles_from_env(ENV), seconds=0.05, min_wait=0.0, max_wait=0.0,
               out=lambda *a, **k: lines.append(a[0])) == 0
    assert vr_system.pulses_for(2) and vr_system.pulses_for(1)
    assert not vr_system.pulses_for(3) and not vr_system.pulses_for(4)
    assert any('-> LHR-RIGHT' in line for line in lines)


def test_run_only_one_serial(vr_system):
    vr = _setup(vr_system)
    run(vr, {}, only_serial='LHR-OTHER', seconds=0.05, min_wait=0.0, max_wait=0.0, out=lambda *a, **k: None)
    assert vr_system.pulses_for(1) and not vr_system.pulses_for(2)


def test_waits_for_a_device_that_shows_up_late(vr_system):
    vr = _setup(vr_system)
    late = FakeDevice(CONTROLLER, 'LHR-ABSENT')
    vr_system.plug(6, late)             # announced, but not polled yet
    lines = []
    run(vr, roles_from_env(ENV), seconds=0.01, min_wait=0.0, max_wait=1.0, out=lambda *a, **k: lines.append(a[0]))
    assert not any('is not connected' in line for line in lines)


def test_no_controller_is_reported(vr_system):
    vr_system.plug(3, FakeDevice(LIGHTHOUSE, 'LHB-ORIGIN'), announce=False)
    vr = triad_openvr()
    lines = []
    assert run(vr, {}, seconds=0.01, min_wait=0.0, max_wait=0.0, out=lambda *a, **k: lines.append(a[0])) == 1
    assert 'No controller to vibrate.' in lines


def test_listing_shows_whether_controllers_are_tracked(vr_system):
    vr = _setup(vr_system)
    vr_system.slots[1].pose_valid = False
    lines = describe(vr, {})
    assert any('LHR-OTHER' in line and 'not tracked' in line for line in lines)
    assert any('LHR-RIGHT' in line and ' tracked' in line and 'not tracked' not in line for line in lines)


def test_waits_for_a_base_station_and_explains_why(vr_system):
    vr_system.plug(2, FakeDevice(CONTROLLER, 'LHR-RIGHT'), announce=False)
    vr = triad_openvr()
    polls = []
    original_poll = vr.poll_vr_events

    def poll():                                                # the controller sees the base station
        polls.append(1)                                        # only after a few polls
        if len(polls) == 5:
            vr_system.plug(3, FakeDevice(LIGHTHOUSE, 'LHB-LATE'))
        original_poll()

    vr.poll_vr_events = poll
    lines = []
    run(vr, {}, seconds=0.01, min_wait=0.0, max_wait=0.0, lighthouse_wait=3.0,
        out=lambda *a, **k: lines.append(a[0]))
    assert any('No base station yet' in line for line in lines)
    assert any('LHB-LATE' in line for line in lines)


def test_gives_up_waiting_for_base_stations(vr_system):
    vr_system.plug(2, FakeDevice(CONTROLLER, 'LHR-RIGHT'), announce=False)
    vr = triad_openvr()
    lines = []
    run(vr, {}, seconds=0.01, min_wait=0.0, max_wait=0.0, lighthouse_wait=0.2, out=lambda *a, **k: lines.append(a[0]))
    assert any('No base station yet' in line for line in lines)
