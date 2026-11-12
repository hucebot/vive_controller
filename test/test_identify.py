"""Standalone identify tool: device listing with .env roles, and vibrating each controller."""
from fake_openvr import CONTROLLER, LIGHTHOUSE, TRACKER, FakeDevice

from ros2_vive_controller.identify import controllers_to_buzz, describe, roles_from_env, run
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


def test_roles_ignore_inline_comments_and_spaces():
    assert roles_from_env(ENV) == {'LHR-RIGHT': 'SERIAL_RIGHT', 'LHR-ABSENT': 'SERIAL_LEFT',
                                   'LHB-ORIGIN': 'REFERENCE_LIGHTHOUSE_SERIAL'}
    assert roles_from_env({'SERIAL_LEFT': ''}) == {}


def test_listing_shows_roles_battery_and_missing_serials(vr_system):
    vr = _setup(vr_system)
    text = '\n'.join(describe(vr, roles_from_env(ENV)))
    assert 'LHR-RIGHT' in text and 'SERIAL_RIGHT' in text and 'battery  80%' in text
    assert 'LHR-OTHER' in text and 'not in .env' in text
    assert 'LHB-ORIGIN' in text and 'REFERENCE_LIGHTHOUSE_SERIAL' in text
    assert '! SERIAL_LEFT=LHR-ABSENT is not connected' in text


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
