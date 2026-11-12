"""Pure teleop logic: clutch hysteresis, button gating, staleness and target pose math."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from ros2_vive_controller.teleop_core import Clutch, compute_target, gate_button, is_stale, parse_bool


def _events(clutch, triggers):
    return [clutch.update(t) for t in triggers]


def _old_clutch_events(triggers):
    """The bridge's original logic: engage on reaching exactly 1.0, release on leaving it."""
    events, prev = [], 0.0
    for t in triggers:
        if t == 1.0 and prev != 1.0:
            events.append('engage')
        elif t != 1.0 and prev == 1.0:
            events.append('release')
        else:
            events.append(None)
        prev = t
    return events


def test_default_thresholds():
    clutch = Clutch()
    assert _events(clutch, [0.5, 0.94, 0.95, 0.92, 0.90, 0.89]) == [None, None, 'engage', None, None, 'release']


def test_worn_trigger_that_never_reaches_one_still_engages():
    assert Clutch().update(0.97) == 'engage'


def test_hysteresis_stops_flapping_near_full_pull():
    clutch = Clutch()
    assert _events(clutch, [1.0, 0.97, 1.0, 0.93, 0.99, 1.0]) == ['engage', None, None, None, None, None]
    assert clutch.engaged


def test_thresholds_of_one_reproduce_the_original_behaviour():
    triggers = [0.0, 0.99, 1.0, 1.0, 0.999, 1.0, 0.3, 1.0]
    assert _events(Clutch(1.0, 1.0), triggers) == _old_clutch_events(triggers)


def test_forced_release_needs_the_trigger_let_go_before_engaging_again():
    clutch = Clutch()
    clutch.update(1.0)
    clutch.force_release()
    assert not clutch.engaged
    assert _events(clutch, [1.0, 0.96, 1.0]) == [None, None, None]
    assert _events(clutch, [0.2, 1.0]) == [None, 'engage']


def test_release_threshold_above_engage_is_rejected():
    with pytest.raises(ValueError):
        Clutch(engage_threshold=0.8, release_threshold=0.9)


@pytest.mark.parametrize('key', ['trackpad_x', 'trackpad_y'])
def test_trackpad_axes_need_a_click_when_required(key):
    assert gate_button(key, 0.7, trackpad_pressed=0.0, pressed_required=True) == 0.0
    assert gate_button(key, 0.7, trackpad_pressed=1.0, pressed_required=True) == 0.7
    assert gate_button(key, 0.7, trackpad_pressed=0.0, pressed_required=False) == 0.7


@pytest.mark.parametrize('key', ['trigger', 'grip', 'menu', 'trackpad_touched', 'trackpad_pressed'])
def test_other_buttons_are_never_gated(key):
    """Regression: trackpad_pressed_required used to zero every button topic."""
    assert gate_button(key, 1.0, trackpad_pressed=0.0, pressed_required=True) == 1.0


@pytest.mark.parametrize('value, expected', [
    (True, True), (False, False), ('true', True), ('True', True), ('false', False), ('False', False), ('', False),
])
def test_parse_bool(value, expected):
    assert parse_bool(value) is expected


def test_is_stale():
    assert not is_stale(10.0, 10.2, 0.3)
    assert is_stale(10.0, 10.4, 0.3)
    assert is_stale(None, 10.0, 0.3)
    assert not is_stale(None, 10.0, 0.0)     # disabled


def test_translation_is_relative_to_the_clutch_point():
    position, _ = compute_target(
        ee_position=(0.5, 0.0, 1.0), joy_reference=(2.0, 3.0, 1.0), joy_position=(2.1, 2.8, 1.05),
        joy_orientation=(0.0, 0.0, 0.0, 1.0), rotation_offset_deg=[0.0, 0.0, 0.0])
    assert position == pytest.approx((0.6, -0.2, 1.05))


def test_orientation_without_offset_is_copied():
    q = (0.1, 0.2, 0.3, 0.927)
    _, orientation = compute_target((0, 0, 0), (0, 0, 0), (0, 0, 0), q, [0.0, 0.0, 0.0])
    assert orientation == q


def test_orientation_offset_matches_the_original_formula():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = R.random(random_state=rng).as_quat()
        offset = list(rng.uniform(-180, 180, 3))
        _, orientation = compute_target((0, 0, 0), (0, 0, 0), (0, 0, 0), tuple(q), offset)
        expected = (R.from_quat([q[0], q[1], q[2], q[3]]) * R.from_euler('xyz', offset, degrees=True)).as_quat()
        assert np.allclose(orientation, expected)
