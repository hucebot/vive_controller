"""vive_node against a fake SteamVR runtime: disconnects, wrong serials, reference lighthouse."""
import time

import pytest
from fake_openvr import CONTROLLER, LIGHTHOUSE, FakeDevice

rclpy = pytest.importorskip('rclpy')
from std_srvs.srv import Trigger  # noqa: E402

# A box large enough that the workspace fence never interferes with these tests.
WIDE_WORKSPACE = {
    'workspace.x_min': -100.0, 'workspace.x_max': 100.0,
    'workspace.y_min': -100.0, 'workspace.y_max': 100.0,
    'workspace.z_min': -100.0, 'workspace.z_max': 100.0,
    'workspace.padding': 0.0,
}


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class LogRecorder:
    def __init__(self):
        self.lines = []

    def _log(self, level):
        return lambda msg, *a, **k: self.lines.append((level, msg))

    def __getattr__(self, name):
        if name in ('debug', 'info', 'warn', 'warning', 'error', 'fatal'):
            return self._log(name)
        raise AttributeError(name)

    def text(self, level=None):
        return '\n'.join(m for lvl, m in self.lines if level is None or lvl == level)


@pytest.fixture
def driver(vr_system):
    """Factory for one ViveDriverNode per test, with publishers and logger recorded."""
    created = []

    def make(**params):
        args = ['--ros-args']
        for name, value in {**WIDE_WORKSPACE, **params}.items():
            if isinstance(value, bool):
                value = str(value).lower()
            args += ['-p', f'{name}:={value}']
        rclpy.init(args=args)
        from ros2_vive_controller.vive_node import ViveDriverNode
        node = ViveDriverNode()
        node.pub_pose = Recorder()
        node.pub_joy = Recorder()
        node.log = LogRecorder()
        node.get_logger = lambda: node.log
        created.append(node)
        return node

    yield make
    for node in created:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _plug_controllers(vr_system):
    left = vr_system.plug(1, FakeDevice(CONTROLLER, 'LHR-LEFT', position=(0.1, 1.0, 0.2)), announce=False)
    right = vr_system.plug(2, FakeDevice(CONTROLLER, 'LHR-RIGHT', position=(-0.1, 1.0, 0.2)), announce=False)
    left.trigger = 0.2
    right.trigger = 0.9
    return left, right


def _published_triggers(node):
    return [msg.position[0] for msg in node.pub_joy.messages]


def test_other_controller_power_cycle_does_not_change_what_is_published(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT')
    node.update()

    vr_system.unplug(1)          # the LEFT controller is switched off...
    node.update()
    vr_system.replug(1)          # ...and on again
    node.update()
    node.update()

    assert _published_triggers(node) == [0.9] * 4


def test_own_controller_power_cycle_is_survived(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='left', serial='LHR-LEFT')
    node.update()
    assert _published_triggers(node) == [0.2]

    vr_system.unplug(1)
    node.update()                # used to raise KeyError and kill the node
    node.update()
    assert _published_triggers(node) == [0.2]
    assert 'disconnected' in node.log.text('warn')

    vr_system.replug(1)
    node.update()
    assert _published_triggers(node) == [0.2, 0.2]


def test_missing_serial_lists_the_connected_devices(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right')
    node.update()
    node.update()                # throttled: only one error line
    errors = node.log.text('error')
    assert errors.count('No serial configured for the right controller') == 1
    assert 'LHR-LEFT (Controller)' in errors and 'LHR-RIGHT (Controller)' in errors
    assert node.pub_joy.messages == []


def test_identify_vibrates_from_the_haptic_timer(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT')
    response = node.identify_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    assert node._haptic_until == pytest.approx(time.monotonic() + 3.0, abs=0.5)

    node._haptic_tick()
    assert vr_system.pulses_for(2) == [(2, 0, 3900)]
    assert vr_system.pulses_for(1) == []

    node._haptic_until = 0.0
    node._haptic_tick()
    assert len(vr_system.pulses_for(2)) == 1


def test_identify_reports_a_disconnected_controller(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-ABSENT')
    assert not node.identify_callback(Trigger.Request(), Trigger.Response()).success


def _plug_lighthouse(vr_system, pose_valid):
    vr_system.plug(3, FakeDevice(LIGHTHOUSE, 'LHB-ORIGIN', position=(0.0, 2.0, 0.0), pose_valid=pose_valid),
                   announce=False)


def test_valid_reference_lighthouse_becomes_the_frame(vr_system, driver):
    _plug_controllers(vr_system)
    _plug_lighthouse(vr_system, pose_valid=True)
    node = driver(side='right', serial='LHR-RIGHT', reference_lighthouse_serial='LHB-ORIGIN')
    node.update()
    assert node.pub_pose.messages[-1].header.frame_id == 'lighthouse_origin'


def test_invalid_reference_lighthouse_pose_keeps_the_world_frame_and_says_so(vr_system, driver):
    _plug_controllers(vr_system)
    _plug_lighthouse(vr_system, pose_valid=False)
    node = driver(side='right', serial='LHR-RIGHT', reference_lighthouse_serial='LHB-ORIGIN')
    node.update()
    node.update()
    assert [m.header.frame_id for m in node.pub_pose.messages] == ['vive_world', 'vive_world']
    assert node.log.text('warn').count('reports no valid pose') == 1


def test_invalid_reference_lighthouse_pose_is_used_when_allowed(vr_system, driver):
    _plug_controllers(vr_system)
    _plug_lighthouse(vr_system, pose_valid=False)
    node = driver(side='right', serial='LHR-RIGHT', reference_lighthouse_serial='LHB-ORIGIN',
                  allow_invalid_lighthouse_pose=True)
    node.update()
    assert node.pub_pose.messages[-1].header.frame_id == 'lighthouse_origin'


def test_tracker_node_survives_a_power_cycle(vr_system):
    from fake_openvr import TRACKER
    vr_system.plug(5, FakeDevice(TRACKER, 'LHR-TRK'), announce=False)
    rclpy.init(args=['--ros-args', '-p', 'serial:=LHR-TRK', '-p', 'name:=wrist'])
    try:
        from ros2_vive_controller.tracker_3_0_node import ViveTracker3_0Node
        node = ViveTracker3_0Node()
        node.pub_pose = Recorder()
        node.pub_pose_hand = Recorder()
        node.update()
        vr_system.unplug(5)
        node.watchdog()          # polls the deactivation event
        node.update()            # used to raise KeyError
        vr_system.replug(5)
        node.watchdog()
        node.update()
        assert len(node.pub_pose.messages) == 2
        node.destroy_node()
    finally:
        rclpy.shutdown()


def _fence_params(position, offsets, frame='vive_world', padding=0.1):
    """Fence params around the published position of a controller at raw `position`.

    offsets: (low, high) distance of the fence faces from the published x coordinate.
    """
    from fake_openvr import translation_matrix
    from ros2_vive_controller.openvr_class.openvr_class import convert_to_quaternion
    px, py, pz = convert_to_quaternion(translation_matrix(*position))[:3]
    return {
        'workspace.enabled': True, 'workspace.frame_id': frame, 'workspace.padding': padding,
        'workspace.x_min': px - offsets[0], 'workspace.x_max': px + offsets[1],
        'workspace.y_min': py - 1.0, 'workspace.y_max': py + 1.0,
        'workspace.z_min': pz - 1.0, 'workspace.z_max': pz + 1.0,
    }


RIGHT_POSITION = (-0.1, 1.0, 0.2)


def _fence_pulses(vr_system):
    return [p for p in vr_system.pulses_for(2) if p[2] == 2000]


def test_fence_inside_publishes_without_vibrating(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT', **_fence_params(RIGHT_POSITION, (0.5, 0.5)))
    node.update()
    assert len(node.pub_pose.messages) == 1
    assert _fence_pulses(vr_system) == []


def test_fence_warning_band_vibrates_but_still_publishes(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT', **_fence_params(RIGHT_POSITION, (0.05, 0.5)))
    node.update()
    assert len(node.pub_pose.messages) == 1
    assert len(_fence_pulses(vr_system)) == 1


def test_fence_outside_holds_the_pose_but_not_the_buttons(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT', **_fence_params(RIGHT_POSITION, (-0.2, 0.5)))
    node.update()
    assert node.pub_pose.messages == []
    assert len(node.pub_joy.messages) == 1          # the clutch release must still get through
    assert len(_fence_pulses(vr_system)) == 1


def test_fence_calibrated_in_another_frame_is_ignored(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT',
                  **_fence_params(RIGHT_POSITION, (-0.2, 0.5), frame='lighthouse_origin'))
    node.update()
    node.update()
    assert len(node.pub_pose.messages) == 2
    assert node.log.text('warn').count("calibrated in 'lighthouse_origin'") == 1


def test_fence_is_off_while_calibrating(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT', **_fence_params(RIGHT_POSITION, (-0.2, 0.5)))
    node.is_calibrating = True
    node.update()
    assert len(node.pub_pose.messages) == 1


def test_controller_back_without_activation_event_is_picked_up(vr_system, driver):
    _plug_controllers(vr_system)
    node = driver(side='right', serial='LHR-RIGHT')
    node.update()
    vr_system.unplug(2)
    node.update()
    vr_system.slots[2].connected = True          # USB device re-enumerated, SteamVR sent nothing
    node._last_resync = 0.0                      # resync runs at most once a second
    node.update()
    node.update()
    assert _published_triggers(node) == [0.9, 0.9, 0.9]


def test_lighthouse_frame_is_kept_when_its_pose_flickers(vr_system, driver):
    """Switching back to vive_world for a moment would make every published pose jump."""
    _plug_controllers(vr_system)
    _plug_lighthouse(vr_system, pose_valid=True)
    node = driver(side='right', serial='LHR-RIGHT', reference_lighthouse_serial='LHB-ORIGIN')
    node.update()
    first = node.pub_pose.messages[-1].pose.position
    vr_system.slots[3].pose_valid = False
    node.update()
    node.update()
    vr_system.slots[3].pose_valid = True
    node.update()
    frames = [m.header.frame_id for m in node.pub_pose.messages]
    assert frames == ['lighthouse_origin'] * 4
    last = node.pub_pose.messages[-1].pose.position
    assert (last.x, last.y, last.z) == pytest.approx((first.x, first.y, first.z))
    assert node.log.text('warn').count('keeping its last known pose') == 1
