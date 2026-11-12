"""teleop_bridge_node in-process: clutch, stale-pose watchdog, TF waits and button gating.

Callbacks are called directly and TF is injected into the node's buffer, so no executor,
robot or Vive driver is needed.
"""
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

BUTTONS = ['trigger', 'trackpad_x', 'trackpad_y', 'grip', 'menu', 'trackpad_touched', 'trackpad_pressed']


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def bridge():
    created = []

    def make(with_tf=True, **params):
        base = {
            'pose_topic': '/test/pose', 'button_state_topic': '/test/buttons', 'output_topic': '/test/out',
            'publish_frequency': -1.0, 'reference_frame': 'base', 'target_frame': 'ee',
            'trigger_topic': '/test/trigger', 'trackpad_x_topic': '/test/trackpad_x', 'menu_topic': '/test/menu',
        }
        args = ['--ros-args']
        for name, value in {**base, **params}.items():
            args += ['-p', f'{name}:={value}']
        rclpy.init(args=args)
        from ros2_vive_controller.teleop_bridge_node import TeleopBridgeNode
        node = TeleopBridgeNode()
        node.pose_pub = Recorder()
        node.button_pubs = {key: Recorder() for key in node.button_pubs}
        if with_tf:
            add_ee_transform(node)
        created.append(node)
        return node

    yield make
    for node in created:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def add_ee_transform(node, x=0.4, y=0.0, z=0.9):
    t = TransformStamped()
    t.header.frame_id = 'base'
    t.child_frame_id = 'ee'
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    t.transform.rotation.w = 1.0
    node.tf_buffer.set_transform_static(t, 'test')


def send_pose(node, x, y=0.0, z=1.0):
    msg = PoseStamped()
    msg.header.frame_id = 'vive_world'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    node.pose_callback(msg)
    return msg


def send_buttons(node, trigger=0.0, trackpad_x=0.0, trackpad_pressed=0.0, menu=0.0):
    values = {'trigger': trigger, 'trackpad_x': trackpad_x, 'trackpad_pressed': trackpad_pressed, 'menu': menu}
    msg = JointState()
    msg.name = BUTTONS
    msg.position = [float(values.get(name, 0.0)) for name in BUTTONS]
    node.button_state_callback(msg)


def last_output_x(node):
    return node.pose_pub.messages[-1].pose.position.x


def test_clutch_moves_the_target_by_the_hand_displacement(bridge):
    node = bridge()
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    assert node.activated
    send_pose(node, 2.1)
    assert last_output_x(node) == pytest.approx(0.5)


def test_default_clutch_releases_as_soon_as_the_trigger_leaves_full_pull(bridge):
    """The original behaviour: a slight relax is enough to ratchet."""
    node = bridge()
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    send_pose(node, 2.2)
    send_buttons(node, trigger=0.97)
    assert not node.activated
    send_pose(node, 2.0)               # hand goes back while released: the target stays
    assert last_output_x(node) == pytest.approx(0.6)
    add_ee_transform(node, x=0.6)      # the arm reached the command
    send_buttons(node, trigger=1.0)
    send_pose(node, 2.2)               # second stroke adds up: ratcheting
    assert last_output_x(node) == pytest.approx(0.8)


def test_each_clutch_restarts_from_the_end_effector_tf(bridge):
    """Original design, unchanged: the reference is where TF says the end effector is, not the last
    command. If the arm (or its TF) lags behind the command, each stroke restarts from there, so
    ratcheting does not add up."""
    node = bridge()
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    send_pose(node, 2.2)
    send_buttons(node, trigger=0.0)
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)    # TF still says x=0.4: the arm never got to 0.6
    send_pose(node, 2.2)
    assert last_output_x(node) == pytest.approx(0.6)


def test_optional_hysteresis_keeps_the_clutch_through_small_trigger_dips(bridge):
    node = bridge(clutch_engage_threshold=0.95, clutch_release_threshold=0.90)
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    send_buttons(node, trigger=0.93)
    assert node.activated
    send_pose(node, 2.2)
    send_buttons(node, trigger=0.5)
    assert not node.activated
    # After release the last command keeps being held, as before.
    held = len(node.pose_pub.messages)
    send_pose(node, 3.0)
    assert len(node.pose_pub.messages) == held + 1
    assert node.pose_pub.messages[-1] is node.pose_pub.messages[-2]


def test_stale_pose_disengages_until_the_trigger_is_pressed_again(bridge):
    node = bridge()
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    assert node.activated

    node.latest_pose_time = node.get_clock().now() - Duration(seconds=1.0)
    node.watchdog_callback()
    assert not node.activated

    send_pose(node, 2.5)                # poses resume somewhere else...
    send_buttons(node, trigger=1.0)     # ...with the trigger still held
    assert not node.activated

    send_buttons(node, trigger=0.0)
    send_buttons(node, trigger=1.0)
    assert node.activated
    send_pose(node, 2.6)
    assert last_output_x(node) == pytest.approx(0.5)


def test_pose_timeout_zero_disables_the_watchdog(bridge):
    node = bridge(pose_timeout=0.0)
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    node.latest_pose_time = node.get_clock().now() - Duration(seconds=10.0)
    node.watchdog_callback()
    assert node.activated


def test_activation_waits_for_tf_without_blocking_and_retries(bridge):
    node = bridge(with_tf=False)
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    assert not node.activated
    add_ee_transform(node)
    send_buttons(node, trigger=1.0)     # still held: retried
    assert node.activated


def test_activation_needs_a_pose(bridge):
    node = bridge()
    send_buttons(node, trigger=1.0)
    assert not node.activated
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    assert node.activated


def test_legacy_string_parameter_gates_only_the_trackpad(bridge):
    node = bridge(trackpad_pressed_required="'true'")   # a str, as the G1 launch file passes it
    assert node.get_parameter('trackpad_pressed_required').value == 'true'
    send_buttons(node, trigger=0.4, trackpad_x=0.8, menu=1.0)
    assert node.button_pubs['trackpad_x'].messages[-1].point.x == 0.0
    assert node.button_pubs['trigger'].messages[-1].point.x == pytest.approx(0.4)
    assert node.button_pubs['menu'].messages[-1].point.x == 1.0
    send_buttons(node, trackpad_x=0.8, trackpad_pressed=1.0)
    assert node.button_pubs['trackpad_x'].messages[-1].point.x == pytest.approx(0.8)


def test_output_header_does_not_alias_the_driver_message(bridge):
    node = bridge()
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    incoming = send_pose(node, 2.1)
    assert node.pose_pub.messages[-1].header.frame_id == 'base'
    assert incoming.header.frame_id == 'vive_world'


def test_last_command_reference_does_not_jump_back_when_tf_lags(bridge):
    node = bridge(clutch_reference='last_command')
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)    # first stroke starts from TF (x=0.4)
    send_pose(node, 2.2)
    assert last_output_x(node) == pytest.approx(0.6)
    send_buttons(node, trigger=0.0)
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)    # TF still says 0.4 (delayed)...
    send_pose(node, 2.0)
    assert last_output_x(node) == pytest.approx(0.6)   # ...but no jump back
    send_pose(node, 2.2)
    assert last_output_x(node) == pytest.approx(0.8)   # strokes add up


def test_last_command_reference_uses_tf_for_the_first_stroke(bridge):
    node = bridge(with_tf=False, clutch_reference='last_command')
    send_pose(node, 2.0)
    send_buttons(node, trigger=1.0)
    assert not node.activated
