#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from ros2_vive_controller.teleop_core import Clutch, compute_target, gate_button, is_stale, parse_bool


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge_node')

        # --- 1. Parameters ---
        self.declare_parameter('pose_topic', '/vive/left')
        self.declare_parameter('button_state_topic', '/vive/button_state')
        self.declare_parameter('output_topic', '/vive/output_pose')
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('rotation_offset', [0.0, 0.0, 0.0]) # [R, P, Y] in degrees
        # Bool, or the legacy 'true'/'false' strings.
        self.declare_parameter('trackpad_pressed_required', False, ParameterDescriptor(dynamic_typing=True))
        # Clutch on the trigger. The defaults are the original behaviour: engage at a full pull, release
        # as soon as the trigger leaves it. Optional hysteresis, e.g. 0.95 / 0.90, for worn triggers;
        # note it then takes a bigger release to ratchet.
        self.declare_parameter('clutch_engage_threshold', 1.0)
        self.declare_parameter('clutch_release_threshold', 1.0)
        # Disengage when no pose arrives for this long while engaged (seconds, 0 disables).
        self.declare_parameter('pose_timeout', 0.3)
        # Where a new clutch stroke starts from:
        #   'tf'           (default, original) the end effector pose read from TF
        #   'last_command' the last pose this bridge sent, which the arm is being held at. TF is then
        #                  only used for the first stroke, so a delayed TF cannot make the arm jump
        #                  back to an old position when the clutch is pressed again.
        self.declare_parameter('clutch_reference', 'tf')

        self.button_keys = [
            'trigger', 'trackpad_x', 'trackpad_y',
            'grip', 'menu', 'trackpad_touched', 'trackpad_pressed'
        ]

        for key in self.button_keys:
            self.declare_parameter(f'{key}_topic', '')

        # --- 2. Publishers Dictionary ---
        self.button_pubs = {}
        for key in self.button_keys:
            topic = self.get_parameter(f'{key}_topic').value
            if topic:
                # All button/axis topics use PointStamped (value in point.x)
                self.button_pubs[key] = self.create_publisher(PointStamped, topic, 10)

        # Standard Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('output_topic').value, 10)

        # --- 3. TF2 and State ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        engage = self.get_parameter('clutch_engage_threshold').value
        release = self.get_parameter('clutch_release_threshold').value
        if release > engage:
            self.get_logger().warn(
                f'clutch_release_threshold ({release}) is above clutch_engage_threshold ({engage}); using {engage}.')
            release = engage
        self.clutch = Clutch(engage, release)
        self.pose_timeout = self.get_parameter('pose_timeout').value
        self.clutch_reference = self.get_parameter('clutch_reference').value
        if self.clutch_reference not in ('tf', 'last_command'):
            self.get_logger().warn(f"Unknown clutch_reference '{self.clutch_reference}', using 'tf'.")
            self.clutch_reference = 'tf'
        self.trackpad_pressed_required = parse_bool(self.get_parameter('trackpad_pressed_required').value)

        self.latest_pose = None
        self.latest_pose_time = None
        self.latest_button_state = None
        self.ee_translation = None
        self.joy_translation = None
        self.activated = False
        self.last_output_msg = None

        # --- 4. Subscribers ---
        self.pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self.pose_callback, 10)

        self.button_state_sub = self.create_subscription(
            JointState, self.get_parameter('button_state_topic').value, self.button_state_callback, 10)

        # --- 5. Timers ---
        freq = self.get_parameter('publish_frequency').value
        if freq > 0:
            self.create_timer(1.0 / freq, self.timer_callback)
        # Runs even when publish_frequency <= 0: a pose stream that stops must still be noticed.
        self.create_timer(0.05, self.watchdog_callback)

    def button_state_callback(self, msg):
        """Store button state, publish individual PointStamped topics, and run the clutch"""
        self.latest_button_state = msg
        button_data = {name: pos for name, pos in zip(msg.name, msg.position)}
        trackpad_pressed = button_data.get('trackpad_pressed', 0.0)

        # --- Individual Topic Publishing ---
        timestamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('reference_frame').value
        for key, pub in self.button_pubs.items():
            if key in button_data:
                p_msg = PointStamped()
                p_msg.header.stamp = timestamp
                p_msg.header.frame_id = frame_id
                p_msg.point.x = gate_button(key, button_data[key], trackpad_pressed, self.trackpad_pressed_required)
                p_msg.point.y = 0.0
                p_msg.point.z = 0.0
                pub.publish(p_msg)

        # --- Activation (Clutch) Logic ---
        if 'trigger' in button_data:
            if self.clutch.update(button_data['trigger']) == 'release':
                self.deactivate('trigger released')
            if self.clutch.engaged and not self.activated:
                # Retried on every message while the trigger is held, until it succeeds.
                self.activate()

    def activate(self):
        """Save reference transforms when the clutch is engaged"""
        target = self.get_parameter('target_frame').value
        ref = self.get_parameter('reference_frame').value

        if self.latest_pose is None or self._pose_is_stale():
            self._activation_failed('no recent pose from the Vive driver')
            return

        if self.clutch_reference == 'last_command' and self.last_output_msg is not None:
            # Continue from the pose the arm is being held at: no jump, whatever TF says.
            self.ee_translation = Point(x=self.last_output_msg.pose.position.x,
                                        y=self.last_output_msg.pose.position.y,
                                        z=self.last_output_msg.pose.position.z)
            self._save_joy_reference()
            return

        # Non-blocking check: never wait for TF inside a callback.
        if not self.tf_buffer.can_transform(ref, target, rclpy.time.Time()):
            self._activation_failed(f'no transform from {ref} to {target} yet')
            return
        try:
            transform = self.tf_buffer.lookup_transform(ref, target, rclpy.time.Time())
        except Exception as e:
            self._activation_failed(str(e))
            return

        self.ee_translation = transform.transform.translation
        self._save_joy_reference()

    def _save_joy_reference(self):
        p = self.latest_pose.pose.position
        self.joy_translation = Point(x=p.x, y=p.y, z=p.z)
        self.activated = True
        self.get_logger().info('Teleop Activated: Reference frames saved.')

    def _activation_failed(self, reason):
        self.get_logger().warn(f'Activation failed: {reason} (keep the trigger pressed to retry)',
                               throttle_duration_sec=1.0)

    def deactivate(self, reason):
        if self.activated:
            self.get_logger().info(f'Teleop Deactivated: {reason}.')
        self.activated = False

    def _pose_is_stale(self):
        now = self.get_clock().now().nanoseconds / 1e9
        last = None if self.latest_pose_time is None else self.latest_pose_time.nanoseconds / 1e9
        return is_stale(last, now, self.pose_timeout)

    def watchdog_callback(self):
        if self.activated and self._pose_is_stale():
            # Without this, poses resuming at a different place would make the arm jump.
            self.clutch.force_release()
            self.deactivate(f'no pose for more than {self.pose_timeout:.2f} s, release the trigger and press it again')

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()
        if self.get_parameter('publish_frequency').value <= 0:
            self.publish_pose()

    def timer_callback(self):
        self.publish_pose()

    def publish_pose(self):
        """Compute delta from reference and publish updated pose"""
        output_msg = None

        if self.latest_pose and self.activated:
            p = self.latest_pose.pose.position
            q = self.latest_pose.pose.orientation
            position, orientation = compute_target(
                ee_position=(self.ee_translation.x, self.ee_translation.y, self.ee_translation.z),
                joy_reference=(self.joy_translation.x, self.joy_translation.y, self.joy_translation.z),
                joy_position=(p.x, p.y, p.z),
                joy_orientation=(q.x, q.y, q.z, q.w),
                rotation_offset_deg=self.get_parameter('rotation_offset').value)

            output_msg = PoseStamped()
            output_msg.header.frame_id = self.get_parameter('reference_frame').value
            output_msg.header.stamp = self.get_clock().now().to_msg()
            (output_msg.pose.position.x,
             output_msg.pose.position.y,
             output_msg.pose.position.z) = (float(v) for v in position)
            (output_msg.pose.orientation.x,
             output_msg.pose.orientation.y,
             output_msg.pose.orientation.z,
             output_msg.pose.orientation.w) = (float(v) for v in orientation)

            self.last_output_msg = output_msg
        else:
            output_msg = self.last_output_msg

        if output_msg:
            self.pose_pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # On Ctrl+C rclpy's signal handler has already shut the context down.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
