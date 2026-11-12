#!/usr/bin/env python3
import time
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from OneEuroFilter import OneEuroFilter
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr, tracking_ok, tracking_result_name
from ros2_vive_controller.workspace import OUTSIDE, WARNING, classify

class ViveDriverNode(Node):
    def __init__(self):
        super().__init__('vive_driver')

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('side', 'right'),
                ('serial', ''),
                ('reference_lighthouse_serial', ''),
                ('allow_invalid_lighthouse_pose', False),
                # Only use poses SteamVR tracks optically (Running_OK). A partly covered controller is
                # often still flagged valid while SteamVR dead-reckons from the IMU, and those poses jump.
                ('require_tracking_ok', True),
                # Consecutive good samples needed before publishing again after tracking was lost.
                ('tracking_ok_samples', 5),
                ('frame_id', 'vive_world'),
                ('linear_scale', 1.0),
                # Fence written by `make calibrate` (config/workspace.yaml). Off until calibrated.
                ('workspace.enabled', False),
                # Frame the fence was calibrated in; the fence turns off if poses are in another one.
                ('workspace.frame_id', ''),
                ('workspace.x_min', -1.0), ('workspace.x_max', 1.0),
                ('workspace.y_min', -1.0), ('workspace.y_max', 1.0),
                ('workspace.z_min', 0.0),  ('workspace.z_max', 2.0),
                ('workspace.padding', 0.1),
                ('filter.mincutoff', 1.0),
                ('filter.beta', 0.007),
                ('filter.dcutoff', 1.0)
            ]
        )

        self.side = self.get_parameter('side').value
        self.serial = self.get_parameter('serial').value
        self.ref_lh_serial = self.get_parameter('reference_lighthouse_serial').value
        self.frame_id = self.get_parameter('frame_id').value
        self.linear_scale = self.get_parameter('linear_scale').value

        self.ws = {
            'x_min': self.get_parameter('workspace.x_min').value,
            'x_max': self.get_parameter('workspace.x_max').value,
            'y_min': self.get_parameter('workspace.y_min').value,
            'y_max': self.get_parameter('workspace.y_max').value,
            'z_min': self.get_parameter('workspace.z_min').value,
            'z_max': self.get_parameter('workspace.z_max').value,
            'pad': self.get_parameter('workspace.padding').value,
            'enabled': self.get_parameter('workspace.enabled').value,
            'frame_id': self.get_parameter('workspace.frame_id').value,
        }

        # Lighthouse 2.0 base stations flag their pose as invalid although it is calibrated.
        # Using it anyway changes the frame the pose is published in, so it is opt-in.
        self.allow_invalid_lh_pose = self.get_parameter('allow_invalid_lighthouse_pose').value

        # --- VR Setup ---
        # Devices are looked up by serial on every cycle (see update()), so a controller that is
        # switched off and on again is picked up again instead of crashing the node.
        self.vr = triad_openvr()
        self.connected = False
        self._last_missing_log = None
        self._last_resync = 0.0
        self._lh_state = None
        self._lh_pose = None

        # --- Publishers & Services ---
        self.srv_identify = self.create_service(Trigger, 'identify', self.identify_callback)
        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.pub_joy = self.create_publisher(JointState, 'joint_states', 10)
        # Transient local: RViz gets the fence even when it starts after the driver.
        self.pub_marker = self.create_publisher(
            Marker, 'workspace_marker', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- State Variables ---
        self.is_calibrating = False
        self.last_heartbeat_time = 0.0
        self.create_subscription(Bool, '/vive/is_calibrating', self.calibration_callback, 10)
        self.create_timer(0.05, self.watchdog_check)

        self.filters = self._init_filters()
        self.tracking_active = False
        self._haptic_until = 0.0
        self.require_tracking_ok = self.get_parameter('require_tracking_ok').value
        self.tracking_ok_samples = max(1, self.get_parameter('tracking_ok_samples').value)
        self._ok_streak = 0
        self._lost_at = None

        self.create_timer(0.02, self.update)
        # Haptics get their own timer so that buzzing never blocks the pose loop.
        self.create_timer(0.01, self._haptic_tick)
        self._marker_frame = None
        self._fence_mismatch_frame = None
        if self.ws['enabled']:
            self.get_logger().info(
                f"Workspace fence enabled (frame '{self.ws['frame_id'] or 'any'}', warning band {self.ws['pad']} m).")

    def identify_callback(self, request, response):
        if self.vr.get_device_by_serial(self.serial) is None:
            response.success = False
            response.message = "Controller not connected."
            return response
        self.get_logger().info(f"Identifying controller: {self.serial}")
        self._buzz(3.0)
        response.success = True
        response.message = f"Vibrating {self.side} controller ({self.serial})"
        return response

    def _buzz(self, seconds):
        """Vibrate the controller for the given time (one 3.9 ms pulse every 10 ms)."""
        self._haptic_until = max(self._haptic_until, time.monotonic() + seconds)

    def _haptic_tick(self):
        if time.monotonic() >= self._haptic_until:
            return
        controller = self.vr.get_device_by_serial(self.serial)
        if controller is None:
            self._haptic_until = 0.0
            return
        controller.trigger_haptic_pulse(3900)

    def _available_serials(self):
        return [f"{dev.get_serial()} ({dev.device_class})" for dev in self.vr.devices.values()
                if dev.device_class in ('Controller', 'Tracker')]

    def _report_missing_controller(self):
        now = time.monotonic()
        if self.connected:
            self.connected = False
            self.tracking_active = False
            self._haptic_until = 0.0
            self._last_missing_log = now
            self.get_logger().warn(
                f"{self.side} controller {self.serial} disconnected. Waiting for it to come back...")
            return
        if self._last_missing_log is not None and now - self._last_missing_log < 5.0:
            return
        self._last_missing_log = now
        if self.serial:
            problem = f"Could not find {self.side} controller {self.serial}."
        else:
            problem = f"No serial configured for the {self.side} controller (parameter 'serial' is empty)."
        available = ', '.join(self._available_serials()) or 'none'
        self.get_logger().error(f"{problem} Connected controllers/trackers: {available}")

    def _log_lighthouse_state(self, state, message, warn):
        if state == self._lh_state:
            return
        self._lh_state = state
        if warn:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _reference_lighthouse_pose(self):
        """Pose of the reference lighthouse, or None to publish in the OpenVR world frame."""
        if not self.ref_lh_serial:
            return None
        lh = self.vr.get_device_by_serial(self.ref_lh_serial)
        lh_pose = None
        if lh is not None:
            lh_pose = lh.get_pose_quaternion()
            if lh_pose is None and self.allow_invalid_lh_pose:
                lh_pose = lh.get_pose_quaternion(require_valid=False)
        if lh_pose is not None:
            if self._lh_pose is None:
                self._log_lighthouse_state('ok', f"Using Lighthouse {self.ref_lh_serial} as origin frame.", False)
            self._lh_pose = lh_pose
            return lh_pose
        if self._lh_pose is not None:
            # Base stations do not move: keep the last known pose instead of switching the published
            # frame back to vive_world, which would make every pose jump.
            self._log_lighthouse_state(
                'held', f"Lighthouse {self.ref_lh_serial} pose unavailable for now: keeping its last known pose.",
                True)
            return self._lh_pose
        if lh is None:
            self._log_lighthouse_state(
                'missing', f"Lighthouse {self.ref_lh_serial} not found. Using default OpenVR world frame.", True)
        else:
            self._log_lighthouse_state(
                'invalid',
                f"Lighthouse {self.ref_lh_serial} reports no valid pose (Base Station 2.0?). Staying in "
                f"{self.frame_id}. Set allow_invalid_lighthouse_pose:=true to use its stored calibration.",
                True)
        return None

    def calibration_callback(self, msg):
        if msg.data:
            self.is_calibrating = True
            self.last_heartbeat_time = self.get_clock().now().nanoseconds / 1e9

    def watchdog_check(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.is_calibrating and (now - self.last_heartbeat_time > 0.5):
            self.get_logger().warn("Calibration heartbeat lost! Re-enabling safety limits.")
            self.is_calibrating = False

    def _init_filters(self):
        params = {
            'freq': 50.0,
            'mincutoff': self.get_parameter('filter.mincutoff').value,
            'beta': self.get_parameter('filter.beta').value,
            'dcutoff': self.get_parameter('filter.dcutoff').value
        }
        return {axis: OneEuroFilter(**params) for axis in ['x', 'y', 'z']}

    def _fence_state(self, position, frame):
        """None when the fence is off, else INSIDE, WARNING or OUTSIDE (see workspace.py)."""
        if not self.ws['enabled'] or self.is_calibrating:
            return None
        if self.ws['frame_id'] and self.ws['frame_id'] != frame:
            if self._fence_mismatch_frame != frame:
                self._fence_mismatch_frame = frame
                self.get_logger().warn(
                    f"Workspace fence was calibrated in '{self.ws['frame_id']}' but poses are in '{frame}': "
                    f"fence disabled. Run 'make calibrate' again with the same reference lighthouse.")
            return None
        if frame != self._marker_frame:
            self.publish_workspace_marker(frame)
            self._marker_frame = frame
        return classify(position, self.ws, self.ws['pad'])

    def _compute_relative_pose(self, target_pose, reference_pose):
        """Transforms target_pose into the coordinate frame of reference_pose."""
        T_ref = np.eye(4)
        T_ref[:3, 3] = reference_pose[:3]
        T_ref[:3, :3] = R.from_quat(reference_pose[3:]).as_matrix()

        T_target = np.eye(4)
        T_target[:3, 3] = target_pose[:3]
        T_target[:3, :3] = R.from_quat(target_pose[3:]).as_matrix()

        T_rel = np.linalg.inv(T_ref) @ T_target

        pos = T_rel[:3, 3]
        quat = R.from_matrix(T_rel[:3, :3]).as_quat()

        return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]

    def _make_tf(self, pose, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]
        return t

    def _apply_rotation_offset(self, pose, axis='z', angle_degrees=180):
        """Rotates a pose around a local axis to correct hardware alignments."""
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_quat(pose[3:]).as_matrix()

        T_rot = np.eye(4)
        T_rot[:3, :3] = R.from_euler(axis, angle_degrees, degrees=True).as_matrix()

        T_new = T @ T_rot

        pos = T_new[:3, 3]
        quat = R.from_matrix(T_new[:3, :3]).as_quat()
        return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]

    def update(self):
        # 1. Poll OpenVR to catch devices that appear or disappear
        self.vr.poll_vr_events()

        # 2. Check controller (looked up by serial every cycle, never by a cached name)
        controller = self.vr.get_device_by_serial(self.serial)
        if controller is None and time.monotonic() - self._last_resync >= 1.0:
            # Don't rely only on activation events: SteamVR does not always send one when a device
            # comes back (seen in the lab with a USB controller that was switched off and on).
            self._last_resync = time.monotonic()
            if self.vr.sync_connected_devices():
                controller = self.vr.get_device_by_serial(self.serial)
        if controller is None:
            self._report_missing_controller()
            return
        if not self.connected:
            self.connected = True
            self.get_logger().info(f"Connected to {self.side} controller: {self.serial}")

        poses = self.vr.get_pose()
        pose = controller.get_pose_quaternion(poses)
        tracking_result = controller.get_tracking_result(poses)
        inputs = controller.get_controller_inputs()

        # Tracking Check. Degraded tracking (IMU only) counts as lost: those poses drift and jump.
        if pose is not None and self.require_tracking_ok and not tracking_ok(tracking_result):
            pose = None
        self._ok_streak = self._ok_streak + 1 if pose is not None else 0
        if pose is None or (not self.tracking_active and self._ok_streak < self.tracking_ok_samples):
            if pose is None and self.tracking_active:
                self.get_logger().warn(f"Tracking LOST for {self.side} ({tracking_result_name(tracking_result)})")
                self.tracking_active = False
                self._lost_at = time.monotonic()
            return

        if not self.tracking_active:
            self.tracking_active = True
            # Buzz on first acquisition and after real losses, not on every short flicker.
            if self._lost_at is None or time.monotonic() - self._lost_at > 1.0:
                self.get_logger().info(f"Tracking ACQUIRED for {self.side}! Buzzing...")
                self._buzz(1.0)
            else:
                self.get_logger().info(f"Tracking back for {self.side}.")

        # --- Frame Resolution & TF Broadcasting ---
        world_frame = self.frame_id
        lh_frame = "lighthouse_origin"
        controller_frame = f"vive_{self.side}_link"

        final_pose = pose
        current_ref_frame = world_frame

        lh_pose = self._reference_lighthouse_pose()
        if lh_pose:
            # --- NEW: Flip the lighthouse frame 180 degrees on the Z (Yaw) axis ---
            # Note: If it's flipping upside down instead of spinning around,
            # change axis='z' to axis='x' or axis='y'
            lh_pose = self._apply_rotation_offset(lh_pose, axis='z', angle_degrees=180)

            current_ref_frame = lh_frame
            # Broadcast World -> Lighthouse
            self.tf_broadcaster.sendTransform(self._make_tf(lh_pose, world_frame, lh_frame))
            # Compute Relative Pose
            final_pose = self._compute_relative_pose(pose, lh_pose)
            # Broadcast Lighthouse -> Controller
            self.tf_broadcaster.sendTransform(self._make_tf(final_pose, lh_frame, controller_frame))

        # Fallback if no lighthouse tracked
        if current_ref_frame == world_frame:
            self.tf_broadcaster.sendTransform(self._make_tf(final_pose, world_frame, controller_frame))

        # --- Workspace fence ---
        # Warning band: vibrate but keep publishing. Outside: vibrate and hold the pose back (the
        # teleop bridge then disengages). Buttons are always published so a clutch release is never missed.
        fence = self._fence_state(final_pose[0:3], current_ref_frame)
        if fence in (WARNING, OUTSIDE):
            controller.trigger_haptic_pulse(2000)

        # --- Publish Data ---
        timestamp = self.get_clock().now().to_msg()
        t = self.get_clock().now().nanoseconds / 1e9

        if fence != OUTSIDE:
            msg_pose = PoseStamped()
            msg_pose.header.stamp = timestamp
            msg_pose.header.frame_id = current_ref_frame

            msg_pose.pose.position.x = self.filters['x'](final_pose[0], t) * self.linear_scale
            msg_pose.pose.position.y = self.filters['y'](final_pose[1], t) * self.linear_scale
            msg_pose.pose.position.z = self.filters['z'](final_pose[2], t) * self.linear_scale

            msg_pose.pose.orientation.x = final_pose[3]
            msg_pose.pose.orientation.y = final_pose[4]
            msg_pose.pose.orientation.z = final_pose[5]
            msg_pose.pose.orientation.w = final_pose[6]
            self.pub_pose.publish(msg_pose)

        msg_joy = JointState()
        msg_joy.header.stamp = timestamp
        msg_joy.header.frame_id = current_ref_frame
        msg_joy.name = ['trigger', 'trackpad_x', 'trackpad_y', 'grip', 'menu', 'trackpad_touched', 'trackpad_pressed']
        msg_joy.position = [
            float(inputs.get('trigger', 0.0)),
            float(inputs.get('trackpad_x', 0.0)),
            float(inputs.get('trackpad_y', 0.0)),
            float(inputs.get('grip_button', False)),
            float(inputs.get('menu_button', False)),
            float(inputs.get('trackpad_touched', False)),
            float(inputs.get('trackpad_pressed', False))
        ]
        self.pub_joy.publish(msg_joy)

    def publish_workspace_marker(self, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workspace_limit"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = self.ws['x_max'] - self.ws['x_min']
        m.scale.y = self.ws['y_max'] - self.ws['y_min']
        m.scale.z = self.ws['z_max'] - self.ws['z_min']
        m.pose.position.x = (self.ws['x_max'] + self.ws['x_min']) / 2.0
        m.pose.position.y = (self.ws['y_max'] + self.ws['y_min']) / 2.0
        m.pose.position.z = (self.ws['z_max'] + self.ws['z_min']) / 2.0
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.15
        self.pub_marker.publish(m)

def main():
    rclpy.init()
    node = ViveDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    # On Ctrl+C rclpy's signal handler has already shut the context down.
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()