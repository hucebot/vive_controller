#!/usr/bin/env python3
"""Bimanual Vive Tracker Node.

Manages two trackers with a single OpenVR instance.
Does NOT publish until both trackers AND the reference lighthouse are detected.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from OneEuroFilter import OneEuroFilter
from ros2_vive_controller.openvr_class.openvr_class import (
    triad_openvr,
    convert_to_quaternion,
    get_pose,
)


def pose_to_matrix(pose):
    """Convert [x, y, z, qx, qy, qz, qw] to 4x4 homogeneous matrix."""
    T = np.eye(4)
    T[:3, :3] = R.from_quat(pose[3:7]).as_matrix()
    T[:3, 3] = pose[0:3]
    return T


def matrix_to_pose(T):
    """Convert 4x4 homogeneous matrix to [x, y, z, qx, qy, qz, qw]."""
    pos = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()  # [qx, qy, qz, qw]
    return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]


class BimanualViveTrackerNode(Node):
    def __init__(self):
        super().__init__("bimanual_vive_tracker")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("serial_left", ""),
                ("serial_right", ""),
                ("reference_lighthouse_serial", ""),
                ("frame_id", "vive_world"),
                ("filter.mincutoff", 1.0),
                ("filter.beta", 0.007),
                ("filter.dcutoff", 1.0),
                ("hand_offset_left",  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
                ("hand_offset_right", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            ],
        )

        self.serial_left = self.get_parameter("serial_left").value
        self.serial_right = self.get_parameter("serial_right").value
        self.reference_lighthouse_serial = self.get_parameter(
            "reference_lighthouse_serial"
        ).value
        self.frame_id = self.get_parameter("frame_id").value

        # Single shared OpenVR instance
        self.vr = triad_openvr()
        self.vr.print_discovered_objects()

        # Find tracker devices
        self.device_left = self._find_device_by_serial(self.serial_left)
        self.device_right = self._find_device_by_serial(self.serial_right)

        for name, serial, dev in [
            ("left", self.serial_left, self.device_left),
            ("right", self.serial_right, self.device_right),
        ]:
            if dev:
                self.get_logger().info(
                    f"Connected to tracker '{name}': {serial} (device: {dev})"
                )
            else:
                self.get_logger().warn(
                    f"Tracker '{name}' with serial '{serial}' not found yet"
                )

        # Lighthouse reference frame
        self.T_lighthouse_inv = None
        self._lighthouse_retry_count = 0
        if self.reference_lighthouse_serial:
            self._compute_lighthouse_inverse()

        # Hand offset transforms (T_tracker_to_hand): [x, y, z, qx, qy, qz, qw]
        # Launch args arrive as strings — parse if needed
        def _parse_offset(val):
            if isinstance(val, str):
                import ast
                val = ast.literal_eval(val)
            return pose_to_matrix(val)

        self.T_tracker_to_hand_left  = _parse_offset(self.get_parameter("hand_offset_left").value)
        self.T_tracker_to_hand_right = _parse_offset(self.get_parameter("hand_offset_right").value)

        # Publishers
        self.pub_left = self.create_publisher(PoseStamped, "/vive_left/pose", 10)
        self.pub_right = self.create_publisher(PoseStamped, "/vive_right/pose", 10)
        self.pub_left_hand = self.create_publisher(PoseStamped, "/vive_left/pose_hand", 10)
        self.pub_right_hand = self.create_publisher(PoseStamped, "/vive_right/pose_hand", 10)

        # Independent filters per hand
        self.filters_left = self._init_filters()
        self.filters_right = self._init_filters()

        # Per-tracker tracking state
        self.tracking_left = False
        self.tracking_right = False
        self.haptic_left = 0
        self.haptic_right = 0

        # Ready state
        self._ready_logged = False

        # Timers
        self.create_timer(0.02, self.update)  # 50Hz
        self.create_timer(0.05, self.watchdog)  # 20Hz

    def _find_device_by_serial(self, target_serial):
        for key, dev in self.vr.devices.items():
            if dev.get_serial() == target_serial:
                return key
        return None

    def _compute_lighthouse_inverse(self):
        self._lighthouse_retry_count += 1

        for key, dev in self.vr.devices.items():
            if dev.get_serial() == self.reference_lighthouse_serial:
                # Read raw mDeviceToAbsoluteTracking directly — bypasses bPoseIsValid
                # which is always False for Lighthouse 2.0, even though the
                # calibrated position is present in the matrix.
                poses = get_pose(self.vr.vr)
                raw_mat = poses[dev.index].mDeviceToAbsoluteTracking
                # Check rotation diagonal — an uninitialized matrix is all zeros,
                # while any valid rotation has non-zero diagonal elements.
                # Translation CAN be (0,0,0) when the lighthouse is at the origin.
                if raw_mat[0][0] == 0.0 and raw_mat[1][1] == 0.0 and raw_mat[2][2] == 0.0:
                    if self._lighthouse_retry_count % 100 == 1:
                        self.get_logger().warn(
                            f"Reference lighthouse '{self.reference_lighthouse_serial}' found but matrix uninitialized (retry {self._lighthouse_retry_count})"
                        )
                    return
                pose = convert_to_quaternion(raw_mat)
                self.T_lighthouse_inv = np.linalg.inv(pose_to_matrix(pose))
                if self.frame_id == "vive_world":
                    self.frame_id = self.reference_lighthouse_serial
                self.get_logger().info(
                    f"Using lighthouse {self.reference_lighthouse_serial} as reference frame (frame_id: {self.frame_id})"
                )
                return
        if self._lighthouse_retry_count % 100 == 1:
            self.get_logger().warn(
                f"Waiting for reference lighthouse '{self.reference_lighthouse_serial}'... "
                f"Available: {[(k, v.get_serial()) for k, v in self.vr.devices.items() if 'tracking_reference' in k]}"
            )

    def _init_filters(self):
        params = {
            "freq": 50.0,
            "mincutoff": self.get_parameter("filter.mincutoff").value,
            "beta": self.get_parameter("filter.beta").value,
            "dcutoff": self.get_parameter("filter.dcutoff").value,
        }
        return {axis: OneEuroFilter(**params) for axis in ["x", "y", "z"]}

    def _is_ready(self):
        """Both trackers found and lighthouse reference established."""
        if self.reference_lighthouse_serial and self.T_lighthouse_inv is None:
            return False
        if not self.device_left or not self.device_right:
            return False
        return True

    def watchdog(self):
        self.vr.poll_vr_events()
        if not self.device_left:
            self.device_left = self._find_device_by_serial(self.serial_left)
            if self.device_left:
                self.get_logger().info(
                    f"Connected to tracker 'left': {self.serial_left} (device: {self.device_left})"
                )
        if not self.device_right:
            self.device_right = self._find_device_by_serial(self.serial_right)
            if self.device_right:
                self.get_logger().info(
                    f"Connected to tracker 'right': {self.serial_right} (device: {self.device_right})"
                )
        if self.reference_lighthouse_serial and self.T_lighthouse_inv is None:
            self._compute_lighthouse_inverse()

    def _publish_tracker(self, _device_name, publisher, pub_hand, T_tracker_to_hand, filters, pose, _name):
        """Transform, filter, and publish a single tracker's pose."""
        # Transform to lighthouse reference frame
        if self.T_lighthouse_inv is not None:
            T_tracker = pose_to_matrix(pose)
            T_rel = self.T_lighthouse_inv @ T_tracker
            pose = matrix_to_pose(T_rel)

        t = self.get_clock().now().nanoseconds / 1e9
        stamp = self.get_clock().now().to_msg()

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = filters["x"](pose[0], t)
        msg.pose.position.y = filters["y"](pose[1], t)
        msg.pose.position.z = filters["z"](pose[2], t)
        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]
        publisher.publish(msg)

        # Hand pose: apply static offset T_tracker_to_hand
        T_world_to_tracker = pose_to_matrix(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
             msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        )
        hand_pose = matrix_to_pose(T_world_to_tracker @ T_tracker_to_hand)
        msg_hand = PoseStamped()
        msg_hand.header.stamp = stamp
        msg_hand.header.frame_id = self.frame_id
        msg_hand.pose.position.x = hand_pose[0]
        msg_hand.pose.position.y = hand_pose[1]
        msg_hand.pose.position.z = hand_pose[2]
        msg_hand.pose.orientation.x = hand_pose[3]
        msg_hand.pose.orientation.y = hand_pose[4]
        msg_hand.pose.orientation.z = hand_pose[5]
        msg_hand.pose.orientation.w = hand_pose[6]
        pub_hand.publish(msg_hand)

    def update(self):
        if not self._is_ready():
            return

        if not self._ready_logged:
            self.get_logger().info(
                "Both trackers and lighthouse detected — publishing started"
            )
            self._ready_logged = True

        try:
            tracker_left = self.vr.devices[self.device_left]
            tracker_right = self.vr.devices[self.device_right]
        except KeyError:
            self.get_logger().warn("Tracker device disappeared, re-scanning...")
            self.device_left = self._find_device_by_serial(self.serial_left)
            self.device_right = self._find_device_by_serial(self.serial_right)
            return

        pose_left = tracker_left.get_pose_quaternion()
        pose_right = tracker_right.get_pose_quaternion()

        # Left tracker
        if pose_left is None:
            if self.tracking_left:
                self.get_logger().warn("Tracking LOST for 'left'")
                self.tracking_left = False
        else:
            if not self.tracking_left:
                self.get_logger().info("Tracking ACQUIRED for 'left'")
                self.tracking_left = True
                self.haptic_left = 50
            if self.haptic_left > 0:
                tracker_left.trigger_haptic_pulse(3000)
                self.haptic_left -= 1
            self._publish_tracker(
                self.device_left, self.pub_left, self.pub_left_hand,
                self.T_tracker_to_hand_left, self.filters_left, pose_left, "left"
            )

        # Right tracker
        if pose_right is None:
            if self.tracking_right:
                self.get_logger().warn("Tracking LOST for 'right'")
                self.tracking_right = False
        else:
            if not self.tracking_right:
                self.get_logger().info("Tracking ACQUIRED for 'right'")
                self.tracking_right = True
                self.haptic_right = 50
            if self.haptic_right > 0:
                tracker_right.trigger_haptic_pulse(3000)
                self.haptic_right -= 1
            self._publish_tracker(
                self.device_right, self.pub_right, self.pub_right_hand,
                self.T_tracker_to_hand_right, self.filters_right, pose_right, "right"
            )


def main():
    rclpy.init()
    node = BimanualViveTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
