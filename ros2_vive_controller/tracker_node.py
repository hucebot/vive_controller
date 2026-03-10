#!/usr/bin/env python3
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


class ViveTrackerNode(Node):
    def __init__(self):
        super().__init__("vive_tracker")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("name", "tracker"),
                ("serial", ""),
                ("frame_id", "vive_world"),
                ("reference_lighthouse_serial", ""),
                ("filter.mincutoff", 1.0),
                ("filter.beta", 0.007),
                ("filter.dcutoff", 1.0),
            ],
        )

        self.tracker_name = self.get_parameter("name").value
        self.serial = self.get_parameter("serial").value
        self.frame_id = self.get_parameter("frame_id").value
        self.reference_lighthouse_serial = self.get_parameter(
            "reference_lighthouse_serial"
        ).value

        # OpenVR setup
        self.vr = triad_openvr()
        self.vr.print_discovered_objects()
        self.device_name = self._find_device_by_serial(self.serial)

        if not self.device_name:
            self.get_logger().error(
                f"Could not find tracker with serial '{self.serial}'. "
                f"Available devices: {[(k, v.get_serial()) for k, v in self.vr.devices.items()]}"
            )
        else:
            self.get_logger().info(
                f"Connected to tracker '{self.tracker_name}': {self.serial} (device: {self.device_name})"
            )

        # Lighthouse reference frame transform
        self.T_lighthouse_inv = None
        self._lighthouse_warned = False
        if self.reference_lighthouse_serial:
            self._compute_lighthouse_inverse()

        # Publisher
        self.pub_pose = self.create_publisher(
            PoseStamped, f"/vive_{self.tracker_name}/pose", 10
        )

        # Filters
        self.filters = self._init_filters()

        # Tracking state
        self.tracking_active = False
        self.haptic_frames_remaining = 0

        # Timers
        self.create_timer(0.02, self.update)  # 50Hz
        self.create_timer(0.05, self.watchdog)  # 20Hz

    def _find_device_by_serial(self, target_serial):
        for key, dev in self.vr.devices.items():
            if dev.get_serial() == target_serial:
                return key
        return None

    def _compute_lighthouse_inverse(self):
        """Find reference lighthouse and cache its inverse transform.

        Reads mDeviceToAbsoluteTracking directly (bypassing bPoseIsValid)
        because Lighthouse 2.0 base stations have bPoseIsValid=False
        even though the calibrated position is present in the matrix.
        """
        for key, dev in self.vr.devices.items():
            if dev.get_serial() == self.reference_lighthouse_serial:
                poses = get_pose(self.vr.vr)
                raw_mat = poses[dev.index].mDeviceToAbsoluteTracking
                # Check rotation diagonal — an uninitialized matrix is all zeros,
                # while any valid rotation has non-zero diagonal elements.
                # Translation CAN be (0,0,0) when the lighthouse is at the origin.
                if raw_mat[0][0] == 0.0 and raw_mat[1][1] == 0.0 and raw_mat[2][2] == 0.0:
                    if not self._lighthouse_warned:
                        self.get_logger().warn(
                            f"Reference lighthouse '{self.reference_lighthouse_serial}' found but matrix uninitialized"
                        )
                        self._lighthouse_warned = True
                    return
                pose = convert_to_quaternion(raw_mat)
                self.T_lighthouse_inv = np.linalg.inv(pose_to_matrix(pose))
                if self.frame_id == "vive_world":
                    self.frame_id = self.reference_lighthouse_serial
                self.get_logger().info(
                    f"Using lighthouse {self.reference_lighthouse_serial} as reference frame (frame_id: {self.frame_id})"
                )
                return
        if not self._lighthouse_warned:
            self.get_logger().warn(
                f"Waiting for reference lighthouse '{self.reference_lighthouse_serial}'... "
                f"Available: {[(k, v.get_serial()) for k, v in self.vr.devices.items() if 'tracking_reference' in k]}"
            )
            self._lighthouse_warned = True

    def _init_filters(self):
        params = {
            "freq": 50.0,
            "mincutoff": self.get_parameter("filter.mincutoff").value,
            "beta": self.get_parameter("filter.beta").value,
            "dcutoff": self.get_parameter("filter.dcutoff").value,
        }
        return {axis: OneEuroFilter(**params) for axis in ["x", "y", "z"]}

    def watchdog(self):
        self.vr.poll_vr_events()
        # Retry finding tracker if not yet connected
        if not self.device_name:
            self.device_name = self._find_device_by_serial(self.serial)
            if self.device_name:
                self.get_logger().info(
                    f"Connected to tracker '{self.tracker_name}': {self.serial} (device: {self.device_name})"
                )
        # Retry finding reference lighthouse
        if self.reference_lighthouse_serial and self.T_lighthouse_inv is None:
            self._compute_lighthouse_inverse()

    def update(self):
        if not self.device_name:
            return

        # Don't publish until reference frame is established
        if self.reference_lighthouse_serial and self.T_lighthouse_inv is None:
            return

        tracker = self.vr.devices[self.device_name]
        pose = tracker.get_pose_quaternion()

        if pose is None:
            if self.tracking_active:
                self.get_logger().warn(f"Tracking LOST for '{self.tracker_name}'")
                self.tracking_active = False
            return

        if not self.tracking_active:
            self.get_logger().info(f"Tracking ACQUIRED for '{self.tracker_name}'")
            self.tracking_active = True
            self.haptic_frames_remaining = 50  # Vibrate ~1 second

        # Haptic feedback on tracking acquisition
        if self.haptic_frames_remaining > 0:
            tracker.trigger_haptic_pulse(3000)
            self.haptic_frames_remaining -= 1

        # Transform to lighthouse reference frame
        if self.T_lighthouse_inv is not None:
            T_tracker = pose_to_matrix(pose)
            T_rel = self.T_lighthouse_inv @ T_tracker
            pose = matrix_to_pose(T_rel)

        t = self.get_clock().now().nanoseconds / 1e9

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.filters["x"](pose[0], t)
        msg.pose.position.y = self.filters["y"](pose[1], t)
        msg.pose.position.z = self.filters["z"](pose[2], t)
        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]

        self.pub_pose.publish(msg)


def main():
    rclpy.init()
    node = ViveTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
