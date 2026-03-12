#!/usr/bin/env python3
"""Base class for Vive tracker nodes.

Provides shared OpenVR infrastructure: device discovery, lighthouse reference
frame calibration, OneEuroFilter initialization, and watchdog polling.
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


def _parse_offset(val):
    """Parse a hand offset parameter to a 4x4 homogeneous matrix.

    Accepts a list of 7 floats [x, y, z, qx, qy, qz, qw] or a string
    representation of that list (as produced by ROS2 launch arguments).
    """
    if isinstance(val, str):
        import ast
        val = ast.literal_eval(val)
    return pose_to_matrix(val)


class BaseViveTracker3_0Node(Node):
    """Base class for Vive tracker nodes.

    Subclasses must:
      1. Call self._declare_common_parameters() early in __init__
      2. Initialize self.vr = triad_openvr() before calling self._init_lighthouse()
      3. Call self._init_lighthouse() after vr is ready
    """

    def _declare_common_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("reference_lighthouse_serial", ""),
                ("frame_id", "vive_world"),
                ("filter.mincutoff", 1.0),
                ("filter.beta", 0.007),
                ("filter.dcutoff", 1.0),
                ("hand_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            ],
        )

    def _init_hand_offset(self):
        """Parse hand_offset parameter into self.T_tracker_to_hand (4x4 matrix)."""
        self.T_tracker_to_hand = _parse_offset(self.get_parameter("hand_offset").value)

    def _init_lighthouse(self):
        """Initialize lighthouse state. Call after vr and parameters are set up."""
        self.reference_lighthouse_serial = self.get_parameter(
            "reference_lighthouse_serial"
        ).value
        self.frame_id = self.get_parameter("frame_id").value
        self.T_lighthouse_inv = None
        self._lighthouse_retry_count = 0
        if self.reference_lighthouse_serial:
            self._compute_lighthouse_inverse()

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
        self._lighthouse_retry_count += 1

        for key, dev in self.vr.devices.items():
            if dev.get_serial() == self.reference_lighthouse_serial:
                poses = get_pose(self.vr.vr)
                raw_mat = poses[dev.index].mDeviceToAbsoluteTracking
                # Check rotation diagonal — an uninitialized matrix is all zeros,
                # while any valid rotation has non-zero diagonal elements.
                # Translation CAN be (0,0,0) when the lighthouse is at the origin.
                if raw_mat[0][0] == 0.0 and raw_mat[1][1] == 0.0 and raw_mat[2][2] == 0.0:
                    if self._lighthouse_retry_count % 100 == 1:
                        self.get_logger().warn(
                            f"Reference lighthouse '{self.reference_lighthouse_serial}' "
                            f"found but matrix uninitialized (retry {self._lighthouse_retry_count})"
                        )
                    return
                pose = convert_to_quaternion(raw_mat)
                self.T_lighthouse_inv = np.linalg.inv(pose_to_matrix(pose))
                if self.frame_id == "vive_world":
                    self.frame_id = self.reference_lighthouse_serial
                self.get_logger().info(
                    f"Using lighthouse {self.reference_lighthouse_serial} as reference frame "
                    f"(frame_id: {self.frame_id})"
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

    def _publish_tracker(self, publisher, pub_hand, T_tracker_to_hand, filters, pose):
        """Transform, filter, and publish a single tracker's pose + hand pose."""
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

    def watchdog(self):
        """Base watchdog: poll VR events and retry lighthouse calibration."""
        self.vr.poll_vr_events()
        if self.reference_lighthouse_serial and self.T_lighthouse_inv is None:
            self._compute_lighthouse_inverse()
