#!/usr/bin/env python3
"""Bimanual Vive Tracker Node.

Manages two trackers with a single OpenVR instance.
Does NOT publish until both trackers AND the reference lighthouse are detected.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr
from ros2_vive_controller.base_tracker_node import (
    BaseViveTracker3_0Node,
    _parse_offset,
)


class BimanualViveTracker3_0Node(BaseViveTracker3_0Node):
    def __init__(self):
        super().__init__("bimanual_vive_tracker")

        self._declare_common_parameters()
        self.declare_parameters(
            namespace="",
            parameters=[
                ("serial_left", ""),
                ("serial_right", ""),
                ("hand_offset_left",  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
                ("hand_offset_right", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            ],
        )

        self.serial_left = self.get_parameter("serial_left").value
        self.serial_right = self.get_parameter("serial_right").value

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

        self._init_lighthouse()

        # Hand offset transforms (T_tracker_to_hand): [x, y, z, qx, qy, qz, qw]
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
        self.create_timer(0.02, self.update)   # 50Hz
        self.create_timer(0.05, self.watchdog)  # 20Hz

    def watchdog(self):
        super().watchdog()
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

    def _is_ready(self):
        """Both trackers found and lighthouse reference established."""
        if self.reference_lighthouse_serial and self.T_lighthouse_inv is None:
            return False
        if not self.device_left or not self.device_right:
            return False
        return True

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
                self.pub_left, self.pub_left_hand,
                self.T_tracker_to_hand_left, self.filters_left, pose_left
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
                self.pub_right, self.pub_right_hand,
                self.T_tracker_to_hand_right, self.filters_right, pose_right
            )


def main():
    rclpy.init()
    node = BimanualViveTracker3_0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
