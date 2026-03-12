#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr
from ros2_vive_controller.base_tracker_node import BaseViveTracker3_0Node


class ViveTracker3_0Node(BaseViveTracker3_0Node):
    def __init__(self):
        super().__init__("vive_tracker")

        self._declare_common_parameters()
        self.declare_parameters(
            namespace="",
            parameters=[
                ("name", "tracker"),
                ("serial", ""),
            ],
        )

        self.tracker_name = self.get_parameter("name").value
        self.serial = self.get_parameter("serial").value

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

        self._init_lighthouse()
        self._init_hand_offset()

        # Publishers
        self.pub_pose = self.create_publisher(
            PoseStamped, f"/vive_{self.tracker_name}/pose", 10
        )
        self.pub_pose_hand = self.create_publisher(
            PoseStamped, f"/vive_{self.tracker_name}/pose_hand", 10
        )

        # Filters
        self.filters = self._init_filters()

        # Tracking state
        self.tracking_active = False
        self.haptic_frames_remaining = 0

        # Timers
        self.create_timer(0.02, self.update)   # 50Hz
        self.create_timer(0.05, self.watchdog)  # 20Hz

    def watchdog(self):
        super().watchdog()
        if not self.device_name:
            self.device_name = self._find_device_by_serial(self.serial)
            if self.device_name:
                self.get_logger().info(
                    f"Connected to tracker '{self.tracker_name}': {self.serial} (device: {self.device_name})"
                )

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

        self._publish_tracker(
            self.pub_pose, self.pub_pose_hand, self.T_tracker_to_hand, self.filters, pose
        )


def main():
    rclpy.init()
    node = ViveTracker3_0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
