#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from OneEuroFilter import OneEuroFilter
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr


class ViveTrackerNode(Node):
    def __init__(self):
        super().__init__('vive_tracker')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('name', 'tracker'),
                ('serial', ''),
                ('frame_id', 'vive_world'),
                ('filter.mincutoff', 1.0),
                ('filter.beta', 0.007),
                ('filter.dcutoff', 1.0),
            ]
        )

        self.tracker_name = self.get_parameter('name').value
        self.serial = self.get_parameter('serial').value
        self.frame_id = self.get_parameter('frame_id').value

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
                f"Connected to tracker '{self.tracker_name}': {self.serial} (device: {self.device_name})")

        # Publisher
        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)

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

    def _init_filters(self):
        params = {
            'freq': 50.0,
            'mincutoff': self.get_parameter('filter.mincutoff').value,
            'beta': self.get_parameter('filter.beta').value,
            'dcutoff': self.get_parameter('filter.dcutoff').value,
        }
        return {axis: OneEuroFilter(**params) for axis in ['x', 'y', 'z']}

    def watchdog(self):
        self.vr.poll_vr_events()
        # Retry finding tracker if not yet connected
        if not self.device_name:
            self.device_name = self._find_device_by_serial(self.serial)
            if self.device_name:
                self.get_logger().info(
                    f"Connected to tracker '{self.tracker_name}': {self.serial} (device: {self.device_name})")

    def update(self):
        if not self.device_name:
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

        t = self.get_clock().now().nanoseconds / 1e9

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.filters['x'](pose[0], t)
        msg.pose.position.y = self.filters['y'](pose[1], t)
        msg.pose.position.z = self.filters['z'](pose[2], t)
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


if __name__ == '__main__':
    main()
