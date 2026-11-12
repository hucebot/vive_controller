#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from ros2_vive_controller import station as st
from ros2_vive_controller.workspace import bounds_from_points


class CalibrationNode(Node):
    """Paint one arm's workspace with its controller and save it as that arm's fence.

    The bounds are written to the station file (config/station.yaml, ``workspace.<side>``) together
    with the frame the poses were in, so the driver only applies them in that same frame. The setup
    app (`make setup`) does the same from a browser.
    """

    def __init__(self):
        super().__init__('vive_calibration')

        # --- Parameters ---
        self.declare_parameter('station_file', '/ros2_ws/src/ros2_vive_controller/config/station.yaml')
        self.declare_parameter('side', 'right')
        # Grow the painted box by this much on every face (m).
        self.declare_parameter('margin', 0.05)

        self.station_file = self.get_parameter('station_file').value
        self.side = self.get_parameter('side').value
        self.margin = self.get_parameter('margin').value

        # --- State ---
        self.points = []
        self.is_recording = False
        self.current_pose = None
        self.frame_id = None

        # --- Subscribers ---
        self.sub_pose = self.create_subscription(
            PoseStamped, f'/vive/{self.side}/pose', self.pose_cb, 10)
        self.sub_joy = self.create_subscription(
            JointState, f'/vive/{self.side}/joint_states', self.joy_cb, 10)

        # --- Publishers ---
        self.pub_marker = self.create_publisher(
            Marker, 'calibration_marker', 10)
        self.pub_bbox = self.create_publisher(Marker, 'calibration_bbox', 10)

        self.pub_is_calibrating = self.create_publisher(
            Bool,
            '/vive/is_calibrating',
            10
        )

        # Heartbeat at 10 Hz: while it is received, the driver's workspace fence is off.
        self.create_timer(0.1, self.publish_heartbeat)
        self.publish_heartbeat()

        self.get_logger().info("AUTO-SAVE CALIBRATION READY.")
        self.get_logger().info(f"Target file: {self.station_file} ({self.side} arm)")
        self.get_logger().info(f"Hold the {self.side.upper()} TRIGGER to paint. Press CTRL+C to SAVE and EXIT.")

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.pub_is_calibrating.publish(msg)

    def pose_cb(self, msg):
        frame = msg.header.frame_id
        if self.frame_id is not None and frame != self.frame_id:
            # E.g. the reference lighthouse was found after painting started.
            self.get_logger().warn(
                f"Pose frame changed from '{self.frame_id}' to '{frame}': discarding {len(self.points)} points.")
            self.points = []
        self.frame_id = frame
        self.current_pose = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ])

    def joy_cb(self, msg):
        if self.current_pose is None:
            return

        try:
            trigger_idx = msg.name.index('trigger')
            trigger_val = msg.position[trigger_idx]
        except ValueError:
            return

        if trigger_val > 0.5:
            if not self.is_recording:
                self.get_logger().info(f"Recording in frame '{self.frame_id}'...")
            self.is_recording = True

            # Distance Filter (2cm)
            if not self.points or np.linalg.norm(self.current_pose - self.points[-1]) > 0.02:
                self.points.append(self.current_pose)
                self.publish_markers()
        else:
            self.is_recording = False

    def publish_markers(self):
        if not self.points:
            return

        # 1. Points
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "cloud"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.scale.y = 0.02
        m.color.g = 1.0
        m.color.a = 1.0

        for p in self.points:
            pt = Marker().pose.position
            pt.x, pt.y, pt.z = p[0], p[1], p[2]
            m.points.append(pt)
        self.pub_marker.publish(m)

        # 2. Bounding Box
        pts = np.array(self.points)
        min_v = np.min(pts, axis=0)
        max_v = np.max(pts, axis=0)
        center = (min_v + max_v) / 2.0
        dims = max_v - min_v

        box = Marker()
        box.header.frame_id = self.frame_id
        box.header.stamp = self.get_clock().now().to_msg()
        box.ns = "bbox"
        box.id = 1
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x, box.pose.position.y, box.pose.position.z = center
        box.scale.x, box.scale.y, box.scale.z = dims
        box.color.b = 1.0
        box.color.a = 0.3
        self.pub_bbox.publish(box)

    def save_and_exit(self):
        if len(self.points) < 10:
            self.get_logger().warn("Not enough points. Exiting without save.")
            return

        bounds = bounds_from_points(self.points, margin=self.margin)
        if bounds is None:
            self.get_logger().error("All points filtered out! Try painting more consistently.")
            return

        try:
            station = st.set_fence(st.load(self.station_file), self.side, bounds, self.frame_id)
            backup = st.save(self.station_file, station, writer='calibration_node (make calibrate)')
            if backup:
                self.get_logger().info(f"Backup created at: {backup}")
            self.get_logger().info("="*40)
            self.get_logger().info(f"SUCCESS! {self.side.capitalize()} arm fence saved: {self.station_file}")
            self.get_logger().info(f"Frame: {self.frame_id}")
            for axis in 'xyz':
                self.get_logger().info(f"  {axis}: [{bounds[axis + '_min']}, {bounds[axis + '_max']}]")
            self.get_logger().info("The fence is active the next time the drivers start.")
            self.get_logger().info("="*40)
        except Exception as e:
            self.get_logger().error(f"Failed to save workspace: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
