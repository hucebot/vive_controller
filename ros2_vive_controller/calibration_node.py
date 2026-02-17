#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 1. NEW IMPORTS FOR LATCHING
import numpy as np
import yaml
import os
import shutil

# ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('vive_calibration')

        # --- Parameters ---
        self.declare_parameter('target_frame', 'vive_world')
        self.declare_parameter(
            'config_path', '/ros2_ws/src/ros2_vive_controller/config/vive.params.yaml')

        self.target_frame = self.get_parameter('target_frame').value
        self.config_path = self.get_parameter('config_path').value

        # --- State ---
        self.points = []
        self.is_recording = False
        self.current_pose = None

        # --- Subscribers ---
        self.sub_pose = self.create_subscription(
            PoseStamped, '/vive/right/pose', self.pose_cb, 10)
        self.sub_joy = self.create_subscription(
            JointState, '/vive/right/joint_states', self.joy_cb, 10)

        # --- Publishers ---
        self.pub_marker = self.create_publisher(
            Marker, 'calibration_marker', 10)
        self.pub_bbox = self.create_publisher(Marker, 'calibration_bbox', 10)

        self.pub_is_calibrating = self.create_publisher(
            Bool,
            '/vive/is_calibrating',
            10
        )

        # Publish heartbeat at 10Hz (Fast enough to keep driver happy)
        self.create_timer(0.1, self.publish_heartbeat)

        # This message will now stick for any node that subscribes later
        self.pub_is_calibrating.publish(Bool(data=True))

        self.get_logger().info("AUTO-SAVE CALIBRATION READY.")
        self.get_logger().info(f"Target Config: {self.config_path}")
        self.get_logger().info("Hold RIGHT TRIGGER to paint. Press CTRL+C to SAVE and EXIT.")

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.pub_is_calibrating.publish(msg)

    def pose_cb(self, msg):
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
                self.get_logger().info("Recording...")
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
        m.header.frame_id = self.target_frame
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
        box.header.frame_id = self.target_frame
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
        # Update the latched status to False so late joiners know we are done
        if len(self.points) < 10:
            self.get_logger().warn("Not enough points. Exiting without save.")
            return

        # 1. Calculation
        pts = np.array(self.points)
        mean = np.mean(pts, axis=0)
        std = np.std(pts, axis=0)
        clean_pts = pts[(np.abs((pts - mean) / (std + 1e-9))
                         < 3.0).all(axis=1)]

        if len(clean_pts) == 0:
            self.get_logger().error("All points filtered out! Try painting more consistently.")
            return

        min_v = np.min(clean_pts, axis=0)
        max_v = np.max(clean_pts, axis=0)

        padding = 0.05

        # 2. Backup existing file
        if os.path.exists(self.config_path):
            backup_path = self.config_path + ".bak"
            shutil.copy(self.config_path, backup_path)
            self.get_logger().info(f"Backup created at: {backup_path}")

        # 3. Load & Update YAML
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f) or {}

            if '/**' not in data:
                data['/**'] = {}
            if 'ros__parameters' not in data['/**']:
                data['/**']['ros__parameters'] = {}

            params = data['/**']['ros__parameters']
            if 'workspace' not in params:
                params['workspace'] = {}

            ws = params['workspace']
            ws['x_min'] = float(round(min_v[0] - padding, 3))
            ws['x_max'] = float(round(max_v[0] + padding, 3))
            ws['y_min'] = float(round(min_v[1] - padding, 3))
            ws['y_max'] = float(round(max_v[1] + padding, 3))
            ws['z_min'] = float(round(min_v[2] - padding, 3))
            ws['z_max'] = float(round(max_v[2] + padding, 3))

            with open(self.config_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.get_logger().info("="*40)
            self.get_logger().info(
                f"SUCCESS! Config updated: {self.config_path}")
            self.get_logger().info(
                f"New Bounds: X[{ws['x_min']}, {ws['x_max']}]")
            self.get_logger().info("="*40)

        except Exception as e:
            self.get_logger().error(f"Failed to save config: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_exit()
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
