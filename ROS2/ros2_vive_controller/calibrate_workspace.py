#!/usr/bin/env python3
"""
Unified Calibration Node for HTC Vive + ROS 2
=============================================

This node provides a unified workflow for calibrating a safety workspace
using a VR controller. It combines real-time visualization with
automatic configuration updates.

Features:
    - **Live Visualization**: See the points you paint and the resulting bounding box in RViz.
    - **Tracking Feedback**: Visual cursor changes color (Red=Idle, Green=Recording).
    - **Data Safety**: Filters outliers (Z-score) to prevent glitches from ruining the calibration.
    - **Auto-Config**: Writes new limits directly to `config.yaml` upon exit.

Usage:
    Run the node, hold the VR controller trigger, and trace the physical boundaries
    of the safe workspace. Press Ctrl+C to save and exit.
"""

import rclpy
from rclpy.node import Node
import os
import yaml
import csv
import math
import numpy as np

# Imports for visualization
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

# VR Import (Ensure ros2_vive_controller package is built/sourced)
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr

# --- Constants ---
# Path to the main configuration file.
# NOTE: Ensure this path is correct for your specific workspace structure.
CONFIG_PATH = "/ros2_ws/src/ros2_vive_controller/config/config.yaml"

class UnifiedCalibrationNode(Node):
    """
    ROS 2 Node for interactive workspace calibration.
    """

    def __init__(self):
        super().__init__('calibration_node')

        # --- 1. Load Configuration & Initialize VR ---
        self.get_logger().info(f"Loading config from: {CONFIG_PATH}")
        self.cfg = self.read_yaml(CONFIG_PATH)

        # Initialize OpenVR (Triad OpenVR wrapper)
        self.v = triad_openvr(CONFIG_PATH)
        self.v.print_discovered_objects()

        # --- 2. Controller Setup ---
        # Retrieve serial number from config (Keys must match config.yaml structure)
        # Defaulting to 'controller_1' (Right hand in your config)
        try:
            self.serial = self.cfg['htc_vive']['controller_1']['serial']
        except KeyError:
            self.get_logger().error("Could not find 'controller_1' serial in config.yaml")
            raise SystemExit

        # Validate connection
        self.controllers = self.v.return_controller_serials()
        if self.serial not in self.controllers:
            self.get_logger().error(f"Controller with serial {self.serial} not found! Is SteamVR running?")
            raise SystemExit

        self.device_name = self.controllers[self.serial]

        # --- 3. State Variables ---
        self.points = []          # List to store valid (x, y, z) tuples
        self.last_point = None    # Track last recorded point for distance filtering
        self.min_dist_sq = 0.02**2 # Threshold: Move 2cm to record a new point (squared for perf)
        self.frame_id = self.cfg['general']['link_name'] # Frame for RViz (e.g., 'world')

        # --- 4. Publishers ---
        # Publishes the raw points painted by the user
        self.pub_cloud = self.create_publisher(PointCloud2, '/calibration/cloud', 10)
        # Publishes the calculated bounding box (cube)
        self.pub_box = self.create_publisher(Marker, '/calibration/bounding_box', 10)
        # Publishes a small cube at the controller tip (Cursor)
        self.pub_status = self.create_publisher(Marker, '/calibration/controller_status', 10)

        # --- 5. Main Loop ---
        # Run at 50Hz (0.02s period)
        self.create_timer(0.02, self.loop)

        self.get_logger().info("READY. Hold TRIGGER to paint workspace.")
        self.get_logger().info("Visual Feedback: RED Box = Tracked, GREEN Box = Recording.")

    def loop(self):
        """
        Main control loop:
        1. Reads VR device pose.
        2. Visualizes status.
        3. Records points if Trigger is pressed.
        """
        # Get device object wrapper
        device = self.v.devices.get(self.device_name)

        # Safety Check: Device lost?
        if device is None:
            self.get_logger().warn("Controller not found in OpenVR list!", throttle_duration_sec=2.0)
            return

        # Get Pose (returns [x, y, z, qx, qy, qz, qw] or None)
        pose = device.get_pose_quaternion()
        if pose is None:
            self.get_logger().warn("Controller connected but NOT TRACKING (sensors blocked?)", throttle_duration_sec=1.0)
            return

        # If we reach here, tracking is valid
        inputs = device.get_controller_inputs()
        trigger_val = inputs.get('trigger', 0.0)

        # Logic: Record if trigger pressed > 50%
        is_recording = trigger_val > 0.5

        # Extract Position (x, y, z)
        pos = np.array(pose[:3])

        # --- Visual Feedback ---
        self.publish_status_marker(pos, is_recording)

        # --- Recording Logic ---
        if is_recording:
            if self.should_record(pos):
                self.points.append(pos)
                self.last_point = pos

                # Update RViz elements immediately
                self.publish_cloud()
                self.publish_bbox()

                # Throttle log to prevent console spam
                self.get_logger().info(f"Recording... Total Points: {len(self.points)}", throttle_duration_sec=0.5)

    def should_record(self, pos: np.ndarray) -> bool:
        """
        Distance Filter: Returns True if 'pos' is far enough from 'last_point'.
        Prevents recording thousands of duplicate points when standing still.
        """
        if self.last_point is None:
            return True

        # Calculate squared Euclidean distance
        dist_sq = np.sum((pos - self.last_point)**2)
        return dist_sq > self.min_dist_sq

    # --- Visualization Helpers ---

    def publish_status_marker(self, pos: np.ndarray, is_recording: bool):
        """
        Publishes a small cube at the controller's tip.
        Color indicates state: RED (Idle) vs GREEN (Recording).
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "status_cursor"
        marker.id = 999
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Size (5cm cube)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Position
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.pose.orientation.w = 1.0

        # Color Logic
        marker.color.a = 1.0
        if is_recording:
            # Green
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        else:
            # Red
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0

        self.pub_status.publish(marker)

    def publish_cloud(self):
        """Converts internal point list to ROS PointCloud2 message."""
        if not self.points:
            return

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()

        # sensor_msgs_py helper makes this easy
        self.pub_cloud.publish(pc2.create_cloud_xyz32(header, self.points))

    def publish_bbox(self):
        """Calculates and publishes the Axis-Aligned Bounding Box (AABB) of the current points."""
        if len(self.points) < 4:
            return

        # Convert to numpy for fast math
        pts = np.array(self.points)
        min_v = np.min(pts, axis=0)
        max_v = np.max(pts, axis=0)

        # Calculate Center and Dimensions
        center = (min_v + max_v) / 2.0
        dims = max_v - min_v

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace_bbox"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = dims[0]
        marker.scale.y = dims[1]
        marker.scale.z = dims[2]

        # Transparent Green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        self.pub_box.publish(marker)

    # --- Data Persistence & Processing ---

    def save_and_exit(self):
        """
        Called on Shutdown.
        1. Saves raw points to CSV.
        2. Filters outliers (Z-Score).
        3. Updates config.yaml with new workspace limits.
        """
        if not self.points:
            self.get_logger().warn("No points recorded. Exiting without save.")
            return

        self.get_logger().info("--- Shutdown Sequence Initiated ---")
        self.get_logger().info(f"Processing {len(self.points)} raw points...")

        pts = np.array(self.points)

        # 1. Save Raw CSV (Backup)
        try:
            csv_path = os.path.join(self.cfg['general']['csv_path'], "points.csv")
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            np.savetxt(csv_path, pts, delimiter=",", header="x,y,z", comments="")
            self.get_logger().info(f"Backup saved to: {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

        # 2. Filter Outliers (Z-Score Algorithm)
        # Removes points that are statistically too far from the cluster center (glitches)
        z_thresh = self.cfg['general'].get('z_threshold', 3.0)

        mean = np.mean(pts, axis=0)
        std = np.std(pts, axis=0)
        std[std == 0] = 1e-9 # Avoid division by zero

        z_scores = np.abs((pts - mean) / std)
        mask = (z_scores < z_thresh).all(axis=1)

        clean_pts = pts[mask]
        outlier_count = len(pts) - len(clean_pts)

        if len(clean_pts) == 0:
            self.get_logger().error("All points were filtered out! Try increasing z_threshold.")
            return

        self.get_logger().info(f"Filtering complete. Removed {outlier_count} outliers.")

        # 3. Calculate Final Limits
        min_v = np.min(clean_pts, axis=0)
        max_v = np.max(clean_pts, axis=0)

        # 4. Update YAML Configuration
        if 'workspace' not in self.cfg:
            self.cfg['workspace'] = {}

        ws = self.cfg['workspace']

        # Rounding to 2 decimal places for readability in the file
        ws['x_min'], ws['x_max'] = round(float(min_v[0]), 2), round(float(max_v[0]), 2)
        ws['y_min'], ws['y_max'] = round(float(min_v[1]), 2), round(float(max_v[1]), 2)
        ws['z_min'], ws['z_max'] = round(float(min_v[2]), 2), round(float(max_v[2]), 2)

        try:
            with open(CONFIG_PATH, 'w') as f:
                yaml.dump(self.cfg, f, default_flow_style=False)
            self.get_logger().info(f"SUCCESS. Config updated at: {CONFIG_PATH}")
            self.get_logger().info(f"New Bounds -> X: [{ws['x_min']}, {ws['x_max']}]")
        except Exception as e:
            self.get_logger().error(f"Failed to write config.yaml: {e}")

    def read_yaml(self, path):
        """Helper to safely read YAML files."""
        try:
            with open(path, 'r') as s:
                return yaml.safe_load(s) or {}
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {path}")
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"YAML Syntax Error: {e}")
            return {}

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C
        pass
    finally:
        # Ensure data is saved before killing the node
        node.save_and_exit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()