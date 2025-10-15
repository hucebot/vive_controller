#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import yaml
import csv
import numpy as np

from ros2_vive_controller.openvr_class.openvr_class import triad_openvr
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

def read_yaml(path):
    with open(path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

def write_yaml(path, data):
    with open(path, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

class CalibrationWS(Node):
    def __init__(self):
        super().__init__('calibration_workspace')

        self.config_file = "/ros2_ws/src/ros2_vive_controller/config/config.yaml"
        self.configurations = read_yaml(self.config_file)

        self.v = triad_openvr(self.config_file)
        self.v.print_discovered_objects()

        self.controllers = self.v.return_controller_serials()
        self.right_serial = self.configurations['htc_vive']['controller_1']['serial']
        self.csv_path = self.configurations['general']['csv_path']
        self.controller_name_right = self.controllers[self.right_serial]

        self.all_points = []

        self.pc_pub = self.create_publisher(PointCloud2, "workspace_pointcloud", 10)

        self.header_frame_id = self.configurations['general']['link_name']
        self.rate = 10

        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.main_loop)

        import atexit
        atexit.register(self.save_points_to_csv)

    def main_loop(self):
        quaternion_pose = self.v.devices[self.controller_name_right].get_pose_quaternion()
        if quaternion_pose is not None:
            x_pos = float(quaternion_pose[0])
            y_pos = float(quaternion_pose[1])
            z_pos = float(quaternion_pose[2])
            self.all_points.append((x_pos, y_pos, z_pos))
            self.publish_pointcloud()
        else:
            self.get_logger().warn("Not receiving pose data from controller. Move the controller to get data.")

    def publish_pointcloud(self):
        if not self.all_points:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.header_frame_id

        pointcloud = pc2.create_cloud_xyz32(header, self.all_points)
        self.pc_pub.publish(pointcloud)

    def save_points_to_csv(self):
        if not self.all_points:
            self.get_logger().warn("No points to save.")
            return

        # Crear carpeta si no existe
        if not os.path.exists(self.csv_path):
            os.makedirs(self.csv_path, exist_ok=True)

        csv_file = os.path.join(self.csv_path, "points.csv")
        self.get_logger().info(f"Saving points to {csv_file}...")
        try:
            with open(csv_file, mode='w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(["x", "y", "z"])
                for (x, y, z) in self.all_points:
                    writer.writerow([x, y, z])
            self.get_logger().info("Points saved successfully")
        except Exception as e:
            self.get_logger().error(f"Error saving points: {e}")

def main(args=None):
    rclpy.init(args=args)
    calibration_ws = CalibrationWS()
    try:
        rclpy.spin(calibration_ws)
    except KeyboardInterrupt:
        pass
    finally:
        calibration_ws.save_points_to_csv()
        calibration_ws.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
