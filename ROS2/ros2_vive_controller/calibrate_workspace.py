from ros2_vive_controller.openvr_class.openvr_class import triad_openvr

import rclpy
import math, os, yaml, csv
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

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

        self.header_frame_id = "ci/world"
        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        self.rate = rclpy.Rate(10)
        
        self.create_subscription(PointCloud2, "workspace_pointcloud", self.publish_pointcloud, 10)

        self.get_logger().info("Calibration workspace node started.")

        self.main_loop()

    def main_loop(self):
        while rclpy.ok():
            quaternion_pose = self.v.devices[self.controller_name_right].get_pose_quaternion()
            if quaternion_pose is not None:
                x_pos = quaternion_pose[0]
                y_pos = quaternion_pose[1]
                z_pos = quaternion_pose[2]

                self.all_points.append((x_pos, y_pos, z_pos))
                self.publish_pointcloud()
            
            rclpy.spin_once(self)

    def publish_pointcloud(self):
        if len(self.all_points) > 0:
            points = np.array(self.all_points, dtype=np.float32)
            pointcloud = pc2.create_cloud_xyz32(self.header_frame_id, points)
            self.pc_pub.publish(pointcloud)
        else:
            self.get_logger().warn("No points to publish.")

    def save_points_to_csv(self):
        if len(self.all_points) > 0:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['x', 'y', 'z'])
                for point in self.all_points:
                    writer.writerow(point)
            self.get_logger().info(f"Points saved to {self.csv_path}")
        else:
            self.get_logger().warn("No points to save.")

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
