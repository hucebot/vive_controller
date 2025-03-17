#!/usr/bin/env python3

import rospy, math, os, yaml, csv
import numpy as np

from openvr_class import triad_openvr, time
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

class CalibrationWS:
    def __init__(self):
        rospy.init_node('calibration_workspace', anonymous=True)

        self.config_file = "/ros_ws/src/ros1_vive_controller/config/config.yaml"
        self.configurations = read_yaml(self.config_file)

        self.v = triad_openvr(self.config_file)
        self.v.print_discovered_objects()

        self.controllers = self.v.return_controller_serials()
        
        self.right_serial = self.configurations['htc_vive']['controller_1']['serial']
        self.csv_path = self.configurations['general']['csv_path']
        self.controller_name_right = self.controllers[self.right_serial]

        self.all_points = []

        self.pc_pub = rospy.Publisher("workspace_pointcloud", PointCloud2, queue_size=10)

        self.header_frame_id = "ci/world"
        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.save_points_to_csv)

        self.main_loop()

    def main_loop(self):
        while not rospy.is_shutdown():
            euler_pose = self.v.devices[self.controller_name_right].get_pose_euler()
            if euler_pose is not None:
                x_pos = euler_pose[0]
                y_pos = euler_pose[1]
                z_pos = euler_pose[2]

                self.all_points.append((x_pos, y_pos, z_pos))
                self.publish_pointcloud()

            else:
                rospy.logwarn("Not receiving pose data from controller. Move the controller to get data.")

            self.rate.sleep()

    def publish_pointcloud(self):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.header_frame_id

        cloud = pc2.create_cloud(header, self.fields, self.all_points)
        self.pc_pub.publish(cloud)

    def save_points_to_csv(self):
        if not os.path.exists(self.csv_path):
            os.makedirs(self.csv_path)
        self.csv_path += "points.csv"
        rospy.loginfo(f"Saving points {self.csv_path}...")

        try:
            with open(self.csv_path, mode='w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(["x", "y", "z"])
                for (x, y, z) in self.all_points:
                    writer.writerow([x, y, z])
            rospy.loginfo("Points saved successfully")
        except Exception as e:
            rospy.logerr(f"Error saving points: {e}")

if __name__ == '__main__':
    calib = CalibrationWS()
