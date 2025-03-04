from ros2_vive_controller.tools.openvr_class import triad_openvr

import rclpy
import math
import yaml
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R


def read_yaml(path):
    with open(path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


class ViveTracker(Node):
    def __init__(self):
        super().__init__('vive_tracker')

        self.config_file = "/ros2_ws/src/ros2_vive_controller/config/config.yaml"
        self.configurations = read_yaml(self.config_file)

        self.v = triad_openvr(self.config_file)
        self.tracker_serials = self.v.return_tracker_serials()

        self.tracker_serial = self.configurations['htc_vive']['tracker_1']['serial']
        self.tracker_name = self.tracker_serials[self.tracker_serial]
        self.get_logger().info(f"Found trackers: {self.tracker_serials}")

        self.marker_publisher = self.create_publisher(Marker, 'vive_tracker_axes', 10)
        self.roll_publisher = self.create_publisher(Float64, '/gstreamer_service/pan', 10)
        self.pitch_publisher = self.create_publisher(Float64, '/gstreamer_service/tilt', 10)

        self.initial_position = None
        self.initial_orientation = None 

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        device = self.v.devices[self.tracker_name]
        euler_pose = device.get_pose_euler()
        device.trigger_haptic_pulse(1000, 0)

        if euler_pose is not None:

            if self.initial_position is None:
                self.set_initial_pose(euler_pose)

            pose_relative, q_rel = self.compute_relative_pose(euler_pose)
            self.publish_arrow_marker(pose_relative)

            rpy_deg = q_rel.as_euler('xyz', degrees=True)
            roll_deg = rpy_deg[0]
            pitch_deg = rpy_deg[1]
            yaw_deg = rpy_deg[2]

            roll_deg = max(min(roll_deg, 50.0), -50.0)
            pitch_deg = max(min(pitch_deg, 50.0), -50.0)

            roll_msg = Float64()
            roll_msg.data = -roll_deg
            self.roll_publisher.publish(roll_msg)

            pitch_msg = Float64()
            pitch_msg.data = -pitch_deg
            self.pitch_publisher.publish(pitch_msg)


    def set_initial_pose(self, euler_pose):
        self.initial_position = euler_pose[:3]
        pitch = math.radians(euler_pose[3])
        yaw   = math.radians(euler_pose[5])
        roll  = math.radians(euler_pose[4])
        q0 = R.from_euler('xyz', [pitch, yaw, roll]).as_quat()
        self.initial_orientation = q0

        self.get_logger().info("Initial pose stored!")

    def compute_relative_pose(self, euler_pose):
        init_y, init_z, init_x = self.initial_position
        cur_y,  cur_z,  cur_x  = euler_pose[0], euler_pose[1], euler_pose[2]

        pose_x = -(cur_y - init_y)
        pose_y =  (cur_x - init_x)
        pose_z =  (cur_z - init_z)

        pitch = math.radians(euler_pose[3])
        roll  = math.radians(euler_pose[4])
        yaw   = math.radians(euler_pose[5])
        q_current = R.from_euler('xyz', [pitch, yaw, roll])

        q0_inv = R.from_quat(self.initial_orientation).inv()
        q_rel = q0_inv * q_current

        pose_msg = Pose()
        pose_msg.position.x = pose_y
        pose_msg.position.y = -pose_x
        pose_msg.position.z = pose_z

        q_rel_array = q_rel.as_quat()
        pose_msg.orientation.x = q_rel_array[0]
        pose_msg.orientation.y = q_rel_array[1]
        pose_msg.orientation.z = q_rel_array[2]
        pose_msg.orientation.w = q_rel_array[3]

        return pose_msg, q_rel

    def publish_arrow_marker(self, pose_relative):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vive_tracker"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.position = pose_relative.position
        marker.pose.orientation = pose_relative.orientation

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    vive_tracker = ViveTracker()
    rclpy.spin(vive_tracker)
    vive_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
