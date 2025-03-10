#!/usr/bin/env python3
from openvr_class import triad_openvr
import rospy, math, yaml, numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse, euler_from_quaternion
from OneEuroFilter import OneEuroFilter

def read_yaml(path):
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)

class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)
        self.config_file = "/ros_ws/src/ros1_vive_controller/config/config.yaml"
        self.configurations = read_yaml(self.config_file)
        self.robot_type = self.configurations['general']['robot']
        self.v = triad_openvr(self.config_file)
        self.v.print_discovered_objects()
        self.controllers = self.v.return_controller_serials()
        self.publish_markers = self.configurations['general']['publish_markers']
        self.use_left_controller = self.configurations['general']['use_left_controller']
        self.use_right_controller = self.configurations['general']['use_right_controller']
        self.rate = rospy.Rate(self.configurations['general']['rate'])
        self.linear_scale = self.configurations['general']['linear_scale']
        self.angular_scale = self.configurations['general']['angular_scale']
        self.move_base = self.configurations['general']['move_base']

        if self.use_right_controller:
            self.right_serial = self.configurations['htc_vive']['controller_1']['serial']
            self.position_publisher_right = rospy.Publisher(
                self.configurations['general']['right_position_topic'],
                PoseStamped,
                queue_size=10
            )
            self.gripper_publisher_right = rospy.Publisher(
                self.configurations['general']['right_gripper_topic'],
                PointStamped,
                queue_size=10
            )
            if self.robot_type == 'talos':
                self.position_publisher_right = rospy.Publisher(
                    self.configurations['general']['talos_position_topic'],
                    PoseStamped,
                    queue_size=10
                )
                self.gripper_publisher_right = rospy.Publisher(
                    self.configurations['general']['talos_gripper_topic'],
                    PointStamped,
                    queue_size=10
                )
            self.controller_name_right = self.controllers[self.right_serial]
            self.right_trigger_active = False
            self.right_reference_position = None
            self.right_cumulative_x = 0
            self.right_cumulative_y = 0
            self.right_cumulative_z = 0
            self.right_initial_orientation = None
            if self.publish_markers:
                self.marker_publisher_right = rospy.Publisher(
                    self.configurations['general']['right_marker_topic'],
                    Marker,
                    queue_size=10
                )
            if self.move_base:
                self.move_base_angular_publisher = rospy.Publisher(
                    self.configurations['general']['move_base_angular_topic'],
                    Float32,
                    queue_size=10
                )

        if self.use_left_controller:
            self.left_serial = self.configurations['htc_vive']['controller_2']['serial']
            self.position_publisher_left = rospy.Publisher(
                self.configurations['general']['left_position_topic'],
                PoseStamped,
                queue_size=10
            )
            self.gripper_publisher_left = rospy.Publisher(
                self.configurations['general']['left_gripper_topic'],
                PointStamped,
                queue_size=10
            )
            self.controller_name_left = self.controllers[self.left_serial]
            self.left_trigger_active = False
            self.left_reference_position = None
            self.left_cumulative_x = 0
            self.left_cumulative_y = 0
            self.left_cumulative_z = 0
            self.left_initial_orientation = None
            if self.publish_markers:
                self.marker_publisher_left = rospy.Publisher(
                    self.configurations['general']['left_marker_topic'],
                    Marker,
                    queue_size=10
                )
            if self.move_base:
                self.move_base_linear_x_publisher = rospy.Publisher(
                    self.configurations['general']['move_base_linear_x_topic'],
                    Float32,
                    queue_size=10
                )
                self.move_base_linear_y_publisher = rospy.Publisher(
                    self.configurations['general']['move_base_linear_y_topic'],
                    Float32,
                    queue_size=10
                )

        self.pose_msg = PoseStamped()
        self.gripper_msg = PointStamped()
        self.workspace_limit = self.configurations['general']['workspace_limit']
        self.x_max = self.configurations['workspace']['x_max']
        self.x_min = self.configurations['workspace']['x_min']
        self.y_max = self.configurations['workspace']['y_max']
        self.y_min = self.configurations['workspace']['y_min']
        self.z_max = self.configurations['workspace']['z_max']
        self.z_min = self.configurations['workspace']['z_min']
        if self.publish_markers:
            self.workspace_marker_pub = rospy.Publisher("workspace_bbox_marker", Marker, queue_size=10)
            self.actual_pose_marker_pub = rospy.Publisher("actual_pose_marker", Marker, queue_size=10)

        cfg = {
            'freq': self.configurations['general']['rate'],
            'mincutoff': 1.0,
            'beta': 0.1,
            'dcutoff': 1.0
        }
        self.right_filter_x = OneEuroFilter(**cfg)
        self.right_filter_y = OneEuroFilter(**cfg)
        self.right_filter_z = OneEuroFilter(**cfg)
        self.right_filter_roll = OneEuroFilter(**cfg)
        self.right_filter_pitch = OneEuroFilter(**cfg)
        self.right_filter_yaw = OneEuroFilter(**cfg)
        self.left_filter_x = OneEuroFilter(**cfg)
        self.left_filter_y = OneEuroFilter(**cfg)
        self.left_filter_z = OneEuroFilter(**cfg)
        self.left_filter_roll = OneEuroFilter(**cfg)
        self.left_filter_pitch = OneEuroFilter(**cfg)
        self.left_filter_yaw = OneEuroFilter(**cfg)

        self.main_loop()

    def publish_axes_marker(self, frame_id, pose, marker_publisher):
        axis_length = 0.2
        axis_diameter = 0.015
        def make_arrow_marker(id, color, orientation_offset):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time.now()
            m.ns = "joystick_axes"
            m.id = id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = axis_length
            m.scale.y = axis_diameter
            m.scale.z = axis_diameter
            m.color.a = 1.0
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = pose.position.z
            q_pose = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            q_final = quaternion_multiply(q_pose, orientation_offset)
            m.pose.orientation.x = q_final[0]
            m.pose.orientation.y = q_final[1]
            m.pose.orientation.z = q_final[2]
            m.pose.orientation.w = q_final[3]
            return m
        q_id = [0, 0, 0, 1]
        q_y = quaternion_from_euler(0, 0, math.pi/2)
        q_z = quaternion_from_euler(0, -math.pi/2, 0)
        marker_x = make_arrow_marker(0, (1.0, 0.0, 0.0), q_id)
        marker_y = make_arrow_marker(1, (0.0, 1.0, 0.0), q_y)
        marker_z = make_arrow_marker(2, (0.0, 0.0, 1.0), q_z)
        marker_publisher.publish(marker_x)
        marker_publisher.publish(marker_y)
        marker_publisher.publish(marker_z)

    def publish_workspace_bbox_marker(self):
        marker = Marker()
        marker.header.frame_id = "ci/world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "workspace"
        marker.id = 1000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = self.x_max - self.x_min
        marker.scale.y = self.y_max - self.y_min
        marker.scale.z = self.z_max - self.z_min
        marker.pose.position.x = (self.x_max + self.x_min) / 2.0
        marker.pose.position.y = (self.y_max + self.y_min) / 2.0
        marker.pose.position.z = (self.z_max + self.z_min) / 2.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.1
        self.workspace_marker_pub.publish(marker)

    def parse_data_device(self, device, side, position_publisher, gripper_publisher, marker_publisher,
                          fx_filter, fy_filter, fz_filter, fr_filter, fp_filter, fw_filter):
        euler_pose = device.get_pose_euler()
        controller_inputs = device.get_controller_inputs()
        if euler_pose is None or controller_inputs is None:
            rospy.logwarn("No data received from device. Move the controller to receive data.")
            return
        px, py, pz = euler_pose[0], euler_pose[2], euler_pose[1]
        if (px < self.y_min + self.workspace_limit or px > self.y_max - self.workspace_limit or
            py < self.x_min + self.workspace_limit or py > self.x_max - self.workspace_limit or
            pz < self.z_min + self.workspace_limit or pz > self.z_max - self.workspace_limit):
            device.trigger_haptic_pulse(1000, 0)
            return
        trigger_value = controller_inputs.get('trigger', 0)
        trackpad_pressed = controller_inputs.get('trackpad_pressed', 0)
        trackpad_touched = controller_inputs.get('trackpad_touched', 0)
        menu_button = controller_inputs.get('menu_button', 0)
        gripper_button = controller_inputs.get('grip_button', 0)

        if self.publish_markers:
            actual_pose_marker = Marker()
            actual_pose_marker.header.frame_id = "ci/world"
            actual_pose_marker.header.stamp = rospy.Time.now()
            actual_pose_marker.ns = "actual_pose"
            actual_pose_marker.id = 0
            actual_pose_marker.type = Marker.SPHERE
            actual_pose_marker.action = Marker.ADD
            actual_pose_marker.scale.x = 0.05
            actual_pose_marker.scale.y = 0.05
            actual_pose_marker.scale.z = 0.05
            actual_pose_marker.pose.position.x = py
            actual_pose_marker.pose.position.y = px
            actual_pose_marker.pose.position.z = pz
            actual_pose_marker.pose.orientation.w = 1.0
            actual_pose_marker.color.r = 1.0
            actual_pose_marker.color.g = 0.0
            actual_pose_marker.color.b = 0.0
            actual_pose_marker.color.a = 1.0
            self.actual_pose_marker_pub.publish(actual_pose_marker)

        if side == "right" and self.use_right_controller:
            if self.right_initial_orientation is None:
                self.right_initial_orientation = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
            if gripper_button:
                self.right_initial_orientation = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
            if trigger_value < 0.5:
                if self.right_trigger_active:
                    self.right_trigger_active = False
                    self.right_cumulative_x += euler_pose[0] - self.right_reference_position[0]
                    self.right_cumulative_y += euler_pose[2] - self.right_reference_position[2]
                    self.right_cumulative_z += euler_pose[1] - self.right_reference_position[1]
            else:
                if not self.right_trigger_active:
                    self.right_trigger_active = True
                    self.right_reference_position = euler_pose[:3]
                if trackpad_touched and trackpad_pressed and self.move_base:
                    trackpad_x = controller_inputs.get('trackpad_x', 0)
                    vel_ang = trackpad_x * self.angular_scale
                    self.move_base_angular_publisher.publish(vel_ang)
                dx = euler_pose[0] - self.right_reference_position[0]
                dy = euler_pose[2] - self.right_reference_position[2]
                dz = euler_pose[1] - self.right_reference_position[1]
                out_x = self.right_cumulative_x + dx
                out_y = self.right_cumulative_y + dy
                out_z = self.right_cumulative_z + dz
                filtered_pose_x = fx_filter(out_x, rospy.Time.now().to_sec())
                filtered_pose_y = fy_filter(out_y, rospy.Time.now().to_sec())
                filtered_pose_z = fz_filter(out_z, rospy.Time.now().to_sec())
                q0_inv = quaternion_inverse(self.right_initial_orientation)
                q_current = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
                q_rel = quaternion_multiply(q0_inv, q_current)
                q_rel = (q_rel[0], q_rel[1], -q_rel[2], q_rel[3])
                r_rel, p_rel, y_rel = euler_from_quaternion(q_rel)
                roll = math.degrees(r_rel)
                pitch = math.degrees(p_rel)
                yaw = math.degrees(y_rel)
                filtered_roll = fr_filter(roll, rospy.Time.now().to_sec())
                filtered_pitch = fp_filter(pitch, rospy.Time.now().to_sec())
                filtered_yaw = fw_filter(yaw, rospy.Time.now().to_sec())
                filtered_q = quaternion_from_euler(
                    math.radians(filtered_roll),
                    math.radians(filtered_pitch),
                    math.radians(filtered_yaw)
                )
                self.pose_msg.pose.position.x = filtered_pose_y
                self.pose_msg.pose.position.y = filtered_pose_x
                self.pose_msg.pose.position.z = filtered_pose_z
                self.pose_msg.pose.orientation.x = filtered_q[0]
                self.pose_msg.pose.orientation.y = filtered_q[1]
                self.pose_msg.pose.orientation.z = filtered_q[2]
                self.pose_msg.pose.orientation.w = filtered_q[3]
                self.pose_msg.header.frame_id = "ci/world"
                self.pose_msg.header.stamp = rospy.Time.now()
                position_publisher.publish(self.pose_msg)
                self.gripper_msg.header = self.pose_msg.header
                self.gripper_msg.point.x = abs(menu_button)
                self.gripper_msg.point.y = 0
                self.gripper_msg.point.z = 0
                gripper_publisher.publish(self.gripper_msg)
                if self.publish_markers:
                    self.publish_axes_marker("ci/world", self.pose_msg.pose, marker_publisher)

        if side == "left" and self.use_left_controller:
            if self.left_initial_orientation is None:
                self.left_initial_orientation = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
            if gripper_button:
                self.left_initial_orientation = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
            if trigger_value < 0.5:
                if self.left_trigger_active:
                    self.left_trigger_active = False
                    self.left_cumulative_x += euler_pose[0] - self.left_reference_position[0]
                    self.left_cumulative_y += euler_pose[2] - self.left_reference_position[2]
                    self.left_cumulative_z += euler_pose[1] - self.left_reference_position[1]
            else:
                if not self.left_trigger_active:
                    self.left_trigger_active = True
                    self.left_reference_position = euler_pose[:3]
                if trackpad_touched and trackpad_pressed and self.move_base:
                    trackpad_x = controller_inputs.get('trackpad_x', 0)
                    trackpad_y = controller_inputs.get('trackpad_y', 0)
                    vel_x = trackpad_x * self.linear_scale
                    vel_y = trackpad_y * self.linear_scale
                    self.move_base_linear_x_publisher.publish(vel_x)
                    self.move_base_linear_y_publisher.publish(vel_y)
                dx = euler_pose[0] - self.left_reference_position[0]
                dy = euler_pose[2] - self.left_reference_position[2]
                dz = euler_pose[1] - self.left_reference_position[1]
                out_x = self.left_cumulative_x + dx
                out_y = self.left_cumulative_y + dy
                out_z = self.left_cumulative_z + dz
                filtered_pose_x = fx_filter(out_x, rospy.Time.now().to_sec())
                filtered_pose_y = fy_filter(out_y, rospy.Time.now().to_sec())
                filtered_pose_z = fz_filter(out_z, rospy.Time.now().to_sec())
                q0_inv = quaternion_inverse(self.left_initial_orientation)
                q_current = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
                q_rel = quaternion_multiply(q0_inv, q_current)
                q_rel = (q_rel[0], q_rel[1], -q_rel[2], q_rel[3])
                r_rel, p_rel, y_rel = euler_from_quaternion(q_rel)
                roll = math.degrees(r_rel)
                pitch = math.degrees(p_rel)
                yaw = math.degrees(y_rel)
                filtered_roll = fr_filter(roll, rospy.Time.now().to_sec())
                filtered_pitch = fp_filter(pitch, rospy.Time.now().to_sec())
                filtered_yaw = fw_filter(yaw, rospy.Time.now().to_sec())
                filtered_q = quaternion_from_euler(
                    math.radians(filtered_roll),
                    math.radians(filtered_pitch),
                    math.radians(filtered_yaw)
                )
                self.pose_msg.pose.position.x = filtered_pose_y
                self.pose_msg.pose.position.y = filtered_pose_x
                self.pose_msg.pose.position.z = filtered_pose_z
                self.pose_msg.pose.orientation.x = filtered_q[0]
                self.pose_msg.pose.orientation.y = filtered_q[1]
                self.pose_msg.pose.orientation.z = filtered_q[2]
                self.pose_msg.pose.orientation.w = filtered_q[3]
                self.pose_msg.header.frame_id = "ci/world"
                self.pose_msg.header.stamp = rospy.Time.now()
                position_publisher.publish(self.pose_msg)
                self.gripper_msg.header = self.pose_msg.header
                self.gripper_msg.point.x = abs(menu_button)
                self.gripper_msg.point.y = 0
                self.gripper_msg.point.z = 0
                gripper_publisher.publish(self.gripper_msg)
                if self.publish_markers:
                    self.publish_axes_marker("ci/world", self.pose_msg.pose, marker_publisher)

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.use_right_controller:
                self.parse_data_device(
                    self.v.devices[self.controller_name_right],
                    "right",
                    self.position_publisher_right,
                    self.gripper_publisher_right,
                    self.marker_publisher_right,
                    self.right_filter_x,
                    self.right_filter_y,
                    self.right_filter_z,
                    self.right_filter_roll,
                    self.right_filter_pitch,
                    self.right_filter_yaw
                )
            if self.use_left_controller:
                self.parse_data_device(
                    self.v.devices[self.controller_name_left],
                    "left",
                    self.position_publisher_left,
                    self.gripper_publisher_left,
                    self.marker_publisher_left,
                    self.left_filter_x,
                    self.left_filter_y,
                    self.left_filter_z,
                    self.left_filter_roll,
                    self.left_filter_pitch,
                    self.left_filter_yaw
                )
            if self.publish_markers:
                self.publish_workspace_bbox_marker()
            self.rate.sleep()

if __name__ == '__main__':
    JoystickNode()
