#!/usr/bin/env python3

from openvr_class import triad_openvr, time
import rospy, math, yaml
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse, euler_from_quaternion

def read_yaml(path):
    with open(path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

class KalmanFilter:
    def __init__(self, dt, state_dim, measurement_dim):
        self.dt = dt
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        self.A = np.eye(state_dim)
        self.H = np.eye(measurement_dim, state_dim)
        self.Q = np.eye(state_dim) * 0.01
        self.R = np.eye(measurement_dim) * 0.1
        self.P = np.eye(state_dim)
        self.x = np.zeros((state_dim, 1))

    def predict(self):
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)
        return self.x

class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)

        self.config_file = "/ros_ws/src/ros1_vive_controller/config/config.yaml"
        self.configurations = read_yaml(self.config_file)

        self.controllers = []
        self.v = triad_openvr(self.config_file)
        self.v.print_discovered_objects()
        self.controllers = self.v.return_controller_serials()

        self.publish_markers = self.configurations['general']['publish_markers']
        self.space_scale = self.configurations['general']['space_scale']
        self.use_left_controller = self.configurations['general']['use_left_controller']
        self.use_right_controller = self.configurations['general']['use_right_controller']
        self.rate = rospy.Rate(self.configurations['general']['rate'])
        self.linear_scale = self.configurations['general']['linear_scale']
        self.angular_scale = self.configurations['general']['angular_scale']
        self.move_base = self.configurations['general']['move_base']

        if self.use_right_controller:
            self.right_serial = self.configurations['htc_vive']['controller_1']['serial']
            self.position_publisher_right = rospy.Publisher(self.configurations['general']['right_position_topic'], PoseStamped, queue_size=10)
            self.gripper_publisher_right = rospy.Publisher(self.configurations['general']['right_gripper_topic'], PointStamped, queue_size=10)
            self.controller_name_right = self.controllers[self.right_serial]
            self.right_initial_orientation = None
            self.right_initial_position = None
            if self.publish_markers:
                self.marker_publisher_right = rospy.Publisher(self.configurations['general']['right_marker_topic'], Marker, queue_size=10)
            if self.move_base:
                self.move_base_angular_publisher = rospy.Publisher(self.configurations['general']['move_base_angular_topic'], Float32, queue_size=10)

        if self.use_left_controller:
            self.left_serial = self.configurations['htc_vive']['controller_2']['serial']
            self.position_publisher_left = rospy.Publisher(self.configurations['general']['left_position_topic'], PoseStamped, queue_size=10)
            self.gripper_publisher_left = rospy.Publisher(self.configurations['general']['left_gripper_topic'], PointStamped, queue_size=10)
            self.controller_name_left = self.controllers[self.left_serial]
            self.left_initial_orientation = None
            self.left_initial_position = None
            if self.publish_markers:
                self.marker_publisher_left = rospy.Publisher(self.configurations['general']['left_marker_topic'], Marker, queue_size=10)
            if self.move_base:
                self.move_base_linear_x_publisher = rospy.Publisher(self.configurations['general']['move_base_linear_x_topic'], Float32, queue_size=10)
                self.move_base_linear_y_publisher = rospy.Publisher(self.configurations['general']['move_base_linear_y_topic'], Float32, queue_size=10)

        self.pose_msg = PoseStamped()
        self.gripper_msg = PointStamped()

        self.dt = 1.0 / self.configurations['kalman_filter']['frequency']
        self.state_dim = self.configurations['kalman_filter']['state_dim']
        self.measurement_dim = self.configurations['kalman_filter']['measurement_dim']
        self.right_kf = KalmanFilter(self.dt, self.state_dim, self.measurement_dim)
        self.left_kf = KalmanFilter(self.dt, self.state_dim, self.measurement_dim)

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
        self.main_loop()

    def publish_axes_marker(self, frame_id, pose, marker_publisher, namespace="joystick_axes"):
        axis_length = 0.2
        axis_diameter = 0.015

        def make_arrow_marker(id, color, orientation_offset):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time.now()
            m.ns = namespace
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
            q_offset = orientation_offset
            q_final = quaternion_multiply(q_pose, q_offset)
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

    def parse_data_device(self, device, side, position_publisher, gripper_publisher, marker_publisher, kf):
        euler_pose = device.get_pose_euler()
        controller_inputs = device.get_controller_inputs()

        if euler_pose is not None:
            pose_marker_msg = Marker()
            pose_marker_msg.header.frame_id = "ci/world"
            pose_marker_msg.header.stamp = rospy.Time.now()
            pose_marker_msg.ns = "actual_pose"
            pose_marker_msg.id = 0
            pose_marker_msg.type = Marker.SPHERE
            pose_marker_msg.action = Marker.ADD
            pose_marker_msg.scale.x = 0.1
            pose_marker_msg.scale.y = 0.1
            pose_marker_msg.scale.z = 0.1
            pose_marker_msg.color.r = 1.0
            pose_marker_msg.color.g = 0.0
            pose_marker_msg.color.b = 0.0
            pose_marker_msg.color.a = 1.0
            pose_marker_msg.pose.position.x = euler_pose[2]
            pose_marker_msg.pose.position.y = euler_pose[0]
            pose_marker_msg.pose.position.z = euler_pose[1]
            pose_marker_msg.pose.orientation.x = 0
            pose_marker_msg.pose.orientation.y = 0
            pose_marker_msg.pose.orientation.z = 0
            pose_marker_msg.pose.orientation.w = 1
            self.actual_pose_marker_pub.publish(pose_marker_msg)

        if euler_pose is None:
            device.trigger_haptic_pulse(1000, 0)
            rospy.logwarn(f"Failed to get pose for {side} controller")
            # self.pose_msg.header.frame_id = "ci/world"
            # self.pose_msg.header.stamp = rospy.Time.now()
            # self.pose_msg.pose.position.x = 0
            # self.pose_msg.pose.position.y = 0
            # self.pose_msg.pose.position.z = 0
            # self.pose_msg.pose.orientation.x = 0
            # self.pose_msg.pose.orientation.y = 0
            # self.pose_msg.pose.orientation.z = 0
            # self.pose_msg.pose.orientation.w = 1
            # position_publisher.publish(self.pose_msg)
            return
        
        if (euler_pose[2] < self.x_min + self.workspace_limit or
            euler_pose[2] > self.x_max - self.workspace_limit or
            euler_pose[0] < self.y_min + self.workspace_limit or
            euler_pose[0] > self.y_max - self.workspace_limit or
            euler_pose[1] < self.z_min + self.workspace_limit or
            euler_pose[1] > self.z_max - self.workspace_limit):
            rospy.logwarn(f"Controller {side} out of workspace")
            device.trigger_haptic_pulse(1000, 0)
            return

        if controller_inputs is not None:
            trigger_value = controller_inputs.get('trigger', 0)
            trackpad_pressed = controller_inputs.get('trackpad_pressed', 0)
            trackpad_touched = controller_inputs.get('trackpad_touched', 0)
            menu_button = controller_inputs.get('menu_button', 0)
            gripper_button = controller_inputs.get('grip_button', 0)


            if gripper_button:
                rospy.loginfo(f'Resetting initial position for {side} controller')
                if side == "right" and self.use_right_controller:
                    self.right_initial_position = euler_pose[:3]
                    self.right_initial_orientation = quaternion_from_euler(
                        math.radians(euler_pose[3]),
                        math.radians(euler_pose[5]),
                        math.radians(euler_pose[4])
                    )
                if side == "left" and self.use_left_controller:
                    self.left_initial_position = euler_pose[:3]
                    self.left_initial_orientation = quaternion_from_euler(
                        math.radians(euler_pose[3]),
                        math.radians(euler_pose[5]),
                        math.radians(euler_pose[4])
                    )

            if self.use_right_controller:
                if self.right_initial_position is None and side == "right":
                    self.right_initial_position = euler_pose[:3]
                    self.right_initial_orientation = quaternion_from_euler(
                        math.radians(euler_pose[3]),
                        math.radians(euler_pose[5]),
                        math.radians(euler_pose[4])
                    )

            if self.use_left_controller:
                if self.left_initial_position is None and side == "left":
                    self.left_initial_position = euler_pose[:3]
                    self.left_initial_orientation = quaternion_from_euler(
                        math.radians(euler_pose[3]),
                        math.radians(euler_pose[5]),
                        math.radians(euler_pose[4])
                    )

            if trackpad_touched and trackpad_pressed:
                trackpad_x = controller_inputs.get('trackpad_x', 0)
                trackpad_y = controller_inputs.get('trackpad_y', 0)
                
                if side == "right" and self.move_base:
                    vel_ang = trackpad_x * self.angular_scale
                    rospy.loginfo(f"Angular velocity: {vel_ang}")
                    self.move_base_angular_publisher.publish(vel_ang)

                if side == "left" and self.move_base:
                    vel_x = trackpad_x * self.linear_scale
                    vel_y = trackpad_y * self.linear_scale
                    rospy.loginfo(f"Linear velocity: {vel_x}, {vel_y}")

            if trigger_value >= 0.5:
                if side == "right" and self.use_right_controller:
                    initial_position = self.right_initial_position
                    initial_orientation = self.right_initial_orientation
                if side == "left" and self.use_left_controller:
                    initial_position = self.left_initial_position
                    initial_orientation = self.left_initial_orientation

                pose_x = -round(euler_pose[0] - initial_position[0], 2)
                pose_y = round(euler_pose[2] - initial_position[2], 2)
                pose_z = round(euler_pose[1] - initial_position[1], 2)

                q_current = quaternion_from_euler(
                    math.radians(euler_pose[3]),
                    math.radians(euler_pose[5]),
                    math.radians(euler_pose[4])
                )
                q0_inv = quaternion_inverse(initial_orientation)
                q_rel = quaternion_multiply(q0_inv, q_current)
                q_rel = (q_rel[0], q_rel[1], -q_rel[2], q_rel[3])
                r_rel, p_rel, y_rel = euler_from_quaternion(q_rel)
                roll = math.degrees(r_rel)
                pitch = math.degrees(p_rel)
                yaw = math.degrees(y_rel)

                measurement = np.array([[pose_x], [pose_y], [pose_z], [roll], [pitch], [yaw]])
                kf.predict()
                filtered_state = kf.update(measurement)

                # filtered_pose_x = pose_x
                # filtered_pose_y = pose_y
                # filtered_pose_z = pose_z
                # filtered_roll   = roll
                # filtered_pitch  = pitch
                # filtered_yaw    = yaw

                filtered_pose_x = filtered_state[0, 0]
                filtered_pose_y = filtered_state[1, 0]
                filtered_pose_z = filtered_state[2, 0]
                filtered_roll   = filtered_state[3, 0]
                filtered_pitch  = filtered_state[4, 0]
                filtered_yaw    = filtered_state[5, 0]

                filtered_q = quaternion_from_euler(
                    math.radians(filtered_roll),
                    math.radians(filtered_pitch),
                    math.radians(filtered_yaw)
                )

                self.pose_msg.pose.position.x = filtered_pose_y
                self.pose_msg.pose.position.y = -filtered_pose_x
                self.pose_msg.pose.position.z = filtered_pose_z

                self.pose_msg.pose.orientation.x = filtered_q[0]
                self.pose_msg.pose.orientation.y = filtered_q[1]
                self.pose_msg.pose.orientation.z = filtered_q[2]
                self.pose_msg.pose.orientation.w = filtered_q[3]

                self.pose_msg.header.frame_id = "ci/world"
                self.pose_msg.header.stamp = rospy.Time.now()

                position_publisher.publish(self.pose_msg)

                self.gripper_msg.header = self.pose_msg.header
                self.gripper_msg.point.x = abs(0.2 - menu_button)
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
                    self.right_kf
                )

            if self.use_left_controller:
                self.parse_data_device(
                    self.v.devices[self.controller_name_left],
                    "left",
                    self.position_publisher_left,
                    self.gripper_publisher_left,
                    self.marker_publisher_left,
                    self.left_kf
                )

            if self.publish_markers:
                self.publish_workspace_bbox_marker()

            self.rate.sleep()

if __name__ == '__main__':
    joystick_node = JoystickNode()
