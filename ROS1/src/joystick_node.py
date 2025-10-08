#!/usr/bin/env python3
from openvr_class.openvr_class import triad_openvr
import rospy, math, yaml
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, quaternion_multiply
from OneEuroFilter import OneEuroFilter

import tf

def read_yaml(path):
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)

class JoystickNode:
    def __init__(self):
        rospy.init_node('joystick_node', anonymous=True)
        self.config_file = "/ros_ws/src/ros1_vive_controller/config/config.yaml"
        self.configurations = read_yaml(self.config_file)
        self.robot_name = self.configurations['general']['robot']
        self.v = triad_openvr(self.config_file)
        self.v.wait_for_n_tracking_references(1)
        self.v.reorder_tracking_references('LHB-DFA5BD2C')
        self.v.reindex_tracking_references()
        self.v.print_discovered_objects()

        self.prev_right_mic_state = False

        self.controllers = self.v.return_controller_serials()
        self.publish_markers = self.configurations['general']['publish_markers']
        self.use_left_controller = self.configurations['general']['use_left_controller']
        self.use_right_controller = self.configurations['general']['use_right_controller']
        self.rate = rospy.Rate(self.configurations['general']['rate'])
        self.linear_scale = self.configurations['general']['linear_scale']
        self.angular_scale = self.configurations['general']['angular_scale']
        self.move_base = self.configurations['general']['move_base']

        rospy.Subscriber(self.configurations['general']['reset_position_topic'], Bool, self.reset_initial_state_cb)

        self.tf_br = tf.TransformBroadcaster()

        self.initial_offset = {
            'x': self.configurations['offset'][self.robot_name]['x'],
            'y': self.configurations['offset'][self.robot_name]['y'],
            'z': self.configurations['offset'][self.robot_name]['z']
        }


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
            if self.robot_name == 'talos':
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
                self.auxiliar_marker_publisher_right = rospy.Publisher(
                    '/auxiliar_marker_right',
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

        self.enable_microphone_publisher = rospy.Publisher(
            self.configurations['general']['enable_microphone_topic'],
            Bool,
            queue_size=10
        )

        self.right_pose_msg = PoseStamped()
        self.left_pose_msg = PoseStamped()
        self.right_gripper_msg = PointStamped()
        self.left_gripper_msg = PointStamped()
        self.workspace_limit = self.configurations['general']['workspace_limit']
        self.x_max = self.configurations['workspace']['x_max']
        self.x_min = self.configurations['workspace']['x_min']
        self.y_max = self.configurations['workspace']['y_max']
        self.y_min = self.configurations['workspace']['y_min']
        self.z_max = self.configurations['workspace']['z_max']
        self.z_min = self.configurations['workspace']['z_min']
        if self.publish_markers:
            self.workspace_marker_pub = rospy.Publisher("workspace_bbox_marker", Marker, queue_size=10)

        cfg = {
            'freq': self.configurations['general']['rate'],
            'mincutoff': 1.0,
            'beta': 0.1,
            'dcutoff': 1.0
        }
        self.right_filter_x = OneEuroFilter(**cfg)
        self.right_filter_y = OneEuroFilter(**cfg)
        self.right_filter_z = OneEuroFilter(**cfg)
        self.right_filter_qx = OneEuroFilter(**cfg)
        self.right_filter_qy = OneEuroFilter(**cfg)
        self.right_filter_qz = OneEuroFilter(**cfg)
        self.right_filter_qw = OneEuroFilter(**cfg)

        self.left_filter_x = OneEuroFilter(**cfg)
        self.left_filter_y = OneEuroFilter(**cfg)
        self.left_filter_z = OneEuroFilter(**cfg)
        self.left_filter_qx = OneEuroFilter(**cfg)
        self.left_filter_qy = OneEuroFilter(**cfg)
        self.left_filter_qz = OneEuroFilter(**cfg)
        self.left_filter_qw = OneEuroFilter(**cfg)


        self.main_loop()

    def reset_initial_state_cb(self, msg):
        if msg.data:
            rospy.loginfo('reseting controller state')
            self.right_cumulative_x = 0.0 
            self.right_cumulative_y = 0.0
            self.right_cumulative_z = 0.0

            self.right_pose_msg.pose.position.x = 0.0 + self.initial_offset['x']
            self.right_pose_msg.pose.position.y = 0.0 + self.initial_offset['y']
            self.right_pose_msg.pose.position.z = 0.0 + self.initial_offset['z']
            self.right_pose_msg.pose.orientation.x = 0.0
            self.right_pose_msg.pose.orientation.y = 0.0
            self.right_pose_msg.pose.orientation.z = 0.0
            self.right_pose_msg.pose.orientation.w = 1.0
            self.right_pose_msg.header.frame_id = "ci/world"
            self.right_pose_msg.header.stamp = rospy.Time.now()
            self.position_publisher_right.publish(self.right_pose_msg)


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
        q_id = quaternion_from_euler(0, 0, 0)
        q_y = quaternion_from_euler(0, 0, math.pi/2)
        q_z = quaternion_from_euler(0, -math.pi/2, 0)
        marker_x = make_arrow_marker(0, (1.0, 0.0, 0.0), q_id)
        marker_y = make_arrow_marker(1, (0.0, 1.0, 0.0), q_y)
        marker_z = make_arrow_marker(2, (0.0, 0.0, 1.0), q_z)
        marker_publisher.publish(marker_x)
        marker_publisher.publish(marker_y)
        marker_publisher.publish(marker_z)

        arrow_maker = Marker()
        arrow_maker.header.frame_id = frame_id
        arrow_maker.header.stamp = rospy.Time.now()
        arrow_maker.ns = "auxiliar"
        arrow_maker.id = 0
        arrow_maker.type = Marker.ARROW
        arrow_maker.action = Marker.ADD
        arrow_maker.scale.x = 0.1
        arrow_maker.scale.y = 0.02
        arrow_maker.scale.z = 0.02
        arrow_maker.color.a = 1.0
        arrow_maker.color.r = 1.0
        arrow_maker.color.g = 1.0
        arrow_maker.color.b = 1.0
        arrow_maker.pose.position.x = pose.position.x
        arrow_maker.pose.position.y = pose.position.y
        arrow_maker.pose.position.z = pose.position.z
        arrow_maker.pose.orientation.x = pose.orientation.x
        arrow_maker.pose.orientation.y = pose.orientation.y
        arrow_maker.pose.orientation.z = pose.orientation.z
        arrow_maker.pose.orientation.w = pose.orientation.w
        self.auxiliar_marker_publisher_right.publish(arrow_maker)

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
                          fx_filter, fy_filter, fz_filter, qx_filter, qy_filter, qz_filter, qw_filter):
        quaternion_pose = device.get_pose_quaternion()
        controller_inputs = device.get_controller_inputs()

        if quaternion_pose is None or controller_inputs is None:
            rospy.logwarn("No data received from device. Move the controller to receive data.")
            return
        
        px, py, pz = quaternion_pose[0:3]
        qx, qy, qz, qw = quaternion_pose[3:7]

        if (px < self.x_min + self.workspace_limit or px > self.x_max - self.workspace_limit or
            py < self.y_min + self.workspace_limit or py > self.y_max - self.workspace_limit or
            pz < self.z_min + self.workspace_limit or pz > self.z_max - self.workspace_limit):
            device.trigger_haptic_pulse(1000, 0)
            return
        
        trigger_value = controller_inputs.get('trigger', 0)
        trackpad_pressed = controller_inputs.get('trackpad_pressed', 0)
        trackpad_touched = controller_inputs.get('trackpad_touched', 0)
        menu_button = controller_inputs.get('menu_button', 0)
        gripper_button = controller_inputs.get('grip_button', 0)

        if side == "right" and self.use_right_controller:
            if self.right_initial_orientation is None:
                self.right_initial_orientation = [qx, qy, qz, qw]
            current_mic_state = gripper_button
            if current_mic_state != self.prev_right_mic_state:
                self.enable_microphone_publisher.publish(Bool(current_mic_state))
                self.prev_right_mic_state = current_mic_state
            if trigger_value < 0.5:
                if self.right_trigger_active:
                    self.right_trigger_active = False
                    self.right_cumulative_x += px - self.right_reference_position[0]
                    self.right_cumulative_y += py - self.right_reference_position[1]
                    self.right_cumulative_z += pz - self.right_reference_position[2]
            else:
                if not self.right_trigger_active:
                    self.right_trigger_active = True
                    self.right_reference_position = [px, py, pz]
                if trackpad_touched and trackpad_pressed and self.move_base:
                    trackpad_x = controller_inputs.get('trackpad_x', 0)
                    vel_ang = trackpad_x * self.angular_scale
                    self.move_base_angular_publisher.publish(vel_ang)
                dx = px - self.right_reference_position[0]
                dy = py - self.right_reference_position[1]
                dz = pz - self.right_reference_position[2]
                out_x = self.right_cumulative_x + dx
                out_y = self.right_cumulative_y + dy
                out_z = self.right_cumulative_z + dz
                filtered_pose_x = fx_filter(out_x, rospy.Time.now().to_sec())
                filtered_pose_y = fy_filter(out_y, rospy.Time.now().to_sec())
                filtered_pose_z = fz_filter(out_z, rospy.Time.now().to_sec())
                # filtered_qx = qx_filter(qx, rospy.Time.now().to_sec())
                # filtered_qy = qy_filter(qy, rospy.Time.now().to_sec())
                # filtered_qz = qz_filter(qz, rospy.Time.now().to_sec())
                # filtered_qw = qw_filter(qw, rospy.Time.now().to_sec())

                self.right_pose_msg.pose.position.x = filtered_pose_x * self.linear_scale + self.initial_offset['x']
                self.right_pose_msg.pose.position.y = filtered_pose_y * self.linear_scale + self.initial_offset['y']
                self.right_pose_msg.pose.position.z = filtered_pose_z * self.linear_scale + self.initial_offset['z']
                self.right_pose_msg.pose.orientation.x = qx
                self.right_pose_msg.pose.orientation.y = qy
                self.right_pose_msg.pose.orientation.z = qz
                self.right_pose_msg.pose.orientation.w = qw
                self.right_pose_msg.header.frame_id = "ci/world"
                self.right_pose_msg.header.stamp = rospy.Time.now()
                position_publisher.publish(self.right_pose_msg)
                self.right_gripper_msg.header = self.right_pose_msg.header
                self.right_gripper_msg.point.x = abs(1 - menu_button)
                self.right_gripper_msg.point.y = 0
                self.right_gripper_msg.point.z = 0
                gripper_publisher.publish(self.right_gripper_msg)
                if self.publish_markers:
                    self.publish_axes_marker("ci/world", self.right_pose_msg.pose, marker_publisher)

        if side == "left" and self.use_left_controller:
            if self.left_initial_orientation is None:
                self.left_initial_orientation = [qx, qy, 
qz, qw]
            if trigger_value < 0.5:
                if self.left_trigger_active:
                    self.left_trigger_active = False
                    self.left_cumulative_x += px - self.left_reference_position[0]
                    self.left_cumulative_y += py - self.left_reference_position[1]
                    self.left_cumulative_z += pz - self.left_reference_position[2]
            else:
                if not self.left_trigger_active:
                    self.left_trigger_active = True
                    self.left_reference_position = [px, py, pz]
                if trackpad_touched and trackpad_pressed and self.move_base:
                    trackpad_y = controller_inputs.get('trackpad_y', 0)
                    vel_x = trackpad_y * self.linear_scale
                    self.move_base_linear_x_publisher.publish(vel_x)
                dx = px - self.left_reference_position[0]
                dy = py - self.left_reference_position[1]
                dz = pz - self.left_reference_position[2]
                out_x = self.left_cumulative_x + dx
                out_y = self.left_cumulative_y + dy
                out_z = self.left_cumulative_z + dz
                filtered_pose_x = fx_filter(out_x, rospy.Time.now().to_sec())
                filtered_pose_y = fy_filter(out_y, rospy.Time.now().to_sec())
                filtered_pose_z = fz_filter(out_z, rospy.Time.now().to_sec())
                # filtered_qx = qx_filter(qx, rospy.Time.now().to_sec())
                # filtered_qy = qy_filter(qy, rospy.Time.now().to_sec())
                # filtered_qz = qz_filter(qz, rospy.Time.now().to_sec())
                # filtered_qw = qw_filter(qw, rospy.Time.now().to_sec())

                self.left_pose_msg.pose.position.x = filtered_pose_x * self.linear_scale + self.initial_offset['x']
                self.left_pose_msg.pose.position.y = filtered_pose_y * self.linear_scale + self.initial_offset['y']
                self.left_pose_msg.pose.position.z = filtered_pose_z * self.linear_scale + self.initial_offset['z']
                self.left_pose_msg.pose.orientation.x = qx
                self.left_pose_msg.pose.orientation.y = qy
                self.left_pose_msg.pose.orientation.z = qz
                self.left_pose_msg.pose.orientation.w = qw
                self.left_pose_msg.header.frame_id = "ci/world"
                self.left_pose_msg.header.stamp = rospy.Time.now()
                position_publisher.publish(self.left_pose_msg)

                self.left_gripper_msg.header = self.left_pose_msg.header
                self.left_gripper_msg.point.x = abs(1 - menu_button)
                self.left_gripper_msg.point.y = 0
                self.left_gripper_msg.point.z = 0
                gripper_publisher.publish(self.left_gripper_msg)
                if self.publish_markers:
                    self.publish_axes_marker("ci/world", self.left_pose_msg.pose, marker_publisher)

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
                    self.right_filter_qx,
                    self.right_filter_qy,
                    self.right_filter_qz,
                    self.right_filter_qw
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
                    self.left_filter_qx,
                    self.left_filter_qy,
                    self.left_filter_qz,
                    self.left_filter_qw
                )
            if self.publish_markers:
                self.publish_workspace_bbox_marker()
            self.rate.sleep()

if __name__ == '__main__':
    JoystickNode()
