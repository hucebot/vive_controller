#!/usr/bin/env python3
"""
Joystick Node for HTC Vive + ROS 2
==================================

Updated: Gripper and Base are independent of Trigger.
SAFETY FIX: Base velocity is explicitly zeroed when trackpad is released.
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from transformations import quaternion_from_euler, quaternion_multiply
from OneEuroFilter import OneEuroFilter
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr

CONFIG_PATH = "/ros2_ws/src/ros2_vive_controller/config/config.yaml"

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # --- Load Config & VR (Standard Setup) ---
        self.cfg = self._read_yaml(CONFIG_PATH)
        self.gen_cfg = self.cfg['general']
        self.v = triad_openvr(CONFIG_PATH)

        # Tracking Reference Logic
        try:
            master = self.cfg['htc_vive']['tracking_reference']['serial']
            ref_cnt = sum(1 for k, v in self.cfg['htc_vive'].items()
                          if isinstance(v, dict) and v.get('type') == 'tracking_reference')
        except KeyError:
            ref_cnt = 1
            master = 'LHB-DFA5BD2C'

        self.v.wait_for_n_tracking_references(ref_cnt)
        self.v.reorder_tracking_references(master)
        self.v.reindex_tracking_references()
        self.controllers = self.v.return_controller_serials()

        # --- Settings ---
        self.robot_name = self.gen_cfg['robot_name']
        self.publish_markers = self.gen_cfg['publish_markers']
        self.use_left = self.gen_cfg.get('use_left_controller', True)
        self.use_right = self.gen_cfg.get('use_right_controller', True)
        self.linear_scale = self.gen_cfg['linear_scale']
        self.angular_scale = self.gen_cfg['angular_scale']
        self.move_base = self.gen_cfg['move_base']
        self.link_name = self.gen_cfg['link_name']

        # --- Offsets ---
        robot_offsets = self.cfg['offset'].get(self.robot_name, {})
        if 'left' in robot_offsets:
            self.offset_right = robot_offsets['right']
            self.offset_left = robot_offsets['left']
        else:
            self.offset_right = robot_offsets
            self.offset_left = robot_offsets

        # --- ROS Setup ---
        self._init_subscribers()
        self._init_publishers()
        self._init_state_variables() # <--- Important changes here
        self._init_filters()

        rate = float(self.gen_cfg['rate'])
        self.create_timer(1.0 / rate, self.main_loop)
        self.get_logger().info(f"Joystick Node Ready. Base stops on release.")

    def _read_yaml(self, path):
        with open(path, 'r') as stream: return yaml.safe_load(stream)

    def _init_subscribers(self):
        self.reset_position_sub = self.create_subscription(
            Bool, self.gen_cfg['reset_position_topic'], self.reset_initial_state_cb, 10)

    def _init_publishers(self):
        self.push_to_talk_publisher = self.create_publisher(Bool, self.gen_cfg['push_to_talk_topic'], 10)

        if self.use_right:
            self.pub_right_pose = self.create_publisher(PoseStamped, self.gen_cfg['right_position_topic'], 10)
            self.pub_right_gripper = self.create_publisher(PointStamped, self.gen_cfg['right_gripper_topic'], 10)
            if self.publish_markers:
                self.pub_right_marker = self.create_publisher(Marker, self.gen_cfg['right_marker_topic'], 10)
                self.pub_right_aux_marker = self.create_publisher(Marker, '/vive/right/aux_marker', 10)

        if self.use_left:
            self.pub_left_pose = self.create_publisher(PoseStamped, self.gen_cfg['left_position_topic'], 10)
            self.pub_left_gripper = self.create_publisher(PointStamped, self.gen_cfg['left_gripper_topic'], 10)
            if self.publish_markers:
                self.pub_left_marker = self.create_publisher(Marker, self.gen_cfg['left_marker_topic'], 10)

        if self.move_base:
            self.pub_base_ang = self.create_publisher(Float32, self.gen_cfg['move_base_angular_topic'], 10)
            self.pub_base_lin_x = self.create_publisher(Float32, self.gen_cfg['move_base_linear_x_topic'], 10)
            self.pub_base_lin_y = self.create_publisher(Float32, self.gen_cfg['move_base_linear_y_topic'], 10)

        if self.publish_markers:
            self.pub_workspace_marker = self.create_publisher(Marker, "workspace_bbox_marker", 10)

    def _init_state_variables(self):
        self.right_pose_msg = PoseStamped()
        self.left_pose_msg = PoseStamped()

        # Workspace
        ws = self.cfg['workspace']
        self.ws_lims = {k: ws[k] for k in ws}
        self.ws_pad = self.gen_cfg['workspace_limit']

        # Right State
        if self.use_right:
            self.right_serial = self.cfg['htc_vive']['controller_right']['serial']
            self.right_name = self.controllers.get(self.right_serial)
            self.right_trigger_active = False
            self.right_ref_pos = None
            self.right_cum_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            self.prev_right_mic_state = False
            self.prev_right_tp_touch = False  # Track previous touch state

        # Left State
        if self.use_left:
            self.left_serial = self.cfg['htc_vive']['controller_left']['serial']
            self.left_name = self.controllers.get(self.left_serial)
            self.left_trigger_active = False
            self.left_ref_pos = None
            self.left_cum_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            self.prev_left_tp_touch = False   # Track previous touch state

    def _init_filters(self):
        cfg = self.cfg['one_euro_filter']
        f_cfg = {'freq': self.gen_cfg['rate'], 'mincutoff': cfg['mincutoff'], 'beta': cfg['beta'], 'dcutoff': cfg['dcutoff']}

        def create_set():
            return {
                'x': OneEuroFilter(**f_cfg), 'y': OneEuroFilter(**f_cfg), 'z': OneEuroFilter(**f_cfg),
                'qx': OneEuroFilter(**f_cfg), 'qy': OneEuroFilter(**f_cfg), 'qz': OneEuroFilter(**f_cfg), 'qw': OneEuroFilter(**f_cfg)
            }
        self.filters = {'right': create_set(), 'left': create_set()}

    def reset_initial_state_cb(self, msg):
        if msg.data and self.use_right:
            self.get_logger().info('Resetting to Home.')
            self.right_cum_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            self.right_pose_msg.pose.position.x = self.offset_right['x']
            self.right_pose_msg.pose.position.y = self.offset_right['y']
            self.right_pose_msg.pose.position.z = self.offset_right['z']
            self.right_pose_msg.pose.orientation.w = 1.0
            self.right_pose_msg.header.frame_id = self.link_name
            self.right_pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_right_pose.publish(self.right_pose_msg)

    def process_controller(self, device, side, pub_pose, pub_grip, pub_marker, offset):
        if not device: return

        q_pose = device.get_pose_quaternion()
        inputs = device.get_controller_inputs()
        if q_pose is None or inputs is None: return

        px, py, pz = q_pose[0:3]
        qx, qy, qz, qw = q_pose[3:7]

        # Safety Check
        if (px < self.ws_lims['x_min'] + self.ws_pad or px > self.ws_lims['x_max'] - self.ws_pad or
            py < self.ws_lims['y_min'] + self.ws_pad or py > self.ws_lims['y_max'] - self.ws_pad or
            pz < self.ws_lims['z_min'] + self.ws_pad or pz > self.ws_lims['z_max'] - self.ws_pad):
            device.trigger_haptic_pulse(1000, 0)
            return

        # Inputs
        trigger_val = inputs.get('trigger', 0)
        menu_btn = inputs.get('menu_button', 0)
        grip_btn = inputs.get('grip_button', 0)
        tp_touch = inputs.get('trackpad_touched', 0)
        tp_press = inputs.get('trackpad_pressed', 0)

        # State Handling
        is_right = (side == "right")
        trigger_active = self.right_trigger_active if is_right else self.left_trigger_active
        ref_pos = self.right_ref_pos if is_right else self.left_ref_pos
        cum_pos = self.right_cum_pos if is_right else self.left_cum_pos

        # Get previous touch state
        prev_tp_touch = self.prev_right_tp_touch if is_right else self.prev_left_tp_touch

        # ==========================================================
        #  INDEPENDENT CONTROLS
        # ==========================================================

        # 1. Microphone (Right Only)
        if is_right and grip_btn != self.prev_right_mic_state:
            self.push_to_talk_publisher.publish(Bool(data=bool(grip_btn)))
            self.prev_right_mic_state = grip_btn

        # 2. Base Movement (Trackpad) - SAFETY STOP ON RELEASE
        if self.move_base:
            if tp_touch and tp_press:
                # User is actively driving
                if is_right:
                    val = inputs.get('trackpad_x', 0) * self.angular_scale
                    self.pub_base_ang.publish(Float32(data=val))
                else:
                    val = inputs.get('trackpad_y', 0) * self.linear_scale
                    self.pub_base_lin_x.publish(Float32(data=val))

            elif prev_tp_touch and not tp_touch:
                # Falling Edge Detection: User JUST released the trackpad
                # Force publish ZERO to stop the robot
                if is_right:
                    self.pub_base_ang.publish(Float32(data=0.0))
                    self.get_logger().info("Right Trackpad Released -> Stopping Rotation")
                else:
                    self.pub_base_lin_x.publish(Float32(data=0.0))
                    self.get_logger().info("Left Trackpad Released -> Stopping Translation")

        # Update touch state for next loop
        if is_right: self.prev_right_tp_touch = tp_touch
        else: self.prev_left_tp_touch = tp_touch

        # 3. Gripper Control
        grip_msg = PointStamped()
        grip_msg.header.frame_id = self.link_name
        grip_msg.header.stamp = self.get_clock().now().to_msg()
        grip_msg.point.x = float(abs(1 - menu_btn))
        pub_grip.publish(grip_msg)

        # ==========================================================
        #  ARM CONTROL (Deadman Switch)
        # ==========================================================

        if trigger_val < 0.5:
            if trigger_active:
                cum_pos['x'] += px - ref_pos[0]
                cum_pos['y'] += py - ref_pos[1]
                cum_pos['z'] += pz - ref_pos[2]
                if is_right: self.right_trigger_active = False
                else: self.left_trigger_active = False
        else:
            if not trigger_active:
                ref_pos = [px, py, pz]
                if is_right:
                    self.right_trigger_active = True
                    self.right_ref_pos = ref_pos
                else:
                    self.left_trigger_active = True
                    self.left_ref_pos = ref_pos

            # Calculate Target
            dx, dy, dz = px - ref_pos[0], py - ref_pos[1], pz - ref_pos[2]
            target_x, target_y, target_z = cum_pos['x'] + dx, cum_pos['y'] + dy, cum_pos['z'] + dz

            # Filter
            t = self.get_clock().now().nanoseconds * 1e-9
            f = self.filters[side]
            fx, fy, fz = f['x'](target_x, t), f['y'](target_y, t), f['z'](target_z, t)

            # Publish Pose
            msg = PoseStamped()
            msg.header.frame_id = self.link_name
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = fx * self.linear_scale + offset['x']
            msg.pose.position.y = fy * self.linear_scale + offset['y']
            msg.pose.position.z = fz * self.linear_scale + offset['z']
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw

            if is_right: self.right_pose_msg = msg
            else: self.left_pose_msg = msg

            pub_pose.publish(msg)

            if self.publish_markers:
                self.publish_axes_marker(self.link_name, msg.pose, pub_marker)

    def main_loop(self):
        if self.use_right and self.right_name:
            self.process_controller(
                self.v.devices.get(self.right_name), "right",
                self.pub_right_pose, self.pub_right_gripper, self.pub_right_marker, self.offset_right
            )
        if self.use_left and self.left_name:
            self.process_controller(
                self.v.devices.get(self.left_name), "left",
                self.pub_left_pose, self.pub_left_gripper, self.pub_left_marker, self.offset_left
            )
        if self.publish_markers:
            self.publish_workspace_bbox_marker()

    def publish_axes_marker(self, frame_id, pose, publisher):
        q_x = quaternion_from_euler(0, 0, 0)
        q_y = quaternion_from_euler(0, 0, math.pi / 2)
        q_z = quaternion_from_euler(0, -math.pi / 2, 0)

        def make_arrow(id, rgb, q_offset):
            m = Marker(type=Marker.ARROW, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = frame_id, self.get_clock().now().to_msg()
            m.ns, m.id = "axes", id
            m.scale.x, m.scale.y, m.scale.z = 0.2, 0.015, 0.015
            m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
            m.pose.position = pose.position
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quaternion_multiply(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], q_offset)
            return m

        publisher.publish(make_arrow(0, (1.0, 0.0, 0.0), q_x))
        publisher.publish(make_arrow(1, (0.0, 1.0, 0.0), q_y))
        publisher.publish(make_arrow(2, (0.0, 0.0, 1.0), q_z))

    def publish_workspace_bbox_marker(self):
        m = Marker(type=Marker.CUBE, action=Marker.ADD)
        m.header.frame_id, m.header.stamp = self.link_name, self.get_clock().now().to_msg()
        m.ns, m.id = "workspace", 1000
        m.scale.x = self.ws_lims['x_max'] - self.ws_lims['x_min']
        m.scale.y = self.ws_lims['y_max'] - self.ws_lims['y_min']
        m.scale.z = self.ws_lims['z_max'] - self.ws_lims['z_min']
        m.pose.position.x = (self.ws_lims['x_max'] + self.ws_lims['x_min']) / 2.0
        m.pose.position.y = (self.ws_lims['y_max'] + self.ws_lims['y_min']) / 2.0
        m.pose.position.z = (self.ws_lims['z_max'] + self.ws_lims['z_min']) / 2.0
        m.pose.orientation.w = 1.0
        m.color.g, m.color.a = 1.0, 0.1
        self.pub_workspace_marker.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()