#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from OneEuroFilter import OneEuroFilter
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr


class ViveDriverNode(Node):
    def __init__(self):
        # 1. Initialize with "Override" priority
        super().__init__(
            'vive_driver',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # 2. Extract Values
        # Note: We use .value directly because automatically_declare handles the setup
        self.side = self.get_parameter('side').value

        # Pull Serials
        self.serial_l = self.get_parameter('htc_vive.serial_left').value
        self.serial_r = self.get_parameter('htc_vive.serial_right').value
        self.ref_serial = self.get_parameter(
            'htc_vive.tracking_reference').value

        # 3. Logic to pick target
        if self.side == 'left':
            self.target_serial = self.serial_l
        else:
            self.target_serial = self.serial_r

        # --- CRITICAL LOGGING ---
        # If these are empty here, the Launch file dictionary isn't reaching the node
        self.get_logger().info(f"--- NODE SIDE: {self.side} ---")
        self.get_logger().info(f"Target Serial: '{self.target_serial}'")
        self.get_logger().info(f"Ref Serial:    '{self.ref_serial}'")

        # 4. Construct Config for OpenVR
        vr_config = {
            "devices": [
                {"serial": self.target_serial,
                    "name": "my_controller", "type": "Controller"}
            ]
        }
        if self.ref_serial:
            vr_config["devices"].append({
                "serial": self.ref_serial, "name": "base_station", "type": "Tracking Reference"
            })

        # 5. Init VR
        try:
            # Check your triad_openvr __init__ signature!
            # If it's (self, config_dict=None), use:
            self.vr = triad_openvr(vr_config)
        except Exception as e:
            self.get_logger().error(f"VR Init Error: {e}")
            return
        self.vr.wait_for_n_tracking_references(1)
        self.vr.reorder_tracking_references(ref_serial)
        self.vr.reindex_tracking_references()
        self.controllers = self.vr.return_controller_serials()

        # Check if OUR specific controller was found
        if "my_controller" in self.vr.devices:
            self.device_name = "my_controller"
            self.get_logger().info(
                f"✅ Connected to {self.side} ({self.target_serial})")
        else:
            self.device_name = None
            self.get_logger().error(
                f"❌ Controller {self.target_serial} NOT FOUND")

        # --- 5. Workspace & Publishers ---
        self.ws = {
            'x_min': self.get_parameter('workspace.x_min').value,
            'x_max': self.get_parameter('workspace.x_max').value,
            'y_min': self.get_parameter('workspace.y_min').value,
            'y_max': self.get_parameter('workspace.y_max').value,
            'z_min': self.get_parameter('workspace.z_min').value,
            'z_max': self.get_parameter('workspace.z_max').value,
            'pad': self.get_parameter('workspace.padding').value
        }

        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.pub_joy = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_marker = self.create_publisher(Marker, 'workspace_marker', 10)
        self.filters = self._init_filters()

        self.create_timer(0.02, self.update)
        self.publish_workspace_marker()

    def _find_device_by_serial(self, target_serial):
        for key, dev in self.vr.devices.items():
            self.get_logger().info(
                f"Checking device {key} with serial {dev.get_serial()}")
            if dev.get_serial() == target_serial:
                return key
        return None

    def _init_filters(self):
        params = {
            'freq': 50.0,
            'mincutoff': self.get_parameter('filter.mincutoff').value,
            'beta': self.get_parameter('filter.beta').value,
            'dcutoff': self.get_parameter('filter.dcutoff').value
        }
        return {axis: OneEuroFilter(**params) for axis in ['x', 'y', 'z']}

    def check_workspace_and_vibrate(self, pos, device):
        """
        Checks if position is within bounds.
        If UNSAFE: Vibrates controller and returns False.
        If SAFE: Returns True.
        """
        x, y, z = pos
        pad = self.ws['pad']

        in_bounds = (
            self.ws['x_min'] + pad < x < self.ws['x_max'] - pad and
            self.ws['y_min'] + pad < y < self.ws['y_max'] - pad and
            self.ws['z_min'] + pad < z < self.ws['z_max'] - pad
        )

        if not in_bounds:
            # Vibrate the controller (Haptic Feedback) - 2ms pulse
            device.trigger_haptic_pulse(2000)
            return False  # Unsafe

        return True  # Safe

    def update(self):
        if not self.device_name:
            return

        controller = self.vr.devices[self.device_name]
        pose = controller.get_pose_quaternion()
        inputs = controller.get_controller_inputs()

        if not pose or not inputs:
            return

        # --- 1. SAFETY CHECK (The Gatekeeper) ---
        raw_pos = pose[0:3]
        is_safe = self.check_workspace_and_vibrate(raw_pos, controller)

        # IF UNSAFE: STOP HERE. DO NOT PUBLISH.
        if not is_safe:
            return

        # --- 2. Publish Data (Only if Safe) ---
        timestamp = self.get_clock().now().to_msg()
        t = self.get_clock().now().nanoseconds / 1e9

        # A. Pose
        msg_pose = PoseStamped()
        msg_pose.header.stamp = timestamp
        msg_pose.header.frame_id = self.frame_id

        msg_pose.pose.position.x = self.filters['x'](pose[0], t)
        msg_pose.pose.position.y = self.filters['y'](pose[1], t)
        msg_pose.pose.position.z = self.filters['z'](pose[2], t)
        msg_pose.pose.orientation.x = pose[3]
        msg_pose.pose.orientation.y = pose[4]
        msg_pose.pose.orientation.z = pose[5]
        msg_pose.pose.orientation.w = pose[6]

        self.pub_pose.publish(msg_pose)

        # B. Buttons
        msg_joy = JointState()
        msg_joy.header.stamp = timestamp
        msg_joy.header.frame_id = self.frame_id
        msg_joy.name = ['trigger', 'trackpad_x', 'trackpad_y',
                        'grip', 'menu', 'trackpad_touched', 'trackpad_pressed']
        msg_joy.position = [
            float(inputs.get('trigger', 0.0)),
            float(inputs.get('trackpad_x', 0.0)),
            float(inputs.get('trackpad_y', 0.0)),
            float(inputs.get('grip_button', False)),
            float(inputs.get('menu_button', False)),
            float(inputs.get('trackpad_touched', False)),
            float(inputs.get('trackpad_pressed', False))
        ]
        self.pub_joy.publish(msg_joy)

        # C. Visualization
        # We publish this occasionally so RViz always shows the red box
        # Only update marker when interacting to save bandwidth
        if inputs.get('trackpad_touched', False):
            self.publish_workspace_marker()

    def publish_workspace_marker(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workspace_limit"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = self.ws['x_max'] - self.ws['x_min']
        m.scale.y = self.ws['y_max'] - self.ws['y_min']
        m.scale.z = self.ws['z_max'] - self.ws['z_min']
        m.pose.position.x = (self.ws['x_max'] + self.ws['x_min']) / 2.0
        m.pose.position.y = (self.ws['y_max'] + self.ws['y_min']) / 2.0
        m.pose.position.z = (self.ws['z_max'] + self.ws['z_min']) / 2.0
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.15  # Red Transparent
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = ViveDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
