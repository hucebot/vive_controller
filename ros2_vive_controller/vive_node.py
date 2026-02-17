#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from OneEuroFilter import OneEuroFilter
# Ensure your import matches your folder structure
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr

class ViveDriverNode(Node):
    def __init__(self):
        super().__init__('vive_driver')

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('side', 'right'),
                ('serial', ''),
                ('frame_id', 'vive_world'),
                # Workspace Limits (meters)
                ('workspace.x_min', -1.0), ('workspace.x_max', 1.0),
                ('workspace.y_min', -1.0), ('workspace.y_max', 1.0),
                ('workspace.z_min', 0.0),  ('workspace.z_max', 2.0),
                ('workspace.padding', 0.1), # Vibration buffer zone
                # Smoothing
                ('filter.mincutoff', 1.0),
                ('filter.beta', 0.007),
                ('filter.dcutoff', 1.0)
            ]
        )

        self.side = self.get_parameter('side').value
        self.serial = self.get_parameter('serial').value
        self.frame_id = self.get_parameter('frame_id').value

        # Load Workspace Cache
        self.ws = {
            'x_min': self.get_parameter('workspace.x_min').value,
            'x_max': self.get_parameter('workspace.x_max').value,
            'y_min': self.get_parameter('workspace.y_min').value,
            'y_max': self.get_parameter('workspace.y_max').value,
            'z_min': self.get_parameter('workspace.z_min').value,
            'z_max': self.get_parameter('workspace.z_max').value,
            'pad': self.get_parameter('workspace.padding').value
        }

        # --- VR Setup ---
        self.vr = triad_openvr()
        self.device_name = self._find_device_by_serial(self.serial)

        if not self.device_name:
            self.get_logger().error(f"Could not find controller {self.serial}")
            # We don't crash, allowing potential re-connection logic later
        else:
            self.get_logger().info(f"Connected to {self.side} controller: {self.serial}")

        # --- Publishers ---
        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.pub_joy = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_marker = self.create_publisher(Marker, 'workspace_marker', 10)

        # --- Filters ---
        self.filters = self._init_filters()

        # --- Loop ---
        self.create_timer(0.02, self.update) # 50Hz

        # Publish marker once at startup (and periodically in loop)
        self.publish_workspace_marker()

    def _find_device_by_serial(self, target_serial):
        for key, dev in self.vr.devices.items():
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
            return False # Unsafe

        return True # Safe

    def update(self):
        if not self.device_name: return

        controller = self.vr.devices[self.device_name]
        pose = controller.get_pose_quaternion()
        inputs = controller.get_controller_inputs()

        if not pose or not inputs: return

        # --- 1. SAFETY CHECK (The Gatekeeper) ---
        raw_pos = pose[0:3]
        is_safe = self.check_workspace_and_vibrate(raw_pos, controller)
        # TODO: pulish or not publish this in the future based on a param (if calibrating or not)
        # IF UNSAFE: STOP HERE. DO NOT PUBLISH.
        # if not is_safe:
        #     return

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
        msg_joy.name = ['trigger', 'trackpad_x', 'trackpad_y', 'grip', 'menu', 'trackpad_touched', 'trackpad_pressed']
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
        if inputs.get('trackpad_touched', False): # Only update marker when interacting to save bandwidth
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
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.15 # Red Transparent
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