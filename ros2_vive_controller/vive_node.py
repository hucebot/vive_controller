#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger # Import standard Trigger service
from OneEuroFilter import OneEuroFilter
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
                ('linear_scale', 1.0),
                ('workspace.x_min', -1.0), ('workspace.x_max', 1.0),
                ('workspace.y_min', -1.0), ('workspace.y_max', 1.0),
                ('workspace.z_min', 0.0),  ('workspace.z_max', 2.0),
                ('workspace.padding', 0.1),
                ('filter.mincutoff', 1.0),
                ('filter.beta', 0.007),
                ('filter.dcutoff', 1.0)
            ]
        )

        self.side = self.get_parameter('side').value
        self.serial = self.get_parameter('serial').value
        self.frame_id = self.get_parameter('frame_id').value
        self.linear_scale = self.get_parameter('linear_scale').value

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

        # Usage: ros2 service call /vive/right/identify std_srvs/srv/Trigger
        self.srv_identify = self.create_service(
            Trigger,
            'identify',
            self.identify_callback
        )

        if not self.device_name:
            self.get_logger().error(f"Could not find controller {self.serial}")
        else:
            self.get_logger().info(f"Connected to {self.side} controller: {self.serial}")

        # --- Publishers & State ---
        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.pub_joy = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_marker = self.create_publisher(Marker, 'workspace_marker', 10)

        self.is_calibrating = False
        self.last_heartbeat_time = 0.0
        self.create_subscription(Bool, '/vive/is_calibrating', self.calibration_callback, 10)
        self.create_timer(0.05, self.watchdog_check)

        self.filters = self._init_filters()
        self.tracking_active = False
        self.haptic_frames_remaining = 0

        # --- Loop ---
        self.create_timer(0.02, self.update)  # 50Hz
        self.publish_workspace_marker()

    # --- NEW: Service Callback ---
    def identify_callback(self, request, response):
        """Vibrates the controller on demand to identify which serial is which."""
        if not self.device_name:
            response.success = False
            response.message = "Controller not connected."
            return response

        self.get_logger().info(f"Identifying controller: {self.serial}")

        # Trigger a strong 1-second buzz (150 frames at 3900us)
        # We set the counter so the update loop handles it safely without blocking
        self.haptic_frames_remaining = 150

        response.success = True
        response.message = f"Vibrating {self.side} controller ({self.serial})"
        return response

    def _find_device_by_serial(self, target_serial):
        for key, dev in self.vr.devices.items():
            if dev.get_serial() == target_serial:
                return key
        return None

    def calibration_callback(self, msg):
        if msg.data:
            self.is_calibrating = True
            self.last_heartbeat_time = self.get_clock().now().nanoseconds / 1e9

    def watchdog_check(self):
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = 0.5
        if self.is_calibrating and (now - self.last_heartbeat_time > timeout):
            self.get_logger().warn("Calibration heartbeat lost! Re-enabling safety limits.")
            self.is_calibrating = False

    def _init_filters(self):
        params = {
            'freq': 50.0,
            'mincutoff': self.get_parameter('filter.mincutoff').value,
            'beta': self.get_parameter('filter.beta').value,
            'dcutoff': self.get_parameter('filter.dcutoff').value
        }
        return {axis: OneEuroFilter(**params) for axis in ['x', 'y', 'z']}

    def check_workspace_and_vibrate(self, pos, device):
        x, y, z = pos
        pad = self.ws['pad']
        in_bounds = (
            self.ws['x_min'] + pad < x < self.ws['x_max'] - pad and
            self.ws['y_min'] + pad < y < self.ws['y_max'] - pad and
            self.ws['z_min'] + pad < z < self.ws['z_max'] - pad
        )
        if not in_bounds:
            device.trigger_haptic_pulse(2000)
            return False
        return True

    def update(self):
        if not self.device_name:
            return

        controller = self.vr.devices[self.device_name]

        # --- HAPTIC VIBRATION HANDLER (Moved UP) ---
        # This now runs even if optical tracking is lost
        if self.haptic_frames_remaining > 0:
            controller.trigger_haptic_pulse(3900)
            time.sleep(0.005) # Optional: Double pulse for stronger feel
            controller.trigger_haptic_pulse(3900)
            self.haptic_frames_remaining -= 1

        pose = controller.get_pose_quaternion()
        inputs = controller.get_controller_inputs()

        # Tracking Check
        if pose is None:
            if self.tracking_active:
                self.get_logger().warn(f"Tracking LOST for {self.side}")
                self.tracking_active = False
            return

        if not self.tracking_active:
            self.get_logger().info(f"Tracking ACQUIRED for {self.side}! Buzzing...")
            self.tracking_active = True
            self.haptic_frames_remaining = 50

        # Safety Check
        raw_pos = pose[0:3]
        if not self.is_calibrating:
            if not self.check_workspace_and_vibrate(raw_pos, controller):
                return

        # --- Publish Data ---
        timestamp = self.get_clock().now().to_msg()
        t = self.get_clock().now().nanoseconds / 1e9

        msg_pose = PoseStamped()
        msg_pose.header.stamp = timestamp
        msg_pose.header.frame_id = self.frame_id

        msg_pose.pose.position.x = self.filters['x'](pose[0], t) * self.linear_scale
        msg_pose.pose.position.y = self.filters['y'](pose[1], t) * self.linear_scale
        msg_pose.pose.position.z = self.filters['z'](pose[2], t) * self.linear_scale

        msg_pose.pose.orientation.x = pose[3]
        msg_pose.pose.orientation.y = pose[4]
        msg_pose.pose.orientation.z = pose[5]
        msg_pose.pose.orientation.w = pose[6]
        self.pub_pose.publish(msg_pose)

        # Buttons
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
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.15
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