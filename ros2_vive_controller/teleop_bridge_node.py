#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped  # Added PointStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from rclpy.duration import Duration
import numpy as np

class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge_node')

        # --- 1. Parameters ---
        self.declare_parameter('pose_topic', '/vive/left')
        self.declare_parameter('button_state_topic', '/vive/button_state')
        self.declare_parameter('output_topic', '/vive/output_pose')
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('rotation_offset', [0.0, 0.0, 0.0]) # [R, P, Y] in degrees

        self.button_keys = [
            'trigger', 'trackpad_x', 'trackpad_y',
            'grip', 'menu', 'trackpad_touched', 'trackpad_pressed'
        ]

        for key in self.button_keys:
            self.declare_parameter(f'{key}_topic', '')

        # --- 2. Publishers Dictionary ---
        self.button_pubs = {}
        for key in self.button_keys:
            topic = self.get_parameter(f'{key}_topic').value
            if topic:
                # All button/axis topics now use PointStamped
                self.button_pubs[key] = self.create_publisher(PointStamped, topic, 10)

        # Standard Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('output_topic').value, 10)

        # --- 3. TF2 and State ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_pose = None
        self.latest_button_state = None
        self.ee_translation = None
        self.joy_translation = None
        self.prev_button_state = None
        self.activated = False
        self.last_output_msg = None

        # --- 4. Subscribers ---
        self.pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self.pose_callback, 10)

        self.button_state_sub = self.create_subscription(
            JointState, self.get_parameter('button_state_topic').value, self.button_state_callback, 10)

        # --- 5. Timer ---
        freq = self.get_parameter('publish_frequency').value
        if freq > 0:
            self.create_timer(1.0 / freq, self.timer_callback)

    def button_state_callback(self, msg):
        """Store button state, handle activation clutch, and publish individual PointStamped topics"""
        self.latest_button_state = msg
        button_data = {name: pos for name, pos in zip(msg.name, msg.position)}

        # --- Individual Topic Publishing ---
        timestamp = self.get_clock().now().to_msg()
        for key, pub in self.button_pubs.items():
            if key in button_data:
                val = float(button_data[key])

                # Create PointStamped message
                p_msg = PointStamped()
                p_msg.header.stamp = timestamp
                p_msg.header.frame_id = self.get_parameter('reference_frame').value

                # Set value on x as requested
                p_msg.point.x = val
                p_msg.point.y = 0.0
                p_msg.point.z = 0.0

                pub.publish(p_msg)

        # --- Activation (Clutch) Logic ---
        if 'trigger' in button_data:
            clutch = button_data['trigger']
            prev_clutch = 0.0
            if self.prev_button_state:
                prev_data = {n: p for n, p in zip(self.prev_button_state.name, self.prev_button_state.position)}
                prev_clutch = prev_data.get('trigger', 0.0)

            if clutch == 1.0 and prev_clutch != 1.0:
                self.activate()
            elif clutch != 1.0 and prev_clutch == 1.0:
                self.activated = False

        self.prev_button_state = msg

    def activate(self):
        """Save reference transforms when the clutch is engaged"""
        try:
            target = self.get_parameter('target_frame').value
            ref = self.get_parameter('reference_frame').value

            transform = self.tf_buffer.lookup_transform(ref, target, rclpy.time.Time(), timeout=Duration(seconds=1.0))
            self.ee_translation = transform.transform.translation

            if self.latest_pose:
                self.joy_translation = self.latest_pose.pose.position
                self.activated = True
                self.get_logger().info('Teleop Activated: Reference frames saved.')
        except Exception as e:
            self.get_logger().warn(f'Activation failed: {e}')

    def pose_callback(self, msg):
        self.latest_pose = msg
        if self.get_parameter('publish_frequency').value <= 0:
            self.publish_pose()

    def timer_callback(self):
        self.publish_pose()

    def publish_pose(self):
        """Compute delta from reference and publish updated pose"""
        output_msg = None

        if self.latest_pose and self.activated:
            # --- 1. Position Delta Calculation ---
            delta_x = self.latest_pose.pose.position.x - self.joy_translation.x
            delta_y = self.latest_pose.pose.position.y - self.joy_translation.y
            delta_z = self.latest_pose.pose.position.z - self.joy_translation.z

            output_msg = PoseStamped()
            output_msg.header = self.latest_pose.header
            output_msg.header.frame_id = self.get_parameter('reference_frame').value
            output_msg.header.stamp = self.get_clock().now().to_msg()

            output_msg.pose.position.x = self.ee_translation.x + delta_x
            output_msg.pose.position.y = self.ee_translation.y + delta_y
            output_msg.pose.position.z = self.ee_translation.z + delta_z

            # --- 2. Orientation + Rotation Offset Logic ---
            q = self.latest_pose.pose.orientation

            # Get the offset from parameters (Default is [0,0,0])
            offset_euler = self.get_parameter('rotation_offset').value

            # If offset is identity, just copy; otherwise, do the math
            if offset_euler == [0.0, 0.0, 0.0]:
                output_msg.pose.orientation = q
            else:
                # Convert current orientation and offset to Scipy Rotation objects
                curr_rot = R.from_quat([q.x, q.y, q.z, q.w])
                off_rot = R.from_euler('xyz', offset_euler, degrees=True)

                # Post-multiply (Local Frame Rotation)
                # This ensures the flip happens relative to the controller's own axes
                final_rot = curr_rot * off_rot
                new_q = final_rot.as_quat() # returns [x, y, z, w]

                output_msg.pose.orientation.x = new_q[0]
                output_msg.pose.orientation.y = new_q[1]
                output_msg.pose.orientation.z = new_q[2]
                output_msg.pose.orientation.w = new_q[3]

            self.last_output_msg = output_msg
        else:
            output_msg = self.last_output_msg

        if output_msg:
            self.pose_pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()