#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge_node')

        # Declare parameters
        self.declare_parameter('pose_topic', '/vive/left')
        self.declare_parameter('button_state_topic', '/vive/button_state')
        self.declare_parameter('output_topic', '/vive/output_pose')
        self.declare_parameter('publish_frequency', 30.0)  # -1 for event-driven
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('reference_frame', 'world')

        # Get parameters
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.button_state_topic = self.get_parameter('button_state_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for latest messages
        self.latest_pose = None
        self.latest_button_state = None
        self.ee_translation = None
        self.joy_translation = None
        self.prev_button_state = None
        self.activated = False
        self.last_output_msg = None

        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.button_state_sub = self.create_subscription(
            JointState,
            self.button_state_topic,
            self.button_state_callback,
            10
        )

        # Create publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10
        )

        # Create timer if frequency-based publishing is enabled
        if self.publish_frequency > 0:
            timer_period = 1.0 / self.publish_frequency
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(
                f'Publishing at {self.publish_frequency} Hz from {self.pose_topic} to {self.output_topic}'
            )
        else:
            self.timer = None
            self.get_logger().info(
                f'Publishing on new data from {self.pose_topic} to {self.output_topic}'
            )

        self.get_logger().info(f'Subscribed to pose: {self.pose_topic}')
        self.get_logger().info(f'Subscribed to button_state: {self.button_state_topic}')

    def pose_callback(self, msg):
        """Store the latest pose message"""
        self.latest_pose = msg

        # If event-driven mode (frequency = -1), publish immediately
        if self.publish_frequency <= 0:
            self.publish_pose()

    def activate(self):
        """Activate: Save current EE and joystick translations"""
        try:
            # Look up the transform (end effector)
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            # Save the end effector translation
            self.ee_translation = transform.transform.translation

            # Save the joystick/pose translation
            if self.latest_pose is not None:
                self.joy_translation = self.latest_pose.pose.position

            self.get_logger().info(
                f'Button pressed! Saved EE translation: '
                f'x={self.ee_translation.x:.3f}, '
                f'y={self.ee_translation.y:.3f}, '
                f'z={self.ee_translation.z:.3f}'
            )
            if self.joy_translation is not None:
                self.get_logger().info(
                    f'Button pressed! Saved Joy translation: '
                    f'x={self.joy_translation.x:.3f}, '
                    f'y={self.joy_translation.y:.3f}, '
                    f'z={self.joy_translation.z:.3f}'
                )
            self.activated = True
        except Exception as e:
            self.get_logger().warn(f'Failed to lookup transform: {e}')

    def button_state_callback(self, msg):
        """Store the latest button state message and check for button press"""
        self.latest_button_state = msg

        # Check if first button was just pressed (transition from 0 to 1)
        if len(msg.position) > 0:
            current_button = msg.position[0]
            prev_button = self.prev_button_state.position[0] if self.prev_button_state and len(self.prev_button_state.position) > 0 else 0.0

            # Detect rising edge (button press)
            if current_button == 1.0 and prev_button != 1.0:
                self.activate()

        # Store current state for next comparison
        self.prev_button_state = msg

        # If event-driven mode (frequency = -1), publish immediately when button state changes
        if self.publish_frequency <= 0:
            self.publish_pose()

    def timer_callback(self):
        """Publish at fixed frequency"""
        self.publish_pose()

    def publish_pose(self):
        """Publish the latest pose if available"""
        self.output_msg = None
        if self.latest_pose is not None and self.activated:
            # remove the joystick translation
            delta_x = self.latest_pose.pose.position.x - self.joy_translation.x
            delta_y = self.latest_pose.pose.position.y - self.joy_translation.y
            delta_z = self.latest_pose.pose.position.z - self.joy_translation.z
            
            # Apply delta to EE position
            output_msg = PoseStamped()
            output_msg.header = self.latest_pose.header
            output_msg.header.stamp = self.get_clock().now().to_msg()       
            output_msg.pose.position.x = self.ee_translation.x + delta_x
            output_msg.pose.position.y = self.ee_translation.y + delta_y
            output_msg.pose.position.z = self.ee_translation.z + delta_z
            
            # keep the orientation 
            output_msg.pose.orientation = self.latest_pose.pose.orientation

            # save for future use
            self.last_output_msg = output_msg
        else:
            # if not activated, publish the last computed message
            # (if we want to publish continuously, we push always the same)
            output_msg = self.last_output_msg if self.last_output_msg is not None
        
        # publish (since we are asked to publish)
        # but nothing will be published until the first message
        if output_msg is not None:
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
