from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/vive/left',
        description='Topic name for input PoseStamped'
    )

    button_state_topic_arg = DeclareLaunchArgument(
        'button_state_topic',
        default_value='/vive/button_state',
        description='Topic name for input JointState (button state)'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/vive/output_pose',
        description='Topic name for output PoseStamped'
    )

    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='30.0',
        description='Publishing frequency in Hz. Set to -1 for event-driven publishing'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='Target frame to lookup when button is pressed'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='world',
        description='Reference frame for transform lookup'
    )

    # Teleop bridge node
    teleop_bridge_node = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_node',
        output='screen',
        parameters=[{
            'pose_topic': LaunchConfiguration('pose_topic'),
            'button_state_topic': LaunchConfiguration('button_state_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame'),
            'reference_frame': LaunchConfiguration('reference_frame'),
        }]
    )

    return LaunchDescription([
        pose_topic_arg,
        button_state_topic_arg,
        output_topic_arg,
        publish_frequency_arg,
        target_frame_arg,
        reference_frame_arg,
        teleop_bridge_node,
    ])
