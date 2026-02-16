from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='30.0',
        description='Publishing frequency in Hz. Set to -1 for event-driven publishing'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='world',
        description='Reference frame for transform lookup'
    )

    target_frame_left_arg = DeclareLaunchArgument(
        'target_frame_left',
        default_value='left_end_effector',
        description='Target frame for left controller'
    )

    target_frame_right_arg = DeclareLaunchArgument(
        'target_frame_right',
        default_value='right_end_effector',
        description='Target frame for right controller'
    )

    # Left controller teleop bridge node
    teleop_bridge_left = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_left',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/left',
            'button_state_topic': '/vive/left/button_state',
            'output_topic': '/vive/left/output_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_left'),
            'reference_frame': LaunchConfiguration('reference_frame'),
        }]
    )

    # Right controller teleop bridge node
    teleop_bridge_right = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_right',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/right',
            'button_state_topic': '/vive/right/button_state',
            'output_topic': '/vive/right/output_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_right'),
            'reference_frame': LaunchConfiguration('reference_frame'),
        }]
    )

    return LaunchDescription([
        publish_frequency_arg,
        reference_frame_arg,
        target_frame_left_arg,
        target_frame_right_arg,
        teleop_bridge_left,
        teleop_bridge_right,
    ])
