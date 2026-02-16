import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the vive_teleop.launch.py file
    pkg_share = get_package_share_directory('ros2_vive_controller')
    included_launch_path = os.path.join(pkg_share, 'launch', 'vive_teleop.launch.py')

    # 2. Define the Include action
    include_vive_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_path),
        launch_arguments={'rviz': 'true'}.items() # Optional: pass args to the included launch
    )

    # Declare launch arguments
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='30.0',
        description='Publishing frequency in Hz.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='base_link',
        description='Reference frame for transform lookup'
    )

    target_frame_left_arg = DeclareLaunchArgument(
        'target_frame_left',
        default_value='gripper_left_grasping_frame',
        description='Target frame for left controller'
    )

    target_frame_right_arg = DeclareLaunchArgument(
        'target_frame_right',
        default_value='gripper_right_grasping_frame',
        description='Target frame for right controller'
    )

    # Teleop bridge nodes
    teleop_bridge_left = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_left',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/left/pose', # Adjusted to match driver output
            'button_state_topic': '/vive/left/joint_states',
            'output_topic': '/vive/left/output_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_left'),
            'reference_frame': LaunchConfiguration('reference_frame'),
        }]
    )

    teleop_bridge_right = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_right',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/right/pose', # Adjusted to match driver output
            'button_state_topic': '/vive/right/joint_states',
            'output_topic': '/vive/right/output_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_right'),
            'reference_frame': LaunchConfiguration('reference_frame'),
        }]
    )

    return LaunchDescription([
        include_vive_teleop, # Starts the drivers first
        publish_frequency_arg,
        reference_frame_arg,
        target_frame_left_arg,
        target_frame_right_arg,
        teleop_bridge_left,
        teleop_bridge_right,
    ])