import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    #  Locate paths
    pkg_share = get_package_share_directory('ros2_vive_controller')
    included_launch_path = os.path.join(pkg_share, 'launch', 'vive_teleop.launch.py')

    # --- General Launch Arguments ---
    serial_right_arg = DeclareLaunchArgument(
        'serial_right', default_value='LHR-9ABF6D66',
        description='Serial number for the right controller'
    )
    serial_left_arg = DeclareLaunchArgument(
        'serial_left', default_value='LHR-97752221',
        description='Serial number for the left controller'
    )
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale', default_value='1.0',
        description='Linear scaling for both controllers'
    )
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency', default_value='30.0'
    )
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='opensot/base_link'
    )

    # --- Hardware Driver Inclusion (Dual Mode) ---
    include_vive_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_path),
        launch_arguments={
            'rviz': 'true',
            'serial_right': LaunchConfiguration('serial_right'),
            'serial_left': LaunchConfiguration('serial_left'),
            'only_right': 'false', # Force dual-controller mode for Tiago
            'linear_scale': LaunchConfiguration('linear_scale')
        }.items()
    )

    # ---  Teleop Bridge - LEFT ---
    teleop_bridge_left = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_left',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/left/pose',
            'button_state_topic': '/vive/left/joint_states',
            'output_topic': '/vive/left/output_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_left', default='opensot/gripper_left_grasping_link'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'rotation_offset': [0.0, 0.0, 0.0], # Standard Tiago mapping

            'trigger_topic': '/vive/left/trigger',
            'trackpad_x_topic': '/vive/left/trackpad_x',
            'trackpad_y_topic': '/vive/left/trackpad_y',
            'grip_topic': '/vive/left/grip',
            'menu_topic': '/vive/left/gripper',
            'trackpad_touched_topic': '/vive/left/trackpad_touched',
            'trackpad_pressed_topic': '/vive/left/trackpad_pressed',
        }]
    )

    # --- Teleop Bridge - RIGHT ---
    teleop_bridge_right = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_right',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/right/pose',
            'button_state_topic': '/vive/right/joint_states',
            'output_topic': '/vive/right/output_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_right', default='opensot/gripper_right_grasping_link'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'rotation_offset': [0.0, 0.0, 0.0], # Standard Tiago mapping

            'trigger_topic': '/vive/right/trigger',
            'trackpad_x_topic': '/vive/right/trackpad_x',
            'trackpad_y_topic': '/vive/right/trackpad_y',
            'grip_topic': '/vive/right/grip',
            'menu_topic': '/vive/right/gripper',
            'trackpad_touched_topic': '/vive/right/trackpad_touched',
            'trackpad_pressed_topic': '/vive/right/trackpad_pressed',
        }]
    )

    return LaunchDescription([
        serial_right_arg,
        serial_left_arg,
        linear_scale_arg,
        publish_frequency_arg,
        reference_frame_arg,
        include_vive_teleop,
        teleop_bridge_left,
        teleop_bridge_right,
    ])