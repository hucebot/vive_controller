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

    # 2. Hardware arguments (set SERIAL_LEFT / SERIAL_RIGHT in .env; they used to be hardcoded
    #    here, which silently overrode .env)
    serial_right_arg = DeclareLaunchArgument(
        'serial_right', default_value='', description='Serial number for the right controller'
    )
    serial_left_arg = DeclareLaunchArgument(
        'serial_left', default_value='', description='Serial number for the left controller'
    )
    reference_lighthouse_serial_arg = DeclareLaunchArgument(
        'reference_lighthouse_serial', default_value='',
        description='Lighthouse used as TF origin (changes the frame poses are published in)'
    )
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale', default_value='1.0', description='Linear scaling for both controllers'
    )

    # Define the Include action for hardware drivers
    include_vive_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_path),
        launch_arguments={
            'rviz': 'false',
            'serial_right': LaunchConfiguration('serial_right'),
            'serial_left': LaunchConfiguration('serial_left'),
            'reference_lighthouse_serial': LaunchConfiguration('reference_lighthouse_serial'),
            'linear_scale': LaunchConfiguration('linear_scale'),
        }.items()
    )

    # 3. General Launch Arguments
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='-1.0',
        description='Publishing frequency in Hz.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='world',
        description='Reference frame for transform lookup'
    )

    target_frame_left_arg = DeclareLaunchArgument(
        'target_frame_left',
        default_value='left_hand_point_contact',
        description='Target frame for left controller'
    )

    target_frame_right_arg = DeclareLaunchArgument(
        'target_frame_right',
        default_value='right_hand_point_contact',
        description='Target frame for right controller'
    )

    # 4. Teleop bridge node - LEFT
    teleop_bridge_left = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_left',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/left/pose',
            'button_state_topic': '/vive/left/joint_states',
            'output_topic': '/g1pilot/left_hand/pose_ref',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_left'),
            'reference_frame': LaunchConfiguration('reference_frame'),

            # --- MANUAL BUTTON TOPIC MAPPING (LEFT) ---
            'trigger_topic': '/vive/left/trigger',
            # 'trackpad_x_topic': '/vive/left/trackpad_x',
            'trackpad_x_topic': '/g1pilot/left_hand/dx3/action',
            'trackpad_y_topic': '/vive/left/trackpad_y',
            'grip_topic': '/vive/left/grip',
            'menu_topic': '/vive/left/menu',
            'trackpad_touched_topic': '/vive/left/trackpad_touched',
            'trackpad_pressed_topic': '/vive/left/trackpad_pressed',
            'trackpad_pressed_required' : 'true'  # Only publish when trackpad is pressed
        }]
    )

    # 5. Teleop bridge node - RIGHT
    teleop_bridge_right = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_right',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/right/pose',
            'button_state_topic': '/vive/right/joint_states',
            'output_topic': '/g1pilot/right_hand/pose_ref',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_right'),
            'reference_frame': LaunchConfiguration('reference_frame'),

            # --- MANUAL BUTTON TOPIC MAPPING (RIGHT) ---
            'trigger_topic': '/vive/right/trigger',
            # 'trackpad_x_topic': '/vive/right/trackpad_x',
            'trackpad_y_topic': '/vive/right/trackpad_y',
            'trackpad_x_topic': '/g1pilot/right_hand/dx3/action',
            'grip_topic': '/vive/right/grip',
            'menu_topic': '/vive/right/menu',
            'trackpad_touched_topic': '/vive/right/trackpad_touched',
            'trackpad_pressed_topic': '/vive/right/trackpad_pressed',
            'trackpad_pressed_required' : 'true'  #
        }]
    )

    return LaunchDescription([
        serial_right_arg,
        serial_left_arg,
        reference_lighthouse_serial_arg,
        linear_scale_arg,
        include_vive_teleop,
        publish_frequency_arg,
        reference_frame_arg,
        target_frame_left_arg,
        target_frame_right_arg,
        teleop_bridge_left,
        teleop_bridge_right,
    ])