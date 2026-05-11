import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')
    included_launch_path = os.path.join(pkg_share, 'launch', 'vive_teleop.launch.py')

    # --- General Launch Arguments ---
    serial_right_arg = DeclareLaunchArgument(
        'serial_right', default_value='LHR-9ABF6D66', description='Serial number for the right controller'
    )

    serial_left_arg = DeclareLaunchArgument(
        'serial_left', default_value='LHR-97752221', description='Serial number for the left controller'
    )

    reference_lighthouse_serial_arg = DeclareLaunchArgument(
        'reference_lighthouse_serial', default_value='', description='Serial number for the reference lighthouse'
    )

    only_right_arg = DeclareLaunchArgument(
        'only_right', default_value='true', description="If set, only the right controller will be active."
    )

    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale', default_value='1.0', description='Scaling factor for controller translation.'
    )

    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency', default_value='30.0', description='Publishing frequency in Hz.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='panda_link0', description='Reference frame for transform lookup'
    )

    target_frame_franka_tcp_arg = DeclareLaunchArgument(
        'target_frame_franka_tcp', default_value='panda_hand_tcp', description='Target frame for Franka TCP controller'
    )

    # --- Hardware Driver Inclusion ---
    include_vive_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_path),
        launch_arguments={
            'rviz': 'true',
            'serial_right': LaunchConfiguration('serial_right'),
            'serial_left': LaunchConfiguration('serial_left'),
            'reference_lighthouse_serial': LaunchConfiguration('reference_lighthouse_serial'), # --- NEW: Pass it down ---
            'only_right': LaunchConfiguration('only_right'),
            'linear_scale': LaunchConfiguration('linear_scale')
        }.items()
    )

    # --- Teleop Bridge Node ---
    teleop_bridge_franka_tcp = Node(
        package='ros2_vive_controller',
        executable='teleop_bridge_node',
        name='teleop_bridge_franka_tcp',
        output='screen',
        parameters=[{
            'pose_topic': '/vive/right/pose',
            'button_state_topic': '/vive/right/joint_states',
            'output_topic': '/cartesian_impedance/equilibrium_pose',
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'target_frame': LaunchConfiguration('target_frame_franka_tcp'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'rotation_offset': [180.0, 0.0, 0.0],
            'trigger_topic': '/vive/right/trigger',
            'trackpad_x_topic': '/vive/right/trackpad_x',
            'trackpad_y_topic': '/vive/right/trackpad_y',
            'grip_topic': '/vive/right/grip_button',
            'menu_topic': '/panda_gripper/gripper_command',
            'trackpad_touched_topic': '/vive/right/trackpad_touched',
            'trackpad_pressed_topic': '/vive/right/trackpad_pressed',
        }]
    )

    return LaunchDescription([
        serial_right_arg,
        serial_left_arg,
        reference_lighthouse_serial_arg,
        only_right_arg,
        linear_scale_arg,
        publish_frequency_arg,
        reference_frame_arg,
        target_frame_franka_tcp_arg,
        include_vive_teleop,
        teleop_bridge_franka_tcp,
    ])