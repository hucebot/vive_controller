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

    # 2. Define the Include action for hardware drivers
    include_vive_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_path),
        launch_arguments={'rviz': 'true'}.items()
    )

    # 3. General Launch Arguments
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='30.0',
        description='Publishing frequency in Hz.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='panda_link0',
        description='Reference frame for transform lookup'
    )


    target_frame_franka_tcp_arg = DeclareLaunchArgument(
        'target_frame_franka_tcp',
        default_value='panda_hand_tcp',
        description='Target frame for Franka TCP controller'
    )

    # 5. Teleop bridge node - RIGHT
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

            # --- ADD THE ROTATION OFFSET HERE ---
            'rotation_offset': [180.0, 0.0, 0.0],

            # --- MANUAL BUTTON TOPIC MAPPING (RIGHT) ---
            'trigger_topic': '/vive/right/trigger',
            'trackpad_x_topic': '/vive/right/trackpad_x',
            'trackpad_y_topic': '/vive/right/trackpad_y',
            'grip_topic': '/panda_gripper/gripper_command',
            'menu_topic': '/vive/right/menu',
            'trackpad_touched_topic': '/vive/right/trackpad_touched',
            'trackpad_pressed_topic': '/vive/right/trackpad_pressed',
        }]
    )
    return LaunchDescription([
        include_vive_teleop,
        publish_frequency_arg,
        reference_frame_arg,
        target_frame_franka_tcp_arg,
        teleop_bridge_franka_tcp,
    ])