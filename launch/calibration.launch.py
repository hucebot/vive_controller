import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')
    rviz_config = os.path.join(pkg_share, 'rviz', 'calibration.rviz')

    # Path to the SOURCE yaml so changes persist after container restart
    default_config_path = '/ros2_ws/src/ros2_vive_controller/config/vive.params.yaml'

    # --- Hardcoded Driver Params (For Calibration Convenience) ---
    driver_params = {
        'side': 'right',
        'serial': 'LHR-4BB3817E',
        'frame_id': 'vive_world'
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to vive.params.yaml'
        ),

        # 1. THE DRIVER (The Data Source)
        # We need this running so /vive/right/pose exists!
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='calibration_driver_right',
            namespace='vive/right',
            output='screen',
            parameters=[driver_params]
        ),

        # 2. THE CALIBRATION NODE (The Data Processor)
        Node(
            package='ros2_vive_controller',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[{
                'target_frame': 'vive_world',
                'config_path': LaunchConfiguration('config_path')
            }]
        ),

        # 3. RVIZ2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])