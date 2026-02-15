import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Get the package directory
    pkg_dir = get_package_share_directory('ros2_vive_controller')

    # Path to RViz file
    # Ensure you have a folder named 'rviz' inside your package root!
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'calibration.rviz')

    print(f"DEBUG: Looking for RViz config at: {rviz_config_path}")

    return LaunchDescription([
        # 1. The Calibration Node
        Node(
            package='ros2_vive_controller',
            executable='calibration_node',
            name='calibration_node',
            output='screen'
        ),

        # 2. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
