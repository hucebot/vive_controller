import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Get Package Path
    pkg_share = get_package_share_directory('ros2_vive_controller')

    # 2. Define Paths
    # RViz Config: We put this in 'config' to keep things organized, matching your other files
    rviz_config = os.path.join(pkg_share, 'config', 'calibration.rviz')

    # Config File: This is where the node will WRITE the new calibration data.
    # CRITICAL: We default to the SOURCE path inside the Docker container so changes persist.
    # If we used pkg_share, we would edit the 'install' folder, which is lost on rebuild.
    default_config_path = '/ros2_ws/src/ros2_vive_controller/config/vive.params.yaml'

    return LaunchDescription([
        # Allow overriding the path from command line if needed
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to vive.params.yaml (Source file, not installed file)'
        ),

        # 3. Calibration Node
        Node(
            package='ros2_vive_controller',
            executable='calibration_node', # Entry point defined in setup.py
            name='calibration_node',
            output='screen',
            parameters=[{
                'target_frame': 'vive_world',
                'config_path': LaunchConfiguration('config_path')
            }]
        ),

        # 4. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])