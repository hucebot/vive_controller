import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')
    rviz_config = os.path.join(pkg_share, 'config', 'view_vive.rviz')
    vive_params_file = os.path.join(pkg_share, 'config', 'vive.params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz to visualize'
        ),

        # # --- LEFT HAND DRIVER ---
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='driver_left',
            namespace='vive/left',
            output='screen',
            parameters=[
                vive_params_file,
                {'side': 'left',
                 'serial': 'LHR-97752221'}
            ]
        ),

        # --- RIGHT HAND DRIVER ---
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='driver_right',
            namespace='vive/right',
            output='screen',
            parameters=[
                vive_params_file,
                {'side': 'right',
                 'serial': 'LHR-4BB3817E'
                 }
            ]
        ),

        # --- RVIZ ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=[
                '-d', rviz_config] if os.path.exists(rviz_config) else []
        )
    ])
