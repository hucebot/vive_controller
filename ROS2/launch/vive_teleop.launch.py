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

    # Hardcoded parameters dictionary
    vive_params = {
        'htc_vive.tracking_reference': 'LHB-DFA5BD2C',
        'htc_vive.serial_left': 'LHR-97752221',
        'htc_vive.serial_right': 'LHR-4BB3817E',
        'frame_id': 'vive_world',
        'workspace.x_min': -1.5,
        'workspace.x_max': 1.5,
        'workspace.y_min': -1.5,
        'workspace.y_max': 1.5,
        'workspace.z_min': 0.2,
        'workspace.z_max': 2.2,
        'workspace.padding': 0.15,
        'filter.mincutoff': 1.0,
        'filter.beta': 0.007,
        'filter.dcutoff': 1.0
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz to visualize'
        ),

        # --- LEFT HAND DRIVER ---
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='driver_left',
            namespace='vive/left',
            output='screen',
            parameters=[
                vive_params,
                {'side': 'left'}
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
                vive_params,
                {'side': 'right'}
            ]
        ),

        # --- RVIZ ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
        )
    ])