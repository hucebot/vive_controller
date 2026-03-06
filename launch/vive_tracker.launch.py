import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')
    params_file = os.path.join(pkg_share, 'config', 'vive_tracker.params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_trackers.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('serial_left', default_value='', description='Serial for left tracker'),
        DeclareLaunchArgument('serial_right', default_value='', description='Serial for right tracker'),

        # Left tracker
        Node(
            package='ros2_vive_controller',
            executable='tracker_node',
            name='tracker_left',
            namespace='vive_tracker/left',
            output='screen',
            parameters=[
                params_file,
                {'name': 'left', 'serial': LaunchConfiguration('serial_left')},
            ],
        ),

        # Right tracker (delayed 3s to avoid OpenVR conflict)
        TimerAction(period=3.0, actions=[
            Node(
                package='ros2_vive_controller',
                executable='tracker_node',
                name='tracker_right',
                namespace='vive_tracker/right',
                output='screen',
                parameters=[
                    params_file,
                    {'name': 'right', 'serial': LaunchConfiguration('serial_right')},
                ],
            ),
        ]),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config],
        ),
    ])
