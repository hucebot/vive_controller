import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')

    rviz_filename = 'view_vive.rviz'
    rviz_config = os.path.join(pkg_share, 'rviz', rviz_filename)
    vive_params_file = os.path.join(pkg_share, 'config', 'vive.params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('serial_left', default_value='LHR-97752221'),
        DeclareLaunchArgument('serial_right', default_value='LHR-9ABF6D66'),

        # --- CHANGED: Name matches parent launch and node parameter ---
        DeclareLaunchArgument('reference_lighthouse_serial', default_value=''),

        DeclareLaunchArgument(
            'only_right',
            default_value='false',
            description='If true, the left controller node is skipped.'
        ),
        DeclareLaunchArgument(
            'linear_scale',
            default_value='1.0',
            description='Translation scaling factor.'
        ),

        # --- LEFT HAND DRIVER ---
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='driver_left',
            namespace='vive/left',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('only_right')),
            parameters=[
                vive_params_file,
                {'side': 'left',
                 'linear_scale': LaunchConfiguration('linear_scale'),
                 'serial': LaunchConfiguration('serial_left'),
                 'reference_lighthouse_serial': LaunchConfiguration('reference_lighthouse_serial')} # <--- UPDATED
            ]
        ),

        # --- RIGHT HAND DRIVER ---
        TimerAction(period=3.0, actions=[
            Node(
                package='ros2_vive_controller',
                executable='vive_node',
                name='driver_right',
                namespace='vive/right',
                output='screen',
                parameters=[
                    vive_params_file,
                    {'side': 'right',
                     'linear_scale': LaunchConfiguration('linear_scale'),
                     'serial': LaunchConfiguration('serial_right'),
                     'reference_lighthouse_serial': LaunchConfiguration('reference_lighthouse_serial')} # <--- UPDATED
                ]
            ),
        ]),

        # --- RVIZ ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config]
        )
    ])