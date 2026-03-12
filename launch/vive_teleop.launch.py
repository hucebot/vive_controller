import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')

    # Check if we should use 'vive.rviz' or 'view_vive.rviz' based on your file tree
    rviz_filename = 'view_vive.rviz'
    rviz_config = os.path.join(pkg_share, 'rviz', rviz_filename)
    vive_params_file = os.path.join(pkg_share, 'config', 'vive.params.yaml')

    # --- DEBUG LOGGING ---
    print(f"\n[DEBUG] Package Share Path: {pkg_share}")
    print(f"[DEBUG] Looking for RViz file at: {rviz_config}")

    if os.path.exists(rviz_config):
        print(f"[DEBUG] ✅ File FOUND.\n")
    else:
        print(f"[DEBUG] ❌ File NOT FOUND.")
        print(f"[DEBUG] Contents of {os.path.join(pkg_share, 'rviz')}:")
        try:
            print(os.listdir(os.path.join(pkg_share, 'rviz')))
        except FileNotFoundError:
            print("   (The 'rviz' directory itself was not found in share. Check setup.py data_files!)")
        print("\n")
    # ---------------------

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz to visualize'
        ),

        DeclareLaunchArgument(
            'serial_left',
            default_value='LHR-97752221',
            description='Serial number for the left controller'
        ),

        DeclareLaunchArgument(
            'serial_right',
            default_value='LHR-9ABF6D66',
            description='Serial number for the right controller'
        ),

        DeclareLaunchArgument(
            'tracking_reference',
            default_value='LHB-DFA5BD2C',
            description='Serial number for the tracking reference (base station)'
        ),

        # --- LEFT HAND DRIVER ---
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='driver_left',
            namespace='vive/left',
            output='screen',
            parameters=[
                vive_params_file,
                {'side': 'left',
                 'serial': LaunchConfiguration('serial_left'),
                 'htc_vive.tracking_reference': LaunchConfiguration('tracking_reference')}
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
                     'serial': LaunchConfiguration('serial_right'),
                     'htc_vive.tracking_reference': LaunchConfiguration('tracking_reference')
                     }
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