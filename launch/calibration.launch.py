import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vive_controller')
    rviz_config = os.path.join(pkg_share, 'rviz', 'calibration.rviz')
    vive_params_file = os.path.join(pkg_share, 'config', 'vive.params.yaml')

    # Written to the SOURCE tree so it persists after a container restart
    default_workspace_file = '/ros2_ws/src/ros2_vive_controller/config/workspace.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial', default_value='',
            description='Serial of the controller used to paint (SERIAL_RIGHT in .env)'
        ),
        DeclareLaunchArgument(
            'reference_lighthouse_serial', default_value='',
            description='Use the same value as when teleoperating, so the fence is in the same frame'
        ),
        DeclareLaunchArgument(
            'workspace_file',
            default_value=default_workspace_file,
            description='Where to write the calibrated workspace fence'
        ),

        # 1. THE DRIVER (The Data Source)
        # We need this running so /vive/right/pose exists!
        Node(
            package='ros2_vive_controller',
            executable='vive_node',
            name='calibration_driver_right',
            namespace='vive/right',
            output='screen',
            parameters=[
                vive_params_file,
                {'side': 'right',
                 'serial': LaunchConfiguration('serial'),
                 'reference_lighthouse_serial': LaunchConfiguration('reference_lighthouse_serial')}
            ]
        ),

        # 2. THE CALIBRATION NODE (The Data Processor)
        Node(
            package='ros2_vive_controller',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[{
                'workspace_file': LaunchConfiguration('workspace_file')
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
