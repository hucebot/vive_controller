import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ros2_vive_controller")
    params_file = os.path.join(pkg_share, "config", "vive_tracker.params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "view_trackers.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz"
            ),
            DeclareLaunchArgument(
                "serial_left", default_value="", description="Serial for left tracker"
            ),
            DeclareLaunchArgument(
                "serial_right", default_value="", description="Serial for right tracker"
            ),
            DeclareLaunchArgument(
                "reference_lighthouse_serial",
                default_value="",
                description="Serial of lighthouse to use as reference frame",
            ),
            Node(
                package="ros2_vive_controller",
                executable="bimanual_tracker_node",
                name="bimanual_vive_tracker",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "serial_left": LaunchConfiguration("serial_left"),
                        "serial_right": LaunchConfiguration("serial_right"),
                        "reference_lighthouse_serial": LaunchConfiguration(
                            "reference_lighthouse_serial"
                        ),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", rviz_config],
            ),
        ]
    )
