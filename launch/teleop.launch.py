"""Teleop for any robot profile: ros2 launch ros2_vive_controller teleop.launch.py robot:=<name>

The profiles are in config/robots/. The robot launch files (franka_single.launch.py, ...) do the same
with their robot preset.
"""
from ros2_vive_controller.robot_launch import generate


def generate_launch_description():
    return generate()
