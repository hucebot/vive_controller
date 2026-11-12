"""Franka Panda, right controller. The robot is described in config/robots/franka.yaml (see robot_launch.py)."""
from ros2_vive_controller.robot_launch import generate


def generate_launch_description():
    return generate('franka')
