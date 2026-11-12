"""Unitree G1, both hands. The robot is described in config/robots/g1.yaml (see robot_launch.py)."""
from ros2_vive_controller.robot_launch import generate


def generate_launch_description():
    return generate('g1')
