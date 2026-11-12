"""Vive drivers and RViz only, no robot (`make drivers`)."""
from ros2_vive_controller.robot_launch import generate_drivers


def generate_launch_description():
    return generate_drivers()
