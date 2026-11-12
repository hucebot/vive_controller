"""TIAGo Pro, both arms. The robot is described in config/robots/tiago_pro.yaml (see robot_launch.py)."""
from ros2_vive_controller.robot_launch import generate


def generate_launch_description():
    return generate('tiago_pro')
