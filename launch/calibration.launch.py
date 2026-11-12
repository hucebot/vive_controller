"""Paint an arm's workspace fence with its controller (`make calibrate`); saved in config/station.yaml.

Hold the trigger and sweep the space, then Ctrl+C to save. The setup app (`make setup`) does the
same from a browser.
"""
from ros2_vive_controller.robot_launch import generate_calibration


def generate_launch_description():
    return generate_calibration()
