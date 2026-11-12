"""The calibrated fence must actually reach the driver.

Regression: calibration wrote the bounds under '/**' in vive.params.yaml, but a later '/vive/**'
section of the same file set +-10 m bounds. rclpy matches both sections against the driver's name
(/vive/right/...) and the last one wins, so the calibrated fence was never applied.
"""
import os

import pytest
from fake_openvr import CONTROLLER, FakeDevice

from ros2_vive_controller.workspace import write_workspace_file

rclpy = pytest.importorskip('rclpy')

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULTS = os.path.join(REPO_ROOT, 'config', 'vive.params.yaml')
CALIBRATED = {'x_min': -0.9, 'x_max': -0.4, 'y_min': -0.3, 'y_max': 0.25, 'z_min': 0.1, 'z_max': 0.7}


def _driver_with_params_files(*files):
    """A right-hand driver in the /vive/right namespace, as vive_teleop.launch.py starts it."""
    args = ['--ros-args', '-r', '__ns:=/vive/right', '-p', 'serial:=LHR-RIGHT']
    for path in files:
        args += ['--params-file', path]
    rclpy.init(args=args)
    from ros2_vive_controller.vive_node import ViveDriverNode
    return ViveDriverNode()


@pytest.fixture
def ros(vr_system):
    vr_system.plug(2, FakeDevice(CONTROLLER, 'LHR-RIGHT'), announce=False)
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_defaults_leave_the_fence_off(ros):
    node = _driver_with_params_files(DEFAULTS)
    assert node.get_fully_qualified_name().startswith('/vive/right/')
    assert node.ws['enabled'] is False
    assert node.ws['pad'] == 0.15
    assert node.get_parameter('filter.beta').value == 0.01
    node.destroy_node()


def test_calibrated_workspace_file_wins_over_the_defaults(ros, tmp_path):
    workspace_file = str(tmp_path / 'workspace.yaml')
    write_workspace_file(workspace_file, CALIBRATED, 'lighthouse_origin')
    node = _driver_with_params_files(DEFAULTS, workspace_file)
    assert node.ws['enabled'] is True
    assert node.ws['frame_id'] == 'lighthouse_origin'
    for key, value in CALIBRATED.items():
        assert node.ws[key] == value, key
    assert node.ws['pad'] == 0.15            # still from the defaults
    node.destroy_node()
