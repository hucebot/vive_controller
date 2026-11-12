"""The calibrated fence must actually reach the driver.

Regression: calibration wrote the bounds under '/**' in vive.params.yaml, but a later '/vive/**'
section of the same file set +-10 m bounds. rclpy matches both sections against the driver's name
(/vive/right/...) and the last one wins, so the calibrated fence was never applied.
"""
import os

import pytest
from ros2_vive_controller.testing.fake_openvr import CONTROLLER, FakeDevice

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


def test_station_fence_wins_over_the_legacy_workspace_file(ros, tmp_path):
    """launch_ros writes a Node's dict parameters to a temporary file keyed by the full node name,
    after the parameter files; the station fence is passed that way (launch_config.driver_parameters)."""
    import yaml

    from ros2_vive_controller import station as st
    from ros2_vive_controller.launch_config import driver_parameters

    legacy = str(tmp_path / 'workspace.yaml')
    write_workspace_file(legacy, CALIBRATED, 'vive_world')
    painted = {'x_min': -0.5, 'x_max': 0.5, 'y_min': -0.5, 'y_max': 0.5, 'z_min': 0.2, 'z_max': 1.2}
    fence = st.set_fence(st.empty(), 'right', painted, 'lighthouse_origin')['workspace']['right']
    params = driver_parameters('right', 'LHR-RIGHT', '', 1.0, fence)
    launch_file = tmp_path / 'launch_params.yaml'
    launch_file.write_text(yaml.dump({'/vive/right/driver_right': {'ros__parameters': params}}))

    rclpy.init(args=['--ros-args', '-r', '__ns:=/vive/right', '-r', '__node:=driver_right',
                     '--params-file', DEFAULTS, '--params-file', legacy, '--params-file', str(launch_file)])
    from ros2_vive_controller.vive_node import ViveDriverNode
    node = ViveDriverNode()
    assert node.ws['enabled'] is True
    assert node.ws['frame_id'] == 'lighthouse_origin'
    for key, value in painted.items():
        assert node.ws[key] == value, key
    node.destroy_node()
