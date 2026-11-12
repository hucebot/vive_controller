"""Workspace fence logic and the calibration file format."""
import pytest
import yaml

from ros2_vive_controller.workspace import (INSIDE, OUTSIDE, WARNING, bounds_from_points, classify,
                                            write_workspace_file)

BOX = {'x_min': -1.0, 'x_max': 1.0, 'y_min': -1.0, 'y_max': 1.0, 'z_min': 0.0, 'z_max': 2.0}


@pytest.mark.parametrize('position, expected', [
    ((0.0, 0.0, 1.0), INSIDE),
    ((0.95, 0.0, 1.0), WARNING),      # within 0.1 of x_max
    ((0.0, -0.95, 1.0), WARNING),
    ((0.0, 0.0, 0.05), WARNING),
    ((1.05, 0.0, 1.0), OUTSIDE),
    ((0.0, 0.0, -0.01), OUTSIDE),
])
def test_classify(position, expected):
    assert classify(position, BOX, padding=0.1) == expected


def test_zero_padding_has_no_warning_band():
    assert classify((0.999, 0.0, 1.0), BOX, padding=0.0) == INSIDE


def test_bounds_grow_by_the_margin_and_drop_outliers():
    points = [(x / 10.0, 0.0, 1.0) for x in range(11)] + [(0.5, 0.1, 1.2)] * 3
    points += [(0.5, 0.0, 1.0)] * 20
    points.append((50.0, 50.0, 50.0))     # a tracking glitch
    bounds = bounds_from_points(points, margin=0.05)
    assert bounds['x_min'] == pytest.approx(-0.05)
    assert bounds['x_max'] == pytest.approx(1.05)
    assert bounds['z_max'] == pytest.approx(1.25)
    assert bounds['y_max'] < 1.0


def test_workspace_file_is_a_ros_parameter_file_with_its_frame(tmp_path):
    path = tmp_path / 'workspace.yaml'
    assert write_workspace_file(str(path), BOX, 'lighthouse_origin') is None
    data = yaml.safe_load(path.read_text())
    workspace = data['/**']['ros__parameters']['workspace']
    assert workspace['enabled'] is True
    assert workspace['frame_id'] == 'lighthouse_origin'
    assert workspace['x_min'] == -1.0 and workspace['z_max'] == 2.0


def test_previous_workspace_file_is_kept_as_backup(tmp_path):
    path = tmp_path / 'workspace.yaml'
    write_workspace_file(str(path), BOX, 'vive_world')
    first = path.read_text()
    backup = write_workspace_file(str(path), dict(BOX, x_max=3.0), 'vive_world')
    assert backup == str(path) + '.bak'
    assert (tmp_path / 'workspace.yaml.bak').read_text() == first
