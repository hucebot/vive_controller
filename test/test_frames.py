"""frames.py must give exactly what vive_node computed before the math was moved there."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from ros2_vive_controller.frames import (matrix_to_pose, origin_from_lighthouse, pose_in_origin,
                                         pose_to_matrix, relative_pose)


# --- vive_node's code before the refactor (verbatim maths) --------------------------------------
def _old_apply_rotation_offset(pose, axis='z', angle_degrees=180):
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    T[:3, :3] = R.from_quat(pose[3:]).as_matrix()
    T_rot = np.eye(4)
    T_rot[:3, :3] = R.from_euler(axis, angle_degrees, degrees=True).as_matrix()
    T_new = T @ T_rot
    pos = T_new[:3, 3]
    quat = R.from_matrix(T_new[:3, :3]).as_quat()
    return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]


def _old_compute_relative_pose(target_pose, reference_pose):
    T_ref = np.eye(4)
    T_ref[:3, 3] = reference_pose[:3]
    T_ref[:3, :3] = R.from_quat(reference_pose[3:]).as_matrix()
    T_target = np.eye(4)
    T_target[:3, 3] = target_pose[:3]
    T_target[:3, :3] = R.from_quat(target_pose[3:]).as_matrix()
    T_rel = np.linalg.inv(T_ref) @ T_target
    pos = T_rel[:3, 3]
    quat = R.from_matrix(T_rel[:3, :3]).as_quat()
    return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]


def _random_pose(rng):
    return list(rng.uniform(-3, 3, 3)) + list(R.random(random_state=rng.integers(1 << 30)).as_quat())


@pytest.mark.parametrize('seed', range(20))
def test_same_result_as_the_old_driver_code(seed):
    rng = np.random.default_rng(seed)
    controller, lighthouse = _random_pose(rng), _random_pose(rng)

    old_origin = _old_apply_rotation_offset(lighthouse, axis='z', angle_degrees=180)
    old = _old_compute_relative_pose(controller, old_origin)

    assert origin_from_lighthouse(lighthouse) == pytest.approx(old_origin, abs=1e-12)
    assert relative_pose(controller, old_origin) == pytest.approx(old, abs=1e-12)
    assert pose_in_origin(controller, lighthouse) == pytest.approx(old, abs=1e-12)


def test_controller_in_front_of_an_identity_lighthouse():
    """Origin = base station turned 180 deg about z: x and y flip, z stays."""
    lighthouse = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    controller = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    assert pose_in_origin(controller, lighthouse)[:3] == pytest.approx([-1.0, -2.0, 3.0])


def test_matrix_round_trip():
    pose = [0.1, -0.2, 0.3] + list(R.from_euler('xyz', [10, 20, 30], degrees=True).as_quat())
    back = matrix_to_pose(pose_to_matrix(pose))
    assert back[:3] == pytest.approx(pose[:3])
    assert abs(np.dot(back[3:], pose[3:])) == pytest.approx(1.0)
