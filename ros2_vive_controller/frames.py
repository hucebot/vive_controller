"""Frames the Vive driver publishes poses in.

Without a reference base station, poses are in SteamVR's standing universe (``vive_world`` by
default). With one, they are expressed relative to that base station, turned 180 degrees about its
z axis, in the ``lighthouse_origin`` frame.

The driver and the setup app both use these functions, so a fence painted in the app is in the
frame the driver checks it in.

Poses are [x, y, z, qx, qy, qz, qw] lists.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

LIGHTHOUSE_FRAME = 'lighthouse_origin'
WORLD_FRAME = 'vive_world'


def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    T[:3, :3] = R.from_quat(pose[3:]).as_matrix()
    return T


def matrix_to_pose(T):
    pos = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()
    return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]


def rotate_about_local_axis(pose, axis='z', angle_degrees=180):
    """The pose turned about one of its own axes (used to align the base station frame)."""
    T_rot = np.eye(4)
    T_rot[:3, :3] = R.from_euler(axis, angle_degrees, degrees=True).as_matrix()
    return matrix_to_pose(pose_to_matrix(pose) @ T_rot)


def relative_pose(target_pose, reference_pose):
    """target_pose expressed in the frame of reference_pose."""
    return matrix_to_pose(np.linalg.inv(pose_to_matrix(reference_pose)) @ pose_to_matrix(target_pose))


def origin_from_lighthouse(lighthouse_pose):
    """Pose of the lighthouse_origin frame in the SteamVR world, given the base station's pose."""
    return rotate_about_local_axis(lighthouse_pose, axis='z', angle_degrees=180)


def pose_in_origin(pose, lighthouse_pose):
    """A SteamVR world pose expressed in the lighthouse_origin frame."""
    return relative_pose(pose, origin_from_lighthouse(lighthouse_pose))
