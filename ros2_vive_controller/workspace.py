"""Workspace fence: classify controller positions and turn calibration points into bounds.

The fence is an axis-aligned box in the frame the driver publishes in. Inside it, a band
``padding`` wide along each face is a warning zone (the controller vibrates but the pose is still
published). Outside the box the controller vibrates and the pose is not published.
"""
import datetime
import os
import shutil

import numpy as np
import yaml

INSIDE = 'inside'
WARNING = 'warning'
OUTSIDE = 'outside'

AXES = ('x', 'y', 'z')


def classify(position, bounds, padding):
    """INSIDE, WARNING (within ``padding`` of a face) or OUTSIDE the box.

    bounds: dict with x_min, x_max, y_min, y_max, z_min, z_max.
    """
    for axis, p in zip(AXES, position):
        if p < bounds[f'{axis}_min'] or p > bounds[f'{axis}_max']:
            return OUTSIDE
    for axis, p in zip(AXES, position):
        if p < bounds[f'{axis}_min'] + padding or p > bounds[f'{axis}_max'] - padding:
            return WARNING
    return INSIDE


def bounds_from_points(points, margin=0.05, outlier_sigma=3.0):
    """Bounding box of the painted points, outliers removed, grown by ``margin`` on every face.

    Returns a bounds dict, or None if no point survives the outlier filter.
    """
    pts = np.asarray(points, dtype=float)
    mean = np.mean(pts, axis=0)
    std = np.std(pts, axis=0)
    clean = pts[(np.abs((pts - mean) / (std + 1e-9)) < outlier_sigma).all(axis=1)]
    if len(clean) == 0:
        return None
    low = np.min(clean, axis=0) - margin
    high = np.max(clean, axis=0) + margin
    bounds = {}
    for i, axis in enumerate(AXES):
        bounds[f'{axis}_min'] = float(round(low[i], 3))
        bounds[f'{axis}_max'] = float(round(high[i], 3))
    return bounds


def write_workspace_file(path, bounds, frame_id):
    """Write the calibrated fence as a ROS 2 parameter file, keeping a .bak of the previous one.

    Returns the path of the backup, or None if there was no previous file.
    """
    backup = None
    if os.path.exists(path):
        backup = path + '.bak'
        shutil.copy(path, backup)
    workspace = {'enabled': True, 'frame_id': frame_id}
    workspace.update(bounds)
    data = {'/**': {'ros__parameters': {'workspace': workspace}}}
    header = (
        f'# Workspace fence written by calibration_node on {datetime.datetime.now():%Y-%m-%d %H:%M}.\n'
        f'# Local to this setup (not tracked by git). Delete this file or set enabled: false to turn the fence off.\n'
    )
    with open(path, 'w') as f:
        f.write(header)
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return backup
