"""Test setup: import this checkout and replace pyopenvr with an in-memory fake.

Run from the repository root (ROS 2 sourced for the rclpy-based tests):
    python3 -m pytest -q test
"""
import os
import sys

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TEST_DIR)
for path in (TEST_DIR, REPO_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

import fake_openvr  # noqa: E402

sys.modules['openvr'] = fake_openvr


@pytest.fixture
def vr_system():
    """A fresh fake SteamVR runtime with no devices."""
    return fake_openvr.reset()
