"""Teleoperation logic of the teleop bridge, kept free of ROS so it can be unit-tested.

- The trigger is a deadman switch (clutch), with optional hysteresis.
- Translation is relative: the end effector moves by the controller's displacement since the
  clutch was engaged.
- Orientation is absolute by design: the end effector takes the controller's orientation,
  rotated by a fixed offset expressed in the controller's own frame.
"""
from scipy.spatial.transform import Rotation as R

TRACKPAD_AXES = ('trackpad_x', 'trackpad_y')


def parse_bool(value):
    """Accept a real bool or the 'true'/'false' strings older launch files pass."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def gate_button(key, value, trackpad_pressed, pressed_required):
    """Value to publish on one button topic.

    With pressed_required, the trackpad axes read 0.0 unless the pad is clicked down, so brushing
    it with a thumb does not move anything. The other buttons are never gated.
    """
    if pressed_required and key in TRACKPAD_AXES and trackpad_pressed < 0.5:
        return 0.0
    return float(value)


def is_stale(last_time, now, timeout):
    """True if the last sample (seconds) is older than timeout. timeout <= 0 disables the check."""
    if timeout <= 0:
        return False
    return last_time is None or now - last_time > timeout


def compute_target(ee_position, joy_reference, joy_position, joy_orientation, rotation_offset_deg):
    """Commanded end-effector pose.

    ee_position:      end-effector position when the clutch was engaged (x, y, z)
    joy_reference:    controller position when the clutch was engaged (x, y, z)
    joy_position:     current controller position (x, y, z)
    joy_orientation:  current controller orientation (x, y, z, w)
    rotation_offset_deg: [roll, pitch, yaw] applied in the controller's own frame

    Returns (position, orientation) as tuples.
    """
    position = tuple(ee + (now - ref) for ee, now, ref in zip(ee_position, joy_position, joy_reference))
    if not any(rotation_offset_deg):
        return position, tuple(joy_orientation)
    # Post-multiply (local frame rotation): the flip happens relative to the controller's own axes.
    rotation = R.from_quat(joy_orientation) * R.from_euler('xyz', rotation_offset_deg, degrees=True)
    return position, tuple(rotation.as_quat())


class Clutch:
    """Trigger deadman switch, with optional hysteresis.

    Engages when the trigger reaches engage_threshold and releases when it drops below
    release_threshold. The defaults (1.0 / 1.0) are the original behaviour: engage only at a full
    pull, release as soon as the trigger leaves 1.0. After force_release() it stays
    released until the trigger has been let go, so teleop never resumes by itself.
    """

    IDLE = 'idle'
    ENGAGED = 'engaged'
    LOCKED = 'locked'

    def __init__(self, engage_threshold=1.0, release_threshold=1.0):
        if release_threshold > engage_threshold:
            raise ValueError('release_threshold must not be above engage_threshold')
        self.engage_threshold = engage_threshold
        self.release_threshold = release_threshold
        self.state = self.IDLE

    @property
    def engaged(self):
        return self.state == self.ENGAGED

    def update(self, trigger):
        """Feed a trigger value. Returns 'engage', 'release' or None."""
        if self.state == self.ENGAGED:
            if trigger < self.release_threshold:
                self.state = self.IDLE
                return 'release'
            return None
        if self.state == self.LOCKED:
            if trigger < self.release_threshold:
                self.state = self.IDLE
            return None
        if trigger >= self.engage_threshold:
            self.state = self.ENGAGED
            return 'engage'
        return None

    def force_release(self):
        """Disengage and stay disengaged until the trigger is let go."""
        self.state = self.LOCKED
