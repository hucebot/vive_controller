"""Simulated SteamVR for `--demo`: try the setup app without hardware.

Two controllers sweep in front of two base stations (SteamVR's world frame is y-up) and pull their
trigger in turn, so assigning a hand by trigger and painting a workspace can be tried.
"""
import math
import sys

RIGHT, LEFT = 'LHR-DEMO-RIGHT', 'LHR-DEMO-LEFT'


def install():
    """Replace openvr with the in-memory fake and fill it. Call before creating triad_openvr."""
    from ros2_vive_controller.openvr_class import openvr_class
    from ros2_vive_controller.testing import fake_openvr
    sys.modules['openvr'] = fake_openvr
    openvr_class.openvr = fake_openvr       # already imported with the real module
    system = fake_openvr.reset()
    origin = fake_openvr.FakeDevice(fake_openvr.LIGHTHOUSE, 'LHB-DEMO-B', model='Valve SR Imp',
                                    position=(0.0, 2.2, -1.5))
    other = fake_openvr.FakeDevice(fake_openvr.LIGHTHOUSE, 'LHB-DEMO-C', model='Valve SR Imp',
                                   position=(2.5, 2.2, 1.5))
    other.mode_label = 'C'
    right = fake_openvr.FakeDevice(fake_openvr.CONTROLLER, RIGHT, model='VIVE Controller Pro MV')
    left = fake_openvr.FakeDevice(fake_openvr.CONTROLLER, LEFT, model='VIVE Controller Pro MV')
    left.battery = 0.35
    for slot, device in enumerate((origin, other, right, left), start=1):
        system.plug(slot, device, announce=False)
    return Demo(right, left)


class Demo:
    def __init__(self, right, left):
        self.right = right
        self.left = left
        self.start = None

    def tick(self, now):
        """Move the controllers and pull their triggers (called by SetupApp on every step)."""
        if self.start is None:
            self.start = now
        t = now - self.start
        for device, centre, phase in ((self.right, (0.3, 1.0, 0.0), 0.0), (self.left, (-0.3, 1.0, 0.0), 4.0)):
            s = t + phase
            device.move_to(centre[0] + 0.4 * math.sin(0.9 * s),
                           centre[1] + 0.3 * math.sin(1.7 * s),
                           centre[2] + 0.35 * math.sin(1.1 * s + 1.0))
            # Trigger held for 3 s every 8 s, each controller in turn.
            device.trigger = 1.0 if (s % 8.0) < 3.0 else 0.0
