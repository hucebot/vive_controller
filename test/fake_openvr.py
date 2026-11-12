"""In-memory stand-in for the ``openvr`` (pyopenvr) module.

It implements just the IVRSystem calls used by this package, so the device registry and the
driver nodes can be tested without SteamVR or hardware. Tests drive it through the module-level
``SYSTEM`` (see :func:`reset`): plug/unplug devices, move them, press buttons, and read back
the haptic pulses that were requested.
"""
import collections
import types

k_unMaxTrackedDeviceCount = 64

VRApplication_Other = 4
TrackingUniverseStanding = 1

TrackedDeviceClass_Invalid = 0
TrackedDeviceClass_HMD = 1
TrackedDeviceClass_Controller = 2
TrackedDeviceClass_GenericTracker = 3
TrackedDeviceClass_TrackingReference = 4

Prop_ModelNumber_String = 1001
Prop_SerialNumber_String = 1002
Prop_DeviceBatteryPercentage_Float = 1012
Prop_DeviceIsCharging_Bool = 1013
Prop_ModeLabel_String = 2003

TrackingResult_Uninitialized = 1
TrackingResult_Running_OK = 200
TrackingResult_Running_OutOfRange = 201
TrackingResult_Fallback_RotationOnly = 300

VREvent_TrackedDeviceActivated = 100
VREvent_TrackedDeviceDeactivated = 101

# Short aliases for tests.
HMD = TrackedDeviceClass_HMD
CONTROLLER = TrackedDeviceClass_Controller
TRACKER = TrackedDeviceClass_GenericTracker
LIGHTHOUSE = TrackedDeviceClass_TrackingReference

# Bits used by vr_tracked_device.controller_state_to_dict
BUTTON_MENU = 1 << 1
BUTTON_GRIP = 1 << 2
BUTTON_TRACKPAD = 1 << 32


class TrackedPropertyError(Exception):
    pass


error_code = types.SimpleNamespace(TrackedPropertyError=TrackedPropertyError)


class VREvent_t:
    def __init__(self):
        self.eventType = 0
        self.trackedDeviceIndex = 0


class _Vector:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Pose:
    def __init__(self):
        self.bPoseIsValid = False
        self.bDeviceIsConnected = False
        self.eTrackingResult = TrackingResult_Uninitialized
        self.mDeviceToAbsoluteTracking = [[0.0] * 4 for _ in range(3)]
        self.vVelocity = _Vector()
        self.vAngularVelocity = _Vector()


class _ControllerState:
    def __init__(self):
        self.unPacketNum = 0
        self.rAxis = [_Vector() for _ in range(5)]
        self.ulButtonPressed = 0
        self.ulButtonTouched = 0


def translation_matrix(x=0.0, y=0.0, z=0.0):
    """3x4 OpenVR pose matrix with identity rotation."""
    return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z]]


class FakeDevice:
    def __init__(self, device_class, serial, model='Fake', position=(0.0, 0.0, 0.0), pose_valid=True):
        self.device_class = device_class
        self.serial = serial
        self.model = model
        self.matrix = translation_matrix(*position)
        self.pose_valid = pose_valid
        self.tracking_result = TrackingResult_Running_OK
        self.connected = True
        self.serial_readable = True
        self.mode_label = 'B'
        self.battery = 0.8
        self.charging = False
        # Controller inputs
        self.trigger = 0.0
        self.trackpad = (0.0, 0.0)
        self.pressed = 0
        self.touched = 0

    def move_to(self, x, y, z):
        self.matrix = translation_matrix(x, y, z)


class FakeVRSystem:
    def __init__(self):
        self.slots = {}
        self.events = collections.deque()
        self.haptic_pulses = []
        self.shutdowns = 0

    # --- helpers for tests -------------------------------------------------------------
    def plug(self, index, device, announce=True):
        """Put a device in a slot (announce=False: it was already there at start-up)."""
        device.connected = True
        self.slots[index] = device
        if announce:
            self.events.append((VREvent_TrackedDeviceActivated, index))
        return device

    def unplug(self, index, announce=True):
        """Switch the device in this slot off, as when its battery dies."""
        self.slots[index].connected = False
        if announce:
            self.events.append((VREvent_TrackedDeviceDeactivated, index))

    def replug(self, index):
        """Switch the device in this slot back on."""
        self.slots[index].connected = True
        self.events.append((VREvent_TrackedDeviceActivated, index))

    def announce(self, index):
        """Send an activation event for a device that is already registered."""
        self.events.append((VREvent_TrackedDeviceActivated, index))

    def pulses_for(self, index):
        return [p for p in self.haptic_pulses if p[0] == index]

    # --- IVRSystem subset ----------------------------------------------------------------
    def getDeviceToAbsoluteTrackingPose(self, origin, predicted_seconds, count):
        poses = []
        for i in range(count):
            pose = _Pose()
            device = self.slots.get(i)
            if device is not None:
                pose.bDeviceIsConnected = device.connected
                pose.bPoseIsValid = device.connected and device.pose_valid
                pose.mDeviceToAbsoluteTracking = [list(row) for row in device.matrix]
                if device.connected:
                    pose.eTrackingResult = device.tracking_result
            poses.append(pose)
        return poses

    def getTrackedDeviceClass(self, index):
        device = self.slots.get(index)
        return device.device_class if device is not None else TrackedDeviceClass_Invalid

    def getStringTrackedDeviceProperty(self, index, prop):
        device = self.slots.get(index)
        if device is None:
            raise TrackedPropertyError('InvalidDevice')
        if prop == Prop_SerialNumber_String:
            if not device.serial_readable:
                raise TrackedPropertyError('NotYetAvailable')
            return device.serial
        if prop == Prop_ModelNumber_String:
            return device.model
        if prop == Prop_ModeLabel_String and device.mode_label is not None:
            return device.mode_label
        raise TrackedPropertyError('UnknownProperty')

    def getFloatTrackedDeviceProperty(self, index, prop):
        return self.slots[index].battery

    def getBoolTrackedDeviceProperty(self, index, prop):
        return self.slots[index].charging

    def pollNextEvent(self, event):
        if not self.events:
            return False
        event.eventType, event.trackedDeviceIndex = self.events.popleft()
        return True

    def getControllerState(self, index):
        state = _ControllerState()
        device = self.slots.get(index)
        if device is None or not device.connected:
            return False, state
        state.rAxis[0].x, state.rAxis[0].y = device.trackpad
        state.rAxis[1].x = device.trigger
        state.ulButtonPressed = device.pressed
        state.ulButtonTouched = device.touched
        return True, state

    def triggerHapticPulse(self, index, axis_id, duration_micros):
        self.haptic_pulses.append((index, axis_id, duration_micros))


SYSTEM = FakeVRSystem()


def reset():
    """Start over with an empty runtime and return it."""
    global SYSTEM
    SYSTEM = FakeVRSystem()
    return SYSTEM


def init(application_type):
    return SYSTEM


def VRSystem():
    return SYSTEM


def shutdown():
    SYSTEM.shutdowns += 1
