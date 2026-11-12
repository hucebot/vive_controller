"""Device registry: devices that are switched off and on again must keep their identity."""
from fake_openvr import CONTROLLER, LIGHTHOUSE, TRACKER, FakeDevice, translation_matrix

from ros2_vive_controller.openvr_class.openvr_class import convert_to_quaternion, triad_openvr


def _start(vr_system, *devices):
    """Devices already connected when the node starts: (slot, class, serial)."""
    for index, device_class, serial in devices:
        vr_system.plug(index, FakeDevice(device_class, serial), announce=False)
    return triad_openvr()


def _two_controllers_and_lighthouse(vr_system):
    return _start(vr_system,
                  (1, CONTROLLER, 'LHR-LEFT'),
                  (2, CONTROLLER, 'LHR-RIGHT'),
                  (3, LIGHTHOUSE, 'LHB-ORIGIN'))


def _name_of(vr, serial):
    """The lookup the driver nodes did before: scan the registry for a serial."""
    for name, device in vr.devices.items():
        if device.get_serial() == serial:
            return name
    return None


def test_startup_registers_every_connected_device(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    assert sorted(vr.object_names['Controller']) == ['controller_1', 'controller_2']
    assert vr.object_names['Tracking Reference'] == ['tracking_reference_1']
    assert vr.get_device_by_serial('LHR-LEFT').index == 1
    assert vr.get_device_by_serial('LHR-RIGHT').index == 2
    assert vr.get_device_by_serial('LHB-ORIGIN').index == 3


def test_power_cycle_does_not_give_one_controller_the_others_name(vr_system):
    """Regression: the returning controller used to be named controller_<count+1>, which was the
    name of the controller still connected, so it replaced that one in the registry and the
    other side's driver started publishing the wrong controller."""
    vr = _two_controllers_and_lighthouse(vr_system)
    right_name = _name_of(vr, 'LHR-RIGHT')

    vr_system.unplug(1)          # left controller switched off...
    vr.poll_vr_events()
    vr_system.replug(1)          # ...and on again
    vr.poll_vr_events()

    assert vr.devices[right_name].get_serial() == 'LHR-RIGHT'
    assert _name_of(vr, 'LHR-LEFT') is not None
    assert len(vr.object_names['Controller']) == 2
    assert len(set(vr.object_names['Controller'])) == 2


def test_power_cycle_of_either_side_keeps_both_lookups_right(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    for index in (1, 2, 1, 2):
        vr_system.unplug(index)
        vr.poll_vr_events()
        vr_system.replug(index)
        vr.poll_vr_events()
        assert vr.get_device_by_serial('LHR-LEFT').index == 1
        assert vr.get_device_by_serial('LHR-RIGHT').index == 2


def test_disconnected_device_is_not_returned_until_it_comes_back(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    vr_system.unplug(2)
    vr.poll_vr_events()
    assert vr.get_device_by_serial('LHR-RIGHT') is None
    assert vr.get_device_by_serial('LHR-LEFT').index == 1

    vr_system.replug(2)
    vr.poll_vr_events()
    assert vr.get_device_by_serial('LHR-RIGHT').index == 2


def test_returning_device_gets_its_old_name_back(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    left_name = _name_of(vr, 'LHR-LEFT')
    vr_system.unplug(1)
    vr.poll_vr_events()
    assert left_name not in vr.devices
    vr_system.replug(1)
    vr.poll_vr_events()
    assert vr.get_device_name_by_serial('LHR-LEFT') == left_name


def test_repeated_activation_event_does_not_duplicate_a_device(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    vr_system.announce(1)
    vr_system.announce(2)
    vr.poll_vr_events()
    assert len(vr.object_names['Controller']) == 2
    assert len(vr.devices) == 3


def test_device_coming_back_in_another_slot_replaces_its_stale_entry(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    left = vr_system.slots[1]
    vr_system.slots.pop(1)       # no deactivation event was delivered
    vr_system.plug(7, left)
    vr.poll_vr_events()
    assert vr.get_device_by_serial('LHR-LEFT').index == 7
    assert 1 not in vr.device_index_map
    assert len(vr.object_names['Controller']) == 2


def test_slot_reused_by_another_device_replaces_the_old_one(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    vr_system.plug(1, FakeDevice(TRACKER, 'LHR-TRACKER'))
    vr.poll_vr_events()
    assert vr.get_device_by_serial('LHR-LEFT') is None
    assert vr.get_device_by_serial('LHR-TRACKER').index == 1
    assert vr.object_names['Controller'] == [_name_of(vr, 'LHR-RIGHT')]
    assert vr.object_names['Tracker'] == [_name_of(vr, 'LHR-TRACKER')]


def test_serial_unreadable_at_registration_is_found_later(vr_system):
    vr = _start(vr_system, (2, CONTROLLER, 'LHR-RIGHT'))
    late = FakeDevice(CONTROLLER, 'LHR-LATE')
    late.serial_readable = False
    vr_system.plug(4, late)
    vr.poll_vr_events()
    assert vr.get_device_by_serial('LHR-LATE') is None
    late.serial_readable = True
    assert vr.get_device_by_serial('LHR-LATE').index == 4


def test_unsupported_device_class_is_ignored(vr_system):
    vr = _start(vr_system, (2, CONTROLLER, 'LHR-RIGHT'))
    vr_system.plug(9, FakeDevice(0, 'INVALID'))
    vr.poll_vr_events()
    assert len(vr.devices) == 1


def test_empty_serial_never_matches(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    assert vr.get_device_by_serial('') is None
    assert vr.get_device_by_serial(None) is None


def test_lighthouse_pose_flagged_invalid_is_available_on_request(vr_system):
    """Lighthouse 2.0 base stations report bPoseIsValid=False with a calibrated matrix."""
    lighthouse = FakeDevice(LIGHTHOUSE, 'LHB-2', position=(1.0, 2.0, 3.0), pose_valid=False)
    vr_system.plug(0, lighthouse, announce=False)
    device = triad_openvr().get_device_by_serial('LHB-2')

    assert device.get_pose_quaternion() is None
    assert device.get_pose_quaternion(require_valid=False) == convert_to_quaternion(translation_matrix(1.0, 2.0, 3.0))

    lighthouse.matrix = [[0.0] * 4 for _ in range(3)]   # never filled in by SteamVR
    assert device.get_pose_quaternion(require_valid=False) is None


def test_print_discovered_objects_survives_missing_properties(vr_system, capsys):
    vr = _two_controllers_and_lighthouse(vr_system)
    vr_system.slots[3].mode_label = None   # property not available
    vr.print_discovered_objects()
    out = capsys.readouterr().out
    assert 'Found 2 Controllers' in out
    assert 'LHB-ORIGIN, Mode unknown' in out


def test_rename_keeps_lookups_consistent(vr_system):
    vr = _two_controllers_and_lighthouse(vr_system)
    old = _name_of(vr, 'LHB-ORIGIN')
    vr.rename_device(old, 'origin')
    assert vr.get_device_name_by_serial('LHB-ORIGIN') == 'origin'
    vr_system.unplug(3)
    vr.poll_vr_events()
    assert 'origin' not in vr.devices
    vr_system.replug(3)
    vr.poll_vr_events()
    assert vr.get_device_name_by_serial('LHB-ORIGIN') == 'origin'


def test_sync_registers_a_device_that_came_back_without_an_event(vr_system):
    """Seen in the lab: a USB controller switched off and on was not announced again."""
    vr = _two_controllers_and_lighthouse(vr_system)
    vr_system.unplug(1)
    vr.poll_vr_events()
    vr_system.slots[1].connected = True          # back, but no activation event
    vr.poll_vr_events()
    assert vr.get_device_by_serial('LHR-LEFT') is None
    assert vr.sync_connected_devices() == [_name_of(vr, 'LHR-LEFT')]
    assert vr.get_device_by_serial('LHR-LEFT').index == 1
    assert vr.sync_connected_devices() == []     # nothing else to add
