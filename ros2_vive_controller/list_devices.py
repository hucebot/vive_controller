#!/usr/bin/env python3
from ros2_vive_controller.openvr_class.openvr_class import triad_openvr

def list_all_devices():
    try:
        vr = triad_openvr()
    except Exception as e:
        print(f"Failed to initialize OpenVR: {e}")
        return

    print("--- Connected Vive Devices ---")
    for name, device in vr.devices.items():
        serial = device.get_serial()
        device_class = device.device_class

        # Highlight lighthouses
        if device_class == "TrackingReference":
            print(f"🟢 LIGHTHOUSE  | Name: {name:<15} | Serial: {serial}")
        elif device_class == "Controller":
            print(f"🎮 CONTROLLER  | Name: {name:<15} | Serial: {serial}")
        else:
            print(f"⚪ OTHER       | Name: {name:<15} | Serial: {serial} (Class: {device_class})")

if __name__ == "__main__":
    list_all_devices()