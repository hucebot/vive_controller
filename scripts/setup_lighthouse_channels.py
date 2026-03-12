#!/usr/bin/env python3
"""
Set Base Station 2.0 channels via Bluetooth LE.

Each lighthouse in the same tracking space must be on a unique channel (1-16).
Channels are written to BLE characteristic 00001524 and persist in the lighthouse firmware.

Usage:
    python3 setup_lighthouse_channels.py LHB-AF7A4E00 LHB-C335623F

This assigns channel 1 to the first lighthouse, channel 2 to the second, etc.
Run on the HOST (not in Docker) — requires system Bluetooth.

Dependencies: pip install bleak
"""
import asyncio
import sys
from bleak import BleakClient, BleakScanner

CHANNEL_CHAR = "00001524-1212-efde-1523-785feabcd124"
POWER_CHAR = "00001525-1212-efde-1523-785feabcd124"


async def find_lighthouse(name):
    """Scan BLE for a lighthouse by its LHB-xxx name, return its address."""
    print(f"Scanning for {name}...")
    devices = await BleakScanner.discover(timeout=10)
    for d in devices:
        if d.name == name:
            return d.address
    return None


async def set_channel(addr, name, channel):
    """Set the IR channel on a Base Station 2.0 via BLE."""
    async with BleakClient(addr, timeout=15) as client:
        # Read current state
        power_data = await client.read_gatt_char(POWER_CHAR)
        chan_data = await client.read_gatt_char(CHANNEL_CHAR)
        print(
            f"  {name}: power=0x{power_data[0]:02x}, channel_char=0x{chan_data[0]:02x}"
        )

        # Ensure lighthouse is on
        if power_data[0] == 0x00:
            print(f"  {name}: waking up...")
            await client.write_gatt_char(POWER_CHAR, bytearray([0x01]))
            await asyncio.sleep(5)

        # Set channel via the 0x1524 characteristic
        print(f"  {name}: setting channel {channel}...")
        await client.write_gatt_char(CHANNEL_CHAR, bytearray([channel]))
        await asyncio.sleep(5)

        # Verify
        chan_data = await client.read_gatt_char(CHANNEL_CHAR)
        power_data = await client.read_gatt_char(POWER_CHAR)
        print(
            f"  {name}: done. power=0x{power_data[0]:02x}, channel_char=0x{chan_data[0]:02x}"
        )


async def main(lighthouse_names):
    if len(lighthouse_names) > 16:
        print("Error: maximum 16 channels supported")
        sys.exit(1)

    # Discover all requested lighthouses
    lighthouses = []
    for name in lighthouse_names:
        addr = await find_lighthouse(name)
        if addr is None:
            print(f"Error: could not find {name} via Bluetooth. Is it powered on?")
            sys.exit(1)
        print(f"Found {name} at {addr}")
        lighthouses.append((name, addr))

    # Set channels (1-based)
    for i, (name, addr) in enumerate(lighthouses):
        channel = i + 1
        print(f"\nConfiguring {name} -> channel {channel}")
        await set_channel(addr, name, channel)

    print("\nAll lighthouses configured:")
    for i, (name, _) in enumerate(lighthouses):
        print(f"  {name} -> channel {i + 1}")
    print("\nRestart vrserver to pick up the changes.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} LHB-SERIAL1 LHB-SERIAL2 [LHB-SERIAL3 ...]")
        print(f"Example: {sys.argv[0]} LHB-AF7A4E00 LHB-C335623F")
        sys.exit(1)
    asyncio.run(main(sys.argv[1:]))
