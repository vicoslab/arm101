#!/usr/bin/env python3
import pyudev
import subprocess
import os
import time

RULES_FILE = "/etc/udev/rules.d/99-custom.rules"


def get_device_attributes(dev):
    """
    Extract USB attributes (idVendor, idProduct, serial) for a device.
    Try the device first, then walk parents if needed.
    """
    def query_node(node):
        try:
            output = subprocess.check_output(
                ["udevadm", "info", "--query=all", f"--name={node}"],
                text=True,
            )
        except subprocess.CalledProcessError:
            return {}

        attrs = {}
        for line in output.splitlines():
            if "ID_VENDOR_ID=" in line:
                attrs["idVendor"] = line.split("=", 1)[1]
            elif "ID_MODEL_ID=" in line:
                attrs["idProduct"] = line.split("=", 1)[1]
            elif "ID_SERIAL_SHORT=" in line:
                attrs["serial"] = line.split("=", 1)[1]
        return attrs

    # 1️⃣ Try the device itself
    node = dev.device_node or dev.sys_path
    attrs = query_node(node)
    if "idVendor" in attrs and "idProduct" in attrs:
        return attrs

    # 2️⃣ If not found, walk up parent chain
    parent = dev.parent
    while parent is not None:
        node = parent.device_node or parent.sys_path
        attrs = query_node(node)
        if "idVendor" in attrs and "idProduct" in attrs:
            return attrs
        parent = parent.parent

    return {}


def append_udev_rule(attrs: dict, friendly_name: str):
    """Append a new udev rule for this device."""
    if not all(k in attrs for k in ("idVendor", "idProduct", "serial")):
        print("Error: missing required attributes, not writing rule.")
        return

    rule = (
        f'SUBSYSTEM=="tty|video4linux", ATTRS{{idVendor}}=="{attrs["idVendor"]}", '
        f'ATTRS{{idProduct}}=="{attrs["idProduct"]}", '
        f'ATTRS{{serial}}=="{attrs["serial"]}", SYMLINK+="{friendly_name}"\n'
    )

    try:
        with open(RULES_FILE, "a") as f:
            f.write(rule)
        print(f"\n✅ Rule added to {RULES_FILE}:\n{rule}")
    except PermissionError:
        print(f"Permission denied. Run as root or adjust permissions for {RULES_FILE}.")


def monitor_devices():
    """Monitor for /dev/tty* and /dev/video* device additions."""
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    # We’ll listen to both subsystems
    monitor.filter_by(subsystem="tty")
    monitor.filter_by(subsystem="video4linux")

    print("Monitoring for new /dev/tty* and /dev/video* devices... (Ctrl+C to exit)")

    for device in iter(monitor.poll, None):
        if device.action != "add" or not device.device_node:
            continue

        devnode = device.device_node
        print(f"\nDetected new device: {devnode}")

        confirm = input("Create a udev rule for this device? [y/N]: ").strip().lower()
        if confirm != "y":
            continue

        friendly_name = input("Enter friendly name for this device: ").strip()
        if not friendly_name:
            print("No name entered, skipping.")
            continue

        attrs = get_device_attributes(device)
        if not attrs:
            print("Unable to retrieve USB attributes.")
            continue

        append_udev_rule(attrs, friendly_name)

        print("\n--- Updated Rules File ---")
        os.system(f"cat {RULES_FILE}")
        print("\nMonitoring again...\n")


if __name__ == "__main__":
    try:
        monitor_devices()
    except KeyboardInterrupt:
        print("\nExiting cleanly.")
        time.sleep(0.5)
