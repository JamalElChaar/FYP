#!/usr/bin/env python3
"""Check Astra Pro USB detection and, optionally, live ROS 2 RGB-D topics."""

import argparse
from pathlib import Path
import time


ORBBEC_VENDOR_ID = "2bc5"
EXPECTED_PRODUCTS = {
    "0403": "Astra Pro depth camera",
    "0501": "Astra Pro HD color camera",
}


def read_text(path):
    """Read a small sysfs text file, returning an empty string if absent."""
    try:
        return path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return ""


def find_orbbec_usb_devices():
    """Return connected Orbbec devices reported by Linux sysfs."""
    devices = []
    usb_root = Path("/sys/bus/usb/devices")
    for device in usb_root.iterdir():
        if read_text(device / "idVendor").lower() != ORBBEC_VENDOR_ID:
            continue
        devices.append({
            "path": device,
            "product_id": read_text(device / "idProduct").lower(),
            "name": read_text(device / "product") or "Unknown Orbbec device",
        })
    return devices


def orbbec_video_nodes():
    """Find V4L2 nodes belonging to an Orbbec USB interface."""
    nodes = []
    video_root = Path("/sys/class/video4linux")
    if not video_root.exists():
        return nodes

    for entry in sorted(video_root.glob("video*")):
        device_path = (entry / "device").resolve()
        is_orbbec = any(
            read_text(parent / "idVendor").lower() == ORBBEC_VENDOR_ID
            for parent in (device_path, *device_path.parents)
        )
        if is_orbbec:
            nodes.append((Path("/dev") / entry.name, read_text(entry / "name")))
    return nodes


def print_usb_status():
    """Print the physical connection status and return whether depth is found."""
    devices = find_orbbec_usb_devices()
    if not devices:
        print("FAIL: no Orbbec USB device found (vendor 2bc5).")
        return False

    print("USB devices:")
    found_products = set()
    for device in devices:
        product_id = device["product_id"]
        found_products.add(product_id)
        expected_name = EXPECTED_PRODUCTS.get(product_id, "other Orbbec device")
        print(
            f"  PASS 2bc5:{product_id}  {device['name']}  "
            f"({expected_name})"
        )

    nodes = orbbec_video_nodes()
    if nodes:
        print("Linux color-camera nodes:")
        for path, name in nodes:
            availability = "accessible" if path.exists() else "listed in sysfs"
            print(f"  {path}: {name} [{availability}]")
    else:
        print("WARN: no Orbbec V4L2 color node found.")

    if "0403" not in found_products:
        print("FAIL: the Astra depth interface (2bc5:0403) is missing.")
        return False
    if "0501" not in found_products:
        print("WARN: the Astra Pro HD color interface (2bc5:0501) is missing.")
    return True


def check_ros_topics(timeout_seconds):
    """Wait for one color, depth, and camera-info message."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as exception:
        print(f"FAIL: ROS 2 Python libraries are unavailable: {exception}")
        print("Run: source /opt/ros/humble/setup.bash")
        return False

    topics = {
        "/camera/color/image_raw": Image,
        "/camera/depth/image_raw": Image,
        "/camera/color/camera_info": CameraInfo,
    }
    received = {topic: None for topic in topics}

    rclpy.init()
    node = Node("astra_pro_connection_test")
    for topic, message_type in topics.items():
        node.create_subscription(
            message_type,
            topic,
            lambda message, name=topic: received.__setitem__(name, message),
            qos_profile_sensor_data,
        )

    print(f"Waiting up to {timeout_seconds:.1f}s for ROS 2 camera messages...")
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline and not all(received.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    print("ROS topics:")
    for topic, message in received.items():
        if message is None:
            print(f"  FAIL {topic}: no message")
        elif hasattr(message, "encoding"):
            print(
                f"  PASS {topic}: {message.width}x{message.height} "
                f"{message.encoding}"
            )
        else:
            print(f"  PASS {topic}: {message.width}x{message.height} calibration")

    success = all(received.values())
    node.destroy_node()
    rclpy.shutdown()
    return success


def main():
    """Run connection checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ros",
        action="store_true",
        help="also require live RGB, depth, and camera-info ROS messages",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="seconds to wait when --ros is used (default: 8)",
    )
    args = parser.parse_args()

    usb_ok = print_usb_status()
    ros_ok = check_ros_topics(args.timeout) if args.ros and usb_ok else True
    print("\nRESULT:", "PASS" if usb_ok and ros_ok else "FAIL")
    raise SystemExit(0 if usb_ok and ros_ok else 1)


if __name__ == "__main__":
    main()
