#!/usr/bin/env python3
"""Approach, stab, retract and go home, as one motion sequence.

Publishes complete six-joint servo commands straight to /esp32/joint_commands.
Unlike direct_esp32_moveit_node's sequential_final mode, which commands one
joint per second and leaves the fork wandering off vertical in between, every
joint here moves together -- which is what makes the descent an actual stab
rather than a six-second shuffle.

The poses are precomputed and were checked for fork-vertical geometry, servo
range and joint limits. NOTHING IS PLANNED OR COLLISION-CHECKED AT RUNTIME:
this drives the servos directly, so the arm goes exactly where these numbers
say, with no safety net. Keep the servo power cutoff to hand.

    ros2 run control_arm stab_sequence            # full sequence
    ros2 run control_arm stab_sequence --dry-run  # print, publish nothing

Each run writes a timestamped directory under src/control_arm/logs/ holding
phases.txt, in the same format the vision pipeline uses, plus poses.json with
the exact servo values commanded. The ESP32 echoes commands rather than
measuring them, so the log records what was SENT; it is not evidence that the
arm physically arrived.
"""

import datetime
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Fork held vertical at the banana's XY throughout; seeded from the hover pose
# so joint_1 barely moves and joint_4 keeps ~2.5 deg of margin off its stop.
POSES = [
    ("APPROACH", "hover, tip at table level per the model",
     [115.21, 138.74, 56.99, 2.81, 96.73, 26.17], 2.5),
    ("DESCEND", "3 cm below hover",
     [115.22, 140.78, 45.68, 2.69, 83.07, 30.02], 1.0),
    ("STAB", "6 cm below hover; expected table contact",
     [115.31, 146.30, 40.75, 2.55, 72.07, 37.26], 1.5),
    ("RETRACT", "back to the hover pose",
     [115.21, 138.74, 56.99, 2.81, 96.73, 26.17], 1.5),
    ("HOME", "servo 90 on every joint, the firmware boot pose",
     [90.0, 90.0, 90.0, 90.0, 90.0, 90.0], 1.0),
]

SERVO_MIN, SERVO_MAX = 0.0, 180.0
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


def phase_log(path, phase, status, detail, payload=None):
    """Append one record, matching vision_pipeline_support.phase_log."""
    if not path:
        return
    record = (f"\n[{datetime.datetime.now().astimezone().isoformat()}] "
              f"{phase} — {status}\n{detail}\n")
    if payload is not None:
        record += json.dumps(payload, indent=2, allow_nan=False) + "\n"
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        os.write(fd, record.encode("utf-8"))
    finally:
        os.close(fd)


def _resolve_log_dir():
    """src/control_arm/logs, whether run from source or a symlinked install."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if parent.name == "control_arm" and (parent / "launch").is_dir():
            return parent / "logs"
    return here.parent / "logs"


def main():
    dry_run = "--dry-run" in sys.argv

    run_dir = _resolve_log_dir() / datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir.mkdir(parents=True, exist_ok=True)
    phases = run_dir / "phases.txt"
    phases.write_text(
        "ARM MOTION SEQUENCE PHASE REPORT\n"
        "Servo angles are degrees (0-180) as sent to /esp32/joint_commands.\n"
        "The ESP32 echoes commands rather than measuring them, so these are\n"
        "the values SENT, not confirmation the arm reached them.\n")
    print(f"Run log: {run_dir}")

    phase_log(phases, "CONFIGURATION", "READY",
              "Direct servo commands; no MoveIt, no planning, no collision checking.",
              {"dry_run": dry_run, "joint_names": JOINT_NAMES,
               "servo_range_deg": [SERVO_MIN, SERVO_MAX],
               "poses": [{"phase": p, "detail": d, "servo_degrees": a,
                          "settle_seconds": s} for p, d, a, s in POSES]})

    for phase, detail, angles, _ in POSES:
        bad = [JOINT_NAMES[i] for i, a in enumerate(angles)
               if not SERVO_MIN <= a <= SERVO_MAX]
        if bad:
            phase_log(phases, phase, "ERROR",
                      f"Refusing to run: joints outside the servo range: {bad}")
            print(f"refusing to run: {phase} has out-of-range joints {bad}")
            return 1

    rclpy.init()
    node = Node("stab_sequence")
    publisher = node.create_publisher(Float64MultiArray, "/esp32/joint_commands", 10)

    if not dry_run:
        # Nothing subscribes on the mock/simulated stack, so a missing
        # subscriber almost always means the ESP32 link is down again.
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            phase_log(phases, "ESP32", "ERROR",
                      "No subscriber on /esp32/joint_commands after 10 s; nothing was sent.")
            print("no subscriber on /esp32/joint_commands after 10 s.")
            print("The ESP32 is probably not connected; run with --dry-run to")
            print("check the sequence without hardware.")
            node.destroy_node(); rclpy.shutdown(); return 1
        phase_log(phases, "ESP32", "SUCCESS",
                  "Subscriber present on /esp32/joint_commands.")

    sent = []
    print(f"{'step':<48} servo angles")
    for phase, detail, angles, settle in POSES:
        print(f"  {phase:<12} {detail:<34} {', '.join(f'{a:6.2f}' for a in angles)}")
        if dry_run:
            phase_log(phases, phase, "DRY_RUN", detail,
                      dict(zip(JOINT_NAMES, angles)))
            continue
        publisher.publish(Float64MultiArray(data=[float(a) for a in angles]))
        # Republish once: a single UDP datagram to the ESP32 can be lost.
        rclpy.spin_once(node, timeout_sec=0.05)
        publisher.publish(Float64MultiArray(data=[float(a) for a in angles]))
        phase_log(phases, phase, "SENT", detail,
                  {"servo_degrees": dict(zip(JOINT_NAMES, angles)),
                   "settle_seconds": settle})
        sent.append({"phase": phase, "detail": detail,
                     "servo_degrees": dict(zip(JOINT_NAMES, angles)),
                     "sent_at": datetime.datetime.now().astimezone().isoformat()})
        time.sleep(settle)

    (run_dir / "poses.json").write_text(
        json.dumps({"dry_run": dry_run, "poses": sent}, indent=2) + "\n")

    if dry_run:
        phase_log(phases, "SEQUENCE", "DRY_RUN", "Nothing was published.")
        print("\ndry run: nothing published")
    else:
        phase_log(phases, "SEQUENCE", "COMPLETE",
                  "All poses sent; arm commanded back to the home pose.")
        print("\nsequence complete; arm returned home")
    print(f"Log: {phases}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
