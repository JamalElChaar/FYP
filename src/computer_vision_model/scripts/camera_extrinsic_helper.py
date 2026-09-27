#!/usr/bin/env python3
"""Turn tape-measure readings into camera extrinsic launch arguments.

Standalone helper (not a ROS node). Measure four things, run this, and it
prints the cam_* arguments for fruit_localization.launch.py plus a check that
the fruit zone clears the Astra Pro dead zone.

    python3 camera_extrinsic_helper.py --distance 0.55 --bearing 150 \
        --height 0.12 --fruit-distance 0.25 --fruit-bearing -30
"""

import argparse
import itertools
import math

MIN_DEPTH = 0.60    # Astra Pro minimum depth range, metres
REACH = 0.40        # practical arm reach from base_link
HFOV, VFOV = 60.0, 49.5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, required=True,
                        help="M1: horizontal distance, base centre -> camera lens (m)")
    parser.add_argument("--bearing", type=float, required=True,
                        help="M2: angle from base_link +X to the camera, CCW (deg)")
    parser.add_argument("--height", type=float, required=True,
                        help="M3: camera lens height above the table (m)")
    parser.add_argument("--fruit-distance", type=float, default=0.25,
                        help="M4a: horizontal distance, base centre -> fruit zone (m)")
    parser.add_argument("--fruit-bearing", type=float, default=None,
                        help="M4b: angle from base_link +X to the fruit zone, CCW "
                             "(deg); default puts it opposite the camera")
    parser.add_argument("--fruit-height", type=float, default=0.05,
                        help="fruit height above the table (m)")
    args = parser.parse_args()

    # Camera position expressed in base_link.
    b = math.radians(args.bearing)
    cam = (args.distance * math.cos(b), args.distance * math.sin(b), args.height)

    # Fruit zone: opposite the camera by default, which maximises the
    # camera-to-fruit distance for a given camera position.
    fb = math.radians(args.fruit_bearing if args.fruit_bearing is not None
                      else args.bearing + 180.0)
    fruit = (args.fruit_distance * math.cos(fb),
             args.fruit_distance * math.sin(fb),
             args.fruit_height)

    # Aim the camera at the fruit zone.
    dx, dy, dz = (fruit[i] - cam[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    los = math.sqrt(dx * dx + dy * dy + dz * dz)

    print(f"\ncamera in base_link : x={cam[0]:+.3f}  y={cam[1]:+.3f}  z={cam[2]:+.3f}")
    print(f"fruit zone centre   : x={fruit[0]:+.3f}  y={fruit[1]:+.3f}  z={fruit[2]:+.3f}")
    print(f"  reach to fruit    : {math.dist(fruit, (0,0,0)):.3f} m "
          f"{'OK' if math.dist(fruit, (0,0,0)) <= REACH else '*** BEYOND ARM REACH ***'}")
    print(f"  camera to fruit   : {los:.3f} m "
          f"{'OK' if los >= MIN_DEPTH else '*** INSIDE DEAD ZONE ***'}")
    print(f"  viewing angle     : {abs(math.degrees(pitch)):.0f} deg above horizontal "
          f"{'*** GRAZING, raise the camera ***' if abs(math.degrees(pitch)) < 15 else 'OK'}")

    # How much of the reachable workspace is actually usable from here.
    volume = [p for p in itertools.product(
                  [i * 0.05 for i in range(-8, 9)],
                  [i * 0.05 for i in range(-8, 9)],
                  [i * 0.05 for i in range(0, 9)])
              if math.dist(p, (0, 0, 0)) <= REACH]
    cy, sy = math.cos(-yaw), math.sin(-yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    usable = 0
    for p in volume:
        v = [p[i] - cam[i] for i in range(3)]
        x1, y1 = v[0] * cy - v[1] * sy, v[0] * sy + v[1] * cy
        x2, y2, z2 = x1 * cp - v[2] * sp, y1, x1 * sp + v[2] * cp
        if (math.dist(p, cam) >= MIN_DEPTH and x2 > 0
                and abs(math.degrees(math.atan2(y2, x2))) < HFOV / 2
                and abs(math.degrees(math.atan2(z2, x2))) < VFOV / 2):
            usable += 1
    print(f"  usable workspace  : {100*usable/len(volume):.0f}% of reachable points\n")

    print("ros2 launch computer_vision_model fruit_localization.launch.py \\")
    print(f"  cam_x:={cam[0]:.3f} cam_y:={cam[1]:.3f} cam_z:={cam[2]:.3f} \\")
    print(f"  cam_roll:=0.0 cam_pitch:={pitch:.3f} cam_yaw:={yaw:.3f}\n")


if __name__ == "__main__":
    main()
