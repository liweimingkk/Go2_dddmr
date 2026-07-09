#!/usr/bin/env python3
"""Check Go2 XT16 TF roll/pitch health without publishing motion commands."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


@dataclass(frozen=True)
class FrameCheck:
    key: str
    parent: str
    child: str
    max_roll_pitch_deg: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect base/lidar/odom/map transforms and fail if localization "
            "adds excessive roll or pitch."
        )
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--lidar-frame", default="hesai_lidar")
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--max-odom-base-roll-pitch-deg", type=float, default=5.0)
    parser.add_argument("--max-map-odom-roll-pitch-deg", type=float, default=3.0)
    parser.add_argument("--max-map-base-roll-pitch-deg", type=float, default=6.0)
    parser.add_argument(
        "--skip-lidar",
        action="store_true",
        help="Do not require base_link -> hesai_lidar; intended only for non-XT16 dry checks.",
    )
    return parser.parse_args()


def quaternion_to_rpy(quat) -> tuple[float, float, float]:
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wait_for_transforms(
    node: Node,
    buffer: Buffer,
    checks: list[FrameCheck],
    timeout_sec: float,
) -> tuple[dict[str, object], dict[str, str]]:
    remaining = {check.key: check for check in checks}
    transforms: dict[str, object] = {}
    errors: dict[str, str] = {}
    deadline = time.monotonic() + timeout_sec

    while remaining and time.monotonic() < deadline:
        for key, check in list(remaining.items()):
            try:
                transform = buffer.lookup_transform(check.parent, check.child, Time())
            except Exception as exc:  # tf2 has several runtime exception classes.
                errors[key] = str(exc)
                continue
            transforms[key] = transform
            remaining.pop(key)
            errors.pop(key, None)
        if remaining:
            rclpy.spin_once(node, timeout_sec=0.05)

    return transforms, {key: errors.get(key, "missing transform") for key in remaining}


def print_transform(check: FrameCheck, transform) -> bool:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    roll, pitch, yaw = quaternion_to_rpy(rotation)
    max_roll_pitch = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
    status = "PASS"
    if check.max_roll_pitch_deg is not None and max_roll_pitch > check.max_roll_pitch_deg:
        status = "FAIL"

    print(f"TF_HEALTH_{check.key}_PARENT={check.parent}")
    print(f"TF_HEALTH_{check.key}_CHILD={check.child}")
    print(
        f"TF_HEALTH_{check.key}_XYZ="
        f"{translation.x:.6f},{translation.y:.6f},{translation.z:.6f}"
    )
    print(f"TF_HEALTH_{check.key}_ROLL_DEG={math.degrees(roll):.3f}")
    print(f"TF_HEALTH_{check.key}_PITCH_DEG={math.degrees(pitch):.3f}")
    print(f"TF_HEALTH_{check.key}_YAW_DEG={math.degrees(yaw):.3f}")
    if check.max_roll_pitch_deg is not None:
        print(
            f"TF_HEALTH_{check.key}_MAX_ROLL_PITCH_DEG="
            f"{max_roll_pitch:.3f}"
        )
        print(
            f"TF_HEALTH_{check.key}_THRESHOLD_DEG="
            f"{check.max_roll_pitch_deg:.3f}"
        )
    print(f"TF_HEALTH_{check.key}_STATUS={status}")
    return status == "PASS"


def main() -> int:
    args = parse_args()
    if args.timeout_sec <= 0.0:
        raise SystemExit("--timeout-sec must be > 0")

    checks = [
        FrameCheck(
            "ODOM_BASE",
            args.odom_frame,
            args.base_frame,
            args.max_odom_base_roll_pitch_deg,
        ),
        FrameCheck(
            "MAP_ODOM",
            args.map_frame,
            args.odom_frame,
            args.max_map_odom_roll_pitch_deg,
        ),
        FrameCheck(
            "MAP_BASE",
            args.map_frame,
            args.base_frame,
            args.max_map_base_roll_pitch_deg,
        ),
    ]
    if not args.skip_lidar:
        checks.insert(0, FrameCheck("BASE_LIDAR", args.base_frame, args.lidar_frame, None))

    rclpy.init()
    node = Node("check_go2_xt16_tf_health")
    buffer = Buffer()
    TransformListener(buffer, node)
    try:
        transforms, errors = wait_for_transforms(node, buffer, checks, args.timeout_sec)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    passed = True
    by_key = {check.key: check for check in checks}
    for check in checks:
        transform = transforms.get(check.key)
        if transform is None:
            print(f"TF_HEALTH_{check.key}_PARENT={check.parent}")
            print(f"TF_HEALTH_{check.key}_CHILD={check.child}")
            print(f"TF_HEALTH_{check.key}_STATUS=FAIL")
            print(f"TF_HEALTH_{check.key}_ERROR={errors.get(check.key, 'missing transform')}")
            passed = False
            continue
        passed = print_transform(by_key[check.key], transform) and passed

    print(f"TF_HEALTH_STATUS={'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
