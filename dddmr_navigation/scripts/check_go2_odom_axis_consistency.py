#!/usr/bin/env python3
"""Check whether odom displacement direction agrees with odom yaw.

This script is read-only. It subscribes to an odometry topic for a bounded
window and reports whether the observed displacement looks like forward motion
in the odom yaw direction.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class OdomSample:
    receipt_sec: float
    stamp_sec: float
    x: float
    y: float
    yaw: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Observe odometry and verify that displacement direction matches "
            "the reported yaw during a short forward probe."
        )
    )
    parser.add_argument("--odom-topic", default="/utlidar/robot_odom")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--min-forward", type=float, default=0.05)
    parser.add_argument("--max-heading-error-deg", type=float, default=20.0)
    parser.add_argument("--max-lateral-ratio", type=float, default=0.35)
    parser.add_argument("--min-samples", type=int, default=5)
    return parser.parse_args()


def shortest_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(msg) -> float:
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stamp_to_sec(msg) -> float:
    return float(msg.sec) + float(msg.nanosec) * 1e-9


class OdomCollector(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("check_go2_odom_axis_consistency")
        self.samples: list[OdomSample] = []
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.samples.append(
            OdomSample(
                receipt_sec=time.monotonic(),
                stamp_sec=stamp_to_sec(msg.header.stamp),
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=quaternion_to_yaw(msg.pose.pose.orientation),
            )
        )


def average_yaw(samples: list[OdomSample]) -> float:
    sin_sum = sum(math.sin(sample.yaw) for sample in samples)
    cos_sum = sum(math.cos(sample.yaw) for sample in samples)
    return math.atan2(sin_sum, cos_sum)


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0")

    rclpy.init()
    node = OdomCollector(args.odom_topic)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        samples = list(node.samples)
        node.destroy_node()
        rclpy.shutdown()

    print(f"ODOM_AXIS_TOPIC={args.odom_topic}")
    print(f"ODOM_AXIS_SAMPLES={len(samples)}")
    if len(samples) < args.min_samples:
        print(
            f"ODOM_AXIS_STATUS=FAIL reason=insufficient_samples "
            f"min_samples={args.min_samples}"
        )
        return 1

    start = samples[0]
    end = samples[-1]
    dx = end.x - start.x
    dy = end.y - start.y
    move_distance = math.hypot(dx, dy)
    move_dir = math.atan2(dy, dx) if move_distance > 0.0 else 0.0
    yaw_mid = average_yaw(samples)
    heading_error = shortest_angle(move_dir - yaw_mid)
    forward = dx * math.cos(yaw_mid) + dy * math.sin(yaw_mid)
    lateral = -dx * math.sin(yaw_mid) + dy * math.cos(yaw_mid)
    lateral_ratio = abs(lateral) / max(abs(forward), 1e-6)
    receipt_span = end.receipt_sec - start.receipt_sec
    stamp_span = end.stamp_sec - start.stamp_sec

    heading_ok = abs(math.degrees(heading_error)) < args.max_heading_error_deg
    forward_ok = forward > args.min_forward
    lateral_ok = lateral_ratio < args.max_lateral_ratio
    passed = heading_ok and forward_ok and lateral_ok

    print(f"ODOM_AXIS_RECEIPT_SPAN_SEC={receipt_span:.3f}")
    print(f"ODOM_AXIS_STAMP_SPAN_SEC={stamp_span:.3f}")
    print(f"ODOM_AXIS_START_X={start.x:.6f}")
    print(f"ODOM_AXIS_START_Y={start.y:.6f}")
    print(f"ODOM_AXIS_END_X={end.x:.6f}")
    print(f"ODOM_AXIS_END_Y={end.y:.6f}")
    print(f"ODOM_AXIS_DX={dx:.6f}")
    print(f"ODOM_AXIS_DY={dy:.6f}")
    print(f"ODOM_AXIS_MOVE_DISTANCE_M={move_distance:.6f}")
    print(f"ODOM_AXIS_MOVE_DIR_DEG={math.degrees(move_dir):.3f}")
    print(f"ODOM_AXIS_AVG_YAW_DEG={math.degrees(yaw_mid):.3f}")
    print(f"ODOM_AXIS_HEADING_ERROR_DEG={math.degrees(heading_error):.3f}")
    print(f"ODOM_AXIS_FORWARD_M={forward:.6f}")
    print(f"ODOM_AXIS_LATERAL_M={lateral:.6f}")
    print(f"ODOM_AXIS_LATERAL_RATIO={lateral_ratio:.6f}")
    print(f"ODOM_AXIS_FORWARD_OK={str(forward_ok).lower()}")
    print(f"ODOM_AXIS_HEADING_OK={str(heading_ok).lower()}")
    print(f"ODOM_AXIS_LATERAL_OK={str(lateral_ok).lower()}")
    print(f"ODOM_AXIS_STATUS={'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
