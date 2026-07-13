#!/usr/bin/env python3
"""Measure external-odometry header offset relative to XT16 frames.

This read-only helper pairs messages by local arrival time and reports:

    odom_time_offset_sec = xt16_header_stamp - odom_header_stamp

The result aligns clock stamps only. It does not prove that the odometry
content itself has no transport or estimator latency.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


@dataclass(frozen=True)
class TimedSample:
    arrival_sec: float
    header_stamp_sec: float


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class OffsetSampler(Node):
    def __init__(self, odom_topic: str, xt16_topic: str) -> None:
        super().__init__("go2_odom_xt16_time_offset_sampler")
        self.odom: list[TimedSample] = []
        self.xt16: list[TimedSample] = []
        self.last_odom_frames = ("", "")
        self.last_xt16_frame = ""
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, xt16_topic, self._xt16_callback, qos_profile_sensor_data
        )

    def _odom_callback(self, msg: Odometry) -> None:
        header_stamp_sec = stamp_to_sec(msg.header.stamp)
        if not math.isfinite(header_stamp_sec) or header_stamp_sec <= 0.0:
            return
        self.odom.append(
            TimedSample(time.monotonic(), header_stamp_sec)
        )
        self.last_odom_frames = (msg.header.frame_id, msg.child_frame_id)

    def _xt16_callback(self, msg: PointCloud2) -> None:
        header_stamp_sec = stamp_to_sec(msg.header.stamp)
        if not math.isfinite(header_stamp_sec) or header_stamp_sec <= 0.0:
            return
        self.xt16.append(
            TimedSample(time.monotonic(), header_stamp_sec)
        )
        self.last_xt16_frame = msg.header.frame_id


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure /utlidar/robot_odom versus /lidar_points header offset."
    )
    parser.add_argument("--odom-topic", default="/utlidar/robot_odom")
    parser.add_argument("--xt16-topic", default="/lidar_points")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--arrival-window", type=float, default=0.03)
    parser.add_argument("--stable-stdev", type=float, default=0.02)
    parser.add_argument("--stable-range", type=float, default=0.08)
    parser.add_argument("--min-pairs", type=int, default=20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0")
    if args.arrival_window <= 0.0:
        raise SystemExit("--arrival-window must be > 0")
    if args.stable_stdev < 0.0:
        raise SystemExit("--stable-stdev must be >= 0")
    if args.stable_range < 0.0:
        raise SystemExit("--stable-range must be >= 0")
    if args.min_pairs <= 0:
        raise SystemExit("--min-pairs must be > 0")

    rclpy.init()
    node = OffsetSampler(args.odom_topic, args.xt16_topic)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        pairs: list[tuple[float, float]] = []
        for xt16 in node.xt16:
            if not node.odom:
                break
            odom = min(
                node.odom,
                key=lambda sample: abs(sample.arrival_sec - xt16.arrival_sec),
            )
            arrival_delta = abs(odom.arrival_sec - xt16.arrival_sec)
            if arrival_delta <= args.arrival_window:
                pairs.append(
                    (xt16.header_stamp_sec - odom.header_stamp_sec, arrival_delta)
                )

        print(
            f"odom_topic={args.odom_topic} xt16_topic={args.xt16_topic} "
            f"duration={args.duration:.1f}s"
        )
        print(
            f"odom_count={len(node.odom)} xt16_count={len(node.xt16)} "
            f"paired_by_arrival={len(pairs)} required_pairs={args.min_pairs}"
        )
        print(
            f"odom_parent_frame={node.last_odom_frames[0]} "
            f"odom_child_frame={node.last_odom_frames[1]} "
            f"xt16_frame={node.last_xt16_frame}"
        )
        if len(pairs) < args.min_pairs:
            print("OFFSET_STABLE_FOR_MAPPING=False")
            print("recommended_odom_time_offset_sec=UNAVAILABLE")
            return 2

        offsets = [pair[0] for pair in pairs]
        arrival_deltas = [pair[1] for pair in pairs]
        offset_median = statistics.median(offsets)
        offset_stdev = statistics.pstdev(offsets)
        offset_range = max(offsets) - min(offsets)
        stable = (
            offset_stdev <= args.stable_stdev
            and offset_range <= args.stable_range
        )
        print(
            f"offset_sec_min={min(offsets):.6f} median={offset_median:.6f} "
            f"mean={statistics.mean(offsets):.6f} max={max(offsets):.6f}"
        )
        print(
            f"offset_sec_stdev={offset_stdev:.6f} "
            f"offset_sec_range={offset_range:.6f} "
            f"arrival_dt_mean={statistics.mean(arrival_deltas):.6f}"
        )
        print(f"OFFSET_STABLE_FOR_MAPPING={stable}")
        print(f"recommended_odom_time_offset_sec={offset_median:.6f}")
        print("CONTENT_LATENCY_VALIDATED=False")
        return 0 if stable else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
