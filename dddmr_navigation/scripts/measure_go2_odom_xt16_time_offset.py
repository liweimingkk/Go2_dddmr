#!/usr/bin/env python3
"""Measure external-odometry header offset relative to XT16 frames.

This read-only helper pairs messages one-to-one by local arrival time, removes
the signed sampling phase, and reports:

    odom_time_offset_sec = (xt16_header_stamp - odom_header_stamp)
                           - (xt16_receipt_time - odom_receipt_time)

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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

from go2_time_sync_utils import clock_phase_diagnostics, ordered_pair_indices


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True)
class TimedSample:
    arrival_ns: int
    header_stamp_ns: int


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class OffsetSampler(Node):
    def __init__(self, odom_topic: str, xt16_topic: str) -> None:
        super().__init__("go2_odom_xt16_time_offset_sampler")
        self.odom: list[TimedSample] = []
        self.xt16: list[TimedSample] = []
        self.last_odom_frames = ("", "")
        self.last_xt16_frame = ""
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, SENSOR_QOS
        )
        self.create_subscription(
            PointCloud2, xt16_topic, self._xt16_callback, SENSOR_QOS
        )

    def _odom_callback(self, msg: Odometry) -> None:
        header_stamp_ns = stamp_to_ns(msg.header.stamp)
        if header_stamp_ns <= 0:
            return
        self.odom.append(
            TimedSample(time.monotonic_ns(), header_stamp_ns)
        )
        self.last_odom_frames = (msg.header.frame_id, msg.child_frame_id)

    def _xt16_callback(self, msg: PointCloud2) -> None:
        header_stamp_ns = stamp_to_ns(msg.header.stamp)
        if header_stamp_ns <= 0:
            return
        self.xt16.append(
            TimedSample(time.monotonic_ns(), header_stamp_ns)
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
    parser.add_argument("--max-clock-step", type=float, default=0.25)
    return parser.parse_args()


def pair_by_arrival(
    xt16: list[TimedSample], odom: list[TimedSample], arrival_window_ns: int
) -> list[tuple[int, int, int]]:
    """Pair samples once, preserving order and maximizing pair count."""
    ordered_xt16 = sorted(xt16, key=lambda sample: sample.arrival_ns)
    ordered_odom = sorted(odom, key=lambda sample: sample.arrival_ns)
    indices = ordered_pair_indices(
        [sample.arrival_ns for sample in ordered_xt16],
        [sample.arrival_ns for sample in ordered_odom],
        arrival_window_ns,
    )
    pairs: list[tuple[int, int, int]] = []
    for xt_index, odom_index in indices:
        signed_arrival_delta_ns = (
            ordered_xt16[xt_index].arrival_ns
            - ordered_odom[odom_index].arrival_ns
        )
        raw_header_offset_ns = (
            ordered_xt16[xt_index].header_stamp_ns
            - ordered_odom[odom_index].header_stamp_ns
        )
        corrected_offset_ns = raw_header_offset_ns - signed_arrival_delta_ns
        pairs.append(
            (corrected_offset_ns, signed_arrival_delta_ns, raw_header_offset_ns)
        )
    return pairs


def main() -> int:
    args = parse_args()
    numeric_args = (
        args.duration,
        args.arrival_window,
        args.stable_stdev,
        args.stable_range,
        args.max_clock_step,
    )
    if not all(math.isfinite(value) for value in numeric_args):
        raise SystemExit("numeric thresholds must be finite")
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
    if args.max_clock_step < 0.0:
        raise SystemExit("--max-clock-step must be >= 0")

    rclpy.init()
    node = OffsetSampler(args.odom_topic, args.xt16_topic)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        pairs = pair_by_arrival(
            node.xt16, node.odom, int(round(args.arrival_window * 1e9))
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

        offsets = [pair[0] * 1e-9 for pair in pairs]
        signed_arrival_deltas = [pair[1] * 1e-9 for pair in pairs]
        raw_header_offsets = [pair[2] * 1e-9 for pair in pairs]
        offset_median = statistics.median(offsets)
        offset_stdev = statistics.pstdev(offsets)
        offset_range = max(offsets) - min(offsets)
        max_clock_step_ns = int(round(args.max_clock_step * 1e9))
        odom_clock_step_ns, odom_clock_span_ns = clock_phase_diagnostics(
            [sample.arrival_ns for sample in node.odom],
            [sample.header_stamp_ns for sample in node.odom],
        )
        xt16_clock_step_ns, xt16_clock_span_ns = clock_phase_diagnostics(
            [sample.arrival_ns for sample in node.xt16],
            [sample.header_stamp_ns for sample in node.xt16],
        )
        clock_steps_ok = max(
            odom_clock_step_ns,
            odom_clock_span_ns,
            xt16_clock_step_ns,
            xt16_clock_span_ns,
        ) <= max_clock_step_ns
        stable = (
            offset_stdev <= args.stable_stdev
            and offset_range <= args.stable_range
            and clock_steps_ok
        )
        print(
            f"offset_sec_min={min(offsets):.6f} median={offset_median:.6f} "
            f"mean={statistics.mean(offsets):.6f} max={max(offsets):.6f}"
        )
        print(
            f"offset_sec_stdev={offset_stdev:.6f} "
            f"offset_sec_range={offset_range:.6f} "
            f"signed_arrival_dt_mean={statistics.mean(signed_arrival_deltas):.6f}"
        )
        print(
            f"raw_header_offset_sec_median={statistics.median(raw_header_offsets):.6f} "
            f"clock_steps_ok={clock_steps_ok} "
            f"odom_clock_phase_span_sec={odom_clock_span_ns * 1e-9:.6f} "
            f"xt16_clock_phase_span_sec={xt16_clock_span_ns * 1e-9:.6f}"
        )
        print(f"OFFSET_STABLE_FOR_MAPPING={stable}")
        recommendation = f"{offset_median:.6f}" if stable else "UNAVAILABLE"
        print(f"recommended_odom_time_offset_sec={recommendation}")
        print("CONTENT_LATENCY_VALIDATED=False")
        return 0 if stable else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
