#!/usr/bin/env python3
"""Measure mouth LiDAR header timestamp offset relative to XT16.

This is a read-only helper. It subscribes to the two PointCloud2 topics,
pairs frames one-to-one by arrival time, removes the signed sampling phase,
and reports:

    offset = (xt16_header_stamp - mouth_header_stamp)
             - (xt16_receipt_time - mouth_receipt_time)

Use the median offset as `mouth_time_offset_sec` only if the sample is stable in
the current live graph.
"""

import argparse
import math
import statistics
import time

import rclpy
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


class OffsetSampler(Node):
    def __init__(self, mouth_topic, xt16_topic):
        super().__init__("go2_mouth_xt16_time_offset_sampler")
        self.mouth = []
        self.xt16 = []
        self.create_subscription(
            PointCloud2, mouth_topic, self._mouth_cb, SENSOR_QOS
        )
        self.create_subscription(
            PointCloud2, xt16_topic, self._xt16_cb, SENSOR_QOS
        )

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _mouth_cb(self, msg):
        stamp_ns = self._stamp(msg)
        if stamp_ns <= 0:
            return
        self.mouth.append(
            (time.monotonic_ns(), stamp_ns, msg.header.frame_id, msg.width, msg.height)
        )

    def _xt16_cb(self, msg):
        stamp_ns = self._stamp(msg)
        if stamp_ns <= 0:
            return
        self.xt16.append(
            (time.monotonic_ns(), stamp_ns, msg.header.frame_id, msg.width, msg.height)
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure /utlidar/cloud_base versus /lidar_points timestamp offset."
    )
    parser.add_argument("--mouth-topic", default="/utlidar/cloud_base")
    parser.add_argument("--xt16-topic", default="/lidar_points")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--arrival-window", type=float, default=0.03)
    parser.add_argument("--stable-stdev", type=float, default=0.02)
    parser.add_argument("--stable-range", type=float, default=0.08)
    parser.add_argument("--min-pairs", type=int, default=20)
    parser.add_argument("--max-clock-step", type=float, default=0.25)
    return parser.parse_args()


def pair_by_arrival(xt16, mouth, arrival_window_ns):
    """Pair samples once, preserving order and maximizing pair count."""
    ordered_xt16 = sorted(xt16, key=lambda sample: sample[0])
    ordered_mouth = sorted(mouth, key=lambda sample: sample[0])
    indices = ordered_pair_indices(
        [sample[0] for sample in ordered_xt16],
        [sample[0] for sample in ordered_mouth],
        arrival_window_ns,
    )
    pairs = []
    for xt_index, mouth_index in indices:
        xt_sample = ordered_xt16[xt_index]
        mouth_sample = ordered_mouth[mouth_index]
        signed_arrival_delta_ns = xt_sample[0] - mouth_sample[0]
        raw_header_offset_ns = xt_sample[1] - mouth_sample[1]
        corrected_offset_ns = raw_header_offset_ns - signed_arrival_delta_ns
        pairs.append(
            (corrected_offset_ns, signed_arrival_delta_ns, raw_header_offset_ns)
        )
    return pairs


def main():
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
    node = OffsetSampler(args.mouth_topic, args.xt16_topic)
    try:
        end_time = time.monotonic() + args.duration
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)

        pairs = pair_by_arrival(
            node.xt16, node.mouth, int(round(args.arrival_window * 1e9))
        )

        print(
            f"mouth_topic={args.mouth_topic} xt16_topic={args.xt16_topic} "
            f"duration={args.duration:.1f}s"
        )
        print(
            f"mouth_count={len(node.mouth)} xt16_count={len(node.xt16)} "
            f"paired_by_arrival={len(pairs)}"
        )
        if node.mouth:
            last = node.mouth[-1]
            print(
                f"mouth_frame={last[2]} mouth_points={last[3] * last[4]} "
                f"mouth_stamp_last={last[1] * 1e-9:.6f}"
            )
        if node.xt16:
            last = node.xt16[-1]
            print(
                f"xt16_frame={last[2]} xt16_points={last[3] * last[4]} "
                f"xt16_stamp_last={last[1] * 1e-9:.6f}"
            )

        if len(pairs) < args.min_pairs:
            print(f"required_pairs={args.min_pairs}")
            print("OFFSET_STABLE_FOR_SMOKE=False")
            print("recommended_mouth_time_offset_sec=UNAVAILABLE")
            return 2

        offsets = [sample[0] * 1e-9 for sample in pairs]
        signed_arrival_dts = [sample[1] * 1e-9 for sample in pairs]
        raw_header_offsets = [sample[2] * 1e-9 for sample in pairs]
        offset_range = max(offsets) - min(offsets)
        offset_stdev = statistics.pstdev(offsets)
        offset_median = statistics.median(offsets)
        max_clock_step_ns = int(round(args.max_clock_step * 1e9))
        mouth_clock_step_ns, mouth_clock_span_ns = clock_phase_diagnostics(
            [sample[0] for sample in node.mouth],
            [sample[1] for sample in node.mouth],
        )
        xt16_clock_step_ns, xt16_clock_span_ns = clock_phase_diagnostics(
            [sample[0] for sample in node.xt16],
            [sample[1] for sample in node.xt16],
        )
        clock_steps_ok = max(
            mouth_clock_step_ns,
            mouth_clock_span_ns,
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
            f"offset_sec_stdev={offset_stdev:.6f} offset_sec_range={offset_range:.6f} "
            f"signed_arrival_dt_mean={statistics.mean(signed_arrival_dts):.6f}"
        )
        print(
            f"raw_header_offset_sec_median={statistics.median(raw_header_offsets):.6f} "
            f"clock_steps_ok={clock_steps_ok} "
            f"mouth_clock_phase_span_sec={mouth_clock_span_ns * 1e-9:.6f} "
            f"xt16_clock_phase_span_sec={xt16_clock_span_ns * 1e-9:.6f}"
        )
        print(f"OFFSET_STABLE_FOR_SMOKE={stable}")
        recommendation = f"{offset_median:.6f}" if stable else "UNAVAILABLE"
        print(f"recommended_mouth_time_offset_sec={recommendation}")
        return 0 if stable else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
