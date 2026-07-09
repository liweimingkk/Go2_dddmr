#!/usr/bin/env python3
"""Measure mouth LiDAR header timestamp offset relative to XT16.

This is a read-only helper. It subscribes to the two PointCloud2 topics,
pairs frames by arrival time, and reports:

    offset = xt16_header_stamp - mouth_header_stamp

Use the median offset as `mouth_time_offset_sec` only if the sample is stable in
the current live graph.
"""

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class OffsetSampler(Node):
    def __init__(self, mouth_topic, xt16_topic):
        super().__init__("go2_mouth_xt16_time_offset_sampler")
        self.mouth = []
        self.xt16 = []
        self.create_subscription(
            PointCloud2, mouth_topic, self._mouth_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, xt16_topic, self._xt16_cb, qos_profile_sensor_data
        )

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _mouth_cb(self, msg):
        self.mouth.append(
            (time.monotonic(), self._stamp(msg), msg.header.frame_id, msg.width, msg.height)
        )

    def _xt16_cb(self, msg):
        self.xt16.append(
            (time.monotonic(), self._stamp(msg), msg.header.frame_id, msg.width, msg.height)
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure /utlidar/cloud_base versus /lidar_points timestamp offset."
    )
    parser.add_argument("--mouth-topic", default="/utlidar/cloud_base")
    parser.add_argument("--xt16-topic", default="/lidar_points")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--arrival-window", type=float, default=0.08)
    parser.add_argument("--stable-stdev", type=float, default=0.06)
    parser.add_argument("--stable-range", type=float, default=0.20)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = OffsetSampler(args.mouth_topic, args.xt16_topic)
    try:
        end_time = time.monotonic() + args.duration
        while time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)

        pairs = []
        for xt_arrival, xt_stamp, *_ in node.xt16:
            if not node.mouth:
                break
            mouth_arrival, mouth_stamp, *_ = min(
                node.mouth, key=lambda sample: abs(sample[0] - xt_arrival)
            )
            arrival_dt = abs(mouth_arrival - xt_arrival)
            if arrival_dt <= args.arrival_window:
                pairs.append((xt_stamp - mouth_stamp, arrival_dt))

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
                f"mouth_stamp_last={last[1]:.6f}"
            )
        if node.xt16:
            last = node.xt16[-1]
            print(
                f"xt16_frame={last[2]} xt16_points={last[3] * last[4]} "
                f"xt16_stamp_last={last[1]:.6f}"
            )

        if not pairs:
            print("OFFSET_STABLE_FOR_SMOKE=False")
            print("recommended_mouth_time_offset_sec=UNAVAILABLE")
            return 2

        offsets = [sample[0] for sample in pairs]
        arrival_dts = [sample[1] for sample in pairs]
        offset_range = max(offsets) - min(offsets)
        offset_stdev = statistics.pstdev(offsets)
        offset_median = statistics.median(offsets)
        stable = offset_stdev <= args.stable_stdev and offset_range <= args.stable_range

        print(
            f"offset_sec_min={min(offsets):.6f} median={offset_median:.6f} "
            f"mean={statistics.mean(offsets):.6f} max={max(offsets):.6f}"
        )
        print(
            f"offset_sec_stdev={offset_stdev:.6f} offset_sec_range={offset_range:.6f} "
            f"arrival_dt_mean={statistics.mean(arrival_dts):.6f}"
        )
        print(f"OFFSET_STABLE_FOR_SMOKE={stable}")
        print(f"recommended_mouth_time_offset_sec={offset_median:.6f}")
        return 0 if stable else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
