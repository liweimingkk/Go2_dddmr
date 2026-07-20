#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


DATATYPES = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}

DATATYPE_NAMES = {
    PointField.INT8: "INT8",
    PointField.UINT8: "UINT8",
    PointField.INT16: "INT16",
    PointField.UINT16: "UINT16",
    PointField.INT32: "INT32",
    PointField.UINT32: "UINT32",
    PointField.FLOAT32: "FLOAT32",
    PointField.FLOAT64: "FLOAT64",
}


def field_map(msg):
    return {field.name: field for field in msg.fields}


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def unpack_field(msg, field, point_index):
    if field.datatype not in DATATYPES:
        raise ValueError(f"unsupported field datatype {field.datatype} for {field.name}")
    fmt, _ = DATATYPES[field.datatype]
    endian = ">" if msg.is_bigendian else "<"
    return struct.unpack_from(endian + fmt, msg.data, point_index * msg.point_step + field.offset)[0]


def finite_range(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None, None
    return min(finite), max(finite)


def analyze_message(msg, args):
    fields = field_map(msg)
    failures = []
    warnings = []

    required_fields = ["x", "y", "z", "intensity", args.ring_field, args.timestamp_field]
    missing = [name for name in required_fields if name not in fields]
    if missing:
        failures.append(f"missing fields: {', '.join(missing)}")

    point_count = msg.width * msg.height
    if args.expect_frame and msg.header.frame_id != args.expect_frame:
        failures.append(f"frame_id={msg.header.frame_id!r}, expected {args.expect_frame!r}")
    if args.expect_width > 0 and msg.width != args.expect_width:
        failures.append(f"width={msg.width}, expected {args.expect_width}")
    if args.expect_point_step > 0 and msg.point_step != args.expect_point_step:
        failures.append(f"point_step={msg.point_step}, expected {args.expect_point_step}")

    report = {
        "stamp": stamp_to_sec(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "width": msg.width,
        "height": msg.height,
        "point_step": msg.point_step,
        "point_count": point_count,
        "fields": fields,
        "failures": failures,
        "warnings": warnings,
    }

    if missing:
        return report

    ring_field = fields[args.ring_field]
    timestamp_field = fields[args.timestamp_field]

    if ring_field.datatype != PointField.UINT16:
        failures.append(
            f"{args.ring_field} datatype={DATATYPE_NAMES.get(ring_field.datatype, ring_field.datatype)}, expected UINT16"
        )
    if timestamp_field.datatype != PointField.FLOAT64:
        failures.append(
            f"{args.timestamp_field} datatype={DATATYPE_NAMES.get(timestamp_field.datatype, timestamp_field.datatype)}, expected FLOAT64"
        )

    ring_counts = Counter()
    timestamps = []
    sample_stride = max(1, args.sample_stride)

    for i in range(point_count):
        try:
            ring = int(unpack_field(msg, ring_field, i))
            ring_counts[ring] += 1
            if i % sample_stride == 0 or i == point_count - 1:
                timestamps.append(float(unpack_field(msg, timestamp_field, i)))
        except (struct.error, ValueError) as exc:
            failures.append(f"failed reading point {i}: {exc}")
            break

    if ring_counts:
        unique_rings = sorted(ring_counts)
        report["ring_min"] = unique_rings[0]
        report["ring_max"] = unique_rings[-1]
        report["ring_count"] = len(unique_rings)
        report["ring_counts"] = ring_counts

        if args.expect_rings > 0 and len(unique_rings) != args.expect_rings:
            failures.append(f"unique ring count={len(unique_rings)}, expected {args.expect_rings}")
        if args.expect_ring_min is not None and unique_rings[0] != args.expect_ring_min:
            failures.append(f"ring min={unique_rings[0]}, expected {args.expect_ring_min}")
        if args.expect_ring_max is not None and unique_rings[-1] != args.expect_ring_max:
            failures.append(f"ring max={unique_rings[-1]}, expected {args.expect_ring_max}")
        if args.expect_points_per_ring > 0:
            bad = {
                ring: count
                for ring, count in ring_counts.items()
                if count != args.expect_points_per_ring
            }
            if bad:
                failures.append(
                    "points per ring mismatch: "
                    + ", ".join(f"{ring}:{count}" for ring, count in sorted(bad.items()))
                )

    timestamp_min, timestamp_max = finite_range(timestamps)
    if timestamp_min is None:
        failures.append("no finite timestamp samples")
    else:
        timestamp_span = timestamp_max - timestamp_min
        report["timestamp_min"] = timestamp_min
        report["timestamp_max"] = timestamp_max
        report["timestamp_span"] = timestamp_span
        if timestamp_span < args.timestamp_span_min or timestamp_span > args.timestamp_span_max:
            failures.append(
                f"timestamp span={timestamp_span:.6f}s, expected "
                f"{args.timestamp_span_min:.3f}..{args.timestamp_span_max:.3f}s"
            )

        first_timestamp = float(unpack_field(msg, timestamp_field, 0))
        header_delta = abs(stamp_to_sec(msg.header.stamp) - first_timestamp)
        report["first_timestamp"] = first_timestamp
        report["header_first_delta"] = header_delta
        if header_delta > args.header_first_tolerance:
            warnings.append(
                f"header stamp differs from first point timestamp by {header_delta:.6f}s"
            )

    return report


class PreflightNode(Node):
    def __init__(self, args):
        super().__init__("go2_xt16_lidar_preflight")
        self.args = args
        self.reports = []
        self.receipt_times = []
        reliability = ReliabilityPolicy.RELIABLE
        if args.qos_reliability == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(PointCloud2, args.topic, self.callback, qos)

    def callback(self, msg):
        if len(self.reports) >= self.args.samples:
            return
        self.receipt_times.append(time.monotonic())
        self.reports.append(analyze_message(msg, self.args))


def print_report(reports, receipt_times):
    for idx, report in enumerate(reports, start=1):
        fields = report["fields"]
        field_desc = ", ".join(
            f"{field.name}:{DATATYPE_NAMES.get(field.datatype, field.datatype)}@{field.offset}"
            for field in sorted(fields.values(), key=lambda field: field.offset)
        )
        print(
            f"[sample {idx}] frame={report['frame_id']} stamp={report['stamp']:.6f} "
            f"size={report['width']}x{report['height']} point_step={report['point_step']} "
            f"points={report['point_count']}"
        )
        print(f"  fields: {field_desc}")
        if "ring_count" in report:
            print(
                f"  ring: min={report['ring_min']} max={report['ring_max']} "
                f"unique={report['ring_count']}"
            )
        if "timestamp_span" in report:
            print(
                f"  timestamp: first={report['first_timestamp']:.6f} "
                f"span={report['timestamp_span']:.6f}s "
                f"header_delta={report['header_first_delta']:.6f}s"
            )
        for warning in report["warnings"]:
            print(f"  WARN: {warning}")
        for failure in report["failures"]:
            print(f"  FAIL: {failure}")

    if len(reports) >= 2:
        header_deltas = [
            reports[i]["stamp"] - reports[i - 1]["stamp"]
            for i in range(1, len(reports))
        ]
        receipt_deltas = [
            receipt_times[i] - receipt_times[i - 1]
            for i in range(1, len(receipt_times))
        ]
        header_hz = [1.0 / delta for delta in header_deltas if delta > 0.0]
        if header_hz:
            print(
                f"header rate: min={min(header_hz):.2f}Hz "
                f"max={max(header_hz):.2f}Hz avg={sum(header_hz) / len(header_hz):.2f}Hz"
            )
        receipt_hz = [1.0 / delta for delta in receipt_deltas if delta > 0.0]
        if receipt_hz:
            print(
                f"receive rate: min={min(receipt_hz):.2f}Hz "
                f"max={max(receipt_hz):.2f}Hz avg={sum(receipt_hz) / len(receipt_hz):.2f}Hz"
            )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Read-only Go2 XT16 PointCloud2 contract preflight for DDDMR lego_loam."
    )
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--qos-reliability",
        choices=["reliable", "best_effort"],
        default="best_effort",
    )
    parser.add_argument("--qos-depth", type=int, default=10)
    parser.add_argument("--expect-frame", default="hesai_lidar")
    parser.add_argument("--expect-width", type=int, default=32000)
    parser.add_argument("--expect-point-step", type=int, default=26)
    parser.add_argument("--expect-rings", type=int, default=16)
    parser.add_argument("--expect-ring-min", type=int, default=0)
    parser.add_argument("--expect-ring-max", type=int, default=15)
    parser.add_argument("--expect-points-per-ring", type=int, default=2000)
    parser.add_argument("--ring-field", default="ring")
    parser.add_argument("--timestamp-field", default="timestamp")
    parser.add_argument("--timestamp-span-min", type=float, default=0.08)
    parser.add_argument("--timestamp-span-max", type=float, default=0.12)
    parser.add_argument("--header-first-tolerance", type=float, default=0.05)
    parser.add_argument("--sample-stride", type=int, default=16)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.samples < 1:
        print("samples must be >= 1", file=sys.stderr)
        return 2
    if args.qos_depth < 1:
        print("qos-depth must be >= 1", file=sys.stderr)
        return 2

    rclpy.init()
    node = PreflightNode(args)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and len(node.reports) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        reports = list(node.reports)
        receipt_times = list(node.receipt_times)
        node.destroy_node()
        rclpy.shutdown()

    if len(reports) < args.samples:
        print(
            f"FAIL: received {len(reports)}/{args.samples} samples from {args.topic} "
            f"within {args.timeout:.1f}s"
        )
        print(
            "Check ROS_DOMAIN_ID, CycloneDDS interface, QoS reliability, "
            "Go2 Ethernet link, and XT16 driver state."
        )
        return 1

    print_report(reports, receipt_times)
    failed = any(report["failures"] for report in reports)
    if failed:
        print("RESULT: FAIL")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
