#!/usr/bin/env python3

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple


POINTCLOUD2_TYPE = "sensor_msgs/msg/PointCloud2"


@dataclass(frozen=True)
class StreamThresholds:
    window_sec: float
    min_samples: int
    min_rate_hz: float
    max_header_gap_sec: float
    max_receive_gap_sec: float
    expected_publishers: int = 1


@dataclass(frozen=True)
class StreamEvaluation:
    passed: bool
    reasons: Tuple[str, ...]
    publisher_count: int
    sample_count: int
    receive_rate_hz: float
    header_rate_hz: float
    max_receive_gap_sec: float
    max_header_gap_sec: float
    tail_age_sec: float


def validate_thresholds(thresholds: StreamThresholds) -> None:
    numeric_values = (
        thresholds.window_sec,
        thresholds.min_rate_hz,
        thresholds.max_header_gap_sec,
        thresholds.max_receive_gap_sec,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in numeric_values):
        raise ValueError("stream timing thresholds must be finite and positive")
    if thresholds.min_samples < 2:
        raise ValueError("min_samples must be at least 2")
    if thresholds.expected_publishers < 1:
        raise ValueError("expected_publishers must be at least 1")


def _positive_deltas(values: Sequence[float]) -> Tuple[float, ...]:
    return tuple(values[index] - values[index - 1] for index in range(1, len(values)))


def _rate(sample_count: int, span_sec: float) -> float:
    if sample_count < 2 or not math.isfinite(span_sec) or span_sec <= 0.0:
        return 0.0
    return (sample_count - 1) / span_sec


def evaluate_stream(
    *,
    receipt_times: Sequence[float],
    header_stamps_ns: Sequence[int],
    evaluation_time: float,
    publisher_count: int,
    topic_types: Iterable[str],
    thresholds: StreamThresholds,
) -> StreamEvaluation:
    """Evaluate a completed observation window without requiring ROS imports."""
    validate_thresholds(thresholds)
    reasons = []
    receipts = tuple(float(value) for value in receipt_times)
    stamps_ns = tuple(int(value) for value in header_stamps_ns)
    advertised_types = tuple(topic_types)

    if len(receipts) != len(stamps_ns):
        raise ValueError("receipt_times and header_stamps_ns must have equal lengths")
    if publisher_count != thresholds.expected_publishers:
        reasons.append(
            "publisher_count=%d expected=%d"
            % (publisher_count, thresholds.expected_publishers)
        )
    if advertised_types != (POINTCLOUD2_TYPE,):
        reasons.append("topic_types=%s expected=%s" % (advertised_types, POINTCLOUD2_TYPE))
    if not math.isfinite(evaluation_time):
        raise ValueError("evaluation_time must be finite")
    if any(not math.isfinite(value) for value in receipts):
        raise ValueError("receipt times must be finite")

    sample_count = len(receipts)
    receive_rate_hz = 0.0
    header_rate_hz = 0.0
    max_receive_gap_sec = math.inf
    max_header_gap_sec = math.inf
    tail_age_sec = math.inf

    if sample_count == 0:
        reasons.append("no_samples")
    else:
        observation_span = evaluation_time - receipts[0]
        tail_age_sec = evaluation_time - receipts[-1]
        if observation_span + 1e-6 < thresholds.window_sec:
            reasons.append(
                "observation_window=%.3f required=%.3f"
                % (observation_span, thresholds.window_sec)
            )
        if tail_age_sec < 0.0 or tail_age_sec > thresholds.max_receive_gap_sec:
            reasons.append(
                "tail_age=%.3f max=%.3f"
                % (tail_age_sec, thresholds.max_receive_gap_sec)
            )

    if sample_count < thresholds.min_samples:
        reasons.append(
            "sample_count=%d min=%d" % (sample_count, thresholds.min_samples)
        )

    if sample_count >= 2:
        receive_deltas = _positive_deltas(receipts)
        header_values_sec = tuple(value * 1e-9 for value in stamps_ns)
        header_deltas = _positive_deltas(header_values_sec)
        receive_rate_hz = _rate(sample_count, receipts[-1] - receipts[0])
        header_rate_hz = _rate(
            sample_count, header_values_sec[-1] - header_values_sec[0]
        )
        max_receive_gap_sec = max(receive_deltas)
        max_header_gap_sec = max(header_deltas)

        if any(delta <= 0.0 for delta in receive_deltas):
            reasons.append("receive_times_not_strictly_increasing")
        if any(delta <= 0.0 for delta in header_deltas) or any(
            value <= 0 for value in stamps_ns
        ):
            reasons.append("header_stamps_not_strictly_increasing")
        if receive_rate_hz + 1e-9 < thresholds.min_rate_hz:
            reasons.append(
                "receive_rate_hz=%.3f min=%.3f"
                % (receive_rate_hz, thresholds.min_rate_hz)
            )
        if header_rate_hz + 1e-9 < thresholds.min_rate_hz:
            reasons.append(
                "header_rate_hz=%.3f min=%.3f"
                % (header_rate_hz, thresholds.min_rate_hz)
            )
        if max_receive_gap_sec > thresholds.max_receive_gap_sec + 1e-9:
            reasons.append(
                "max_receive_gap=%.3f max=%.3f"
                % (max_receive_gap_sec, thresholds.max_receive_gap_sec)
            )
        if max_header_gap_sec > thresholds.max_header_gap_sec + 1e-9:
            reasons.append(
                "max_header_gap=%.3f max=%.3f"
                % (max_header_gap_sec, thresholds.max_header_gap_sec)
            )

    return StreamEvaluation(
        passed=not reasons,
        reasons=tuple(reasons),
        publisher_count=publisher_count,
        sample_count=sample_count,
        receive_rate_hz=receive_rate_hz,
        header_rate_hz=header_rate_hz,
        max_receive_gap_sec=max_receive_gap_sec,
        max_header_gap_sec=max_header_gap_sec,
        tail_age_sec=tail_age_sec,
    )


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fail-closed freshness gate for a live ROS PointCloud2 stream."
    )
    parser.add_argument(
        "--topic",
        default="/perception_3d_local/lidar/current_observation",
    )
    parser.add_argument("--window-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--min-rate-hz", type=float, default=7.0)
    parser.add_argument("--max-header-gap-sec", type=float, default=0.25)
    parser.add_argument("--max-receive-gap-sec", type=float, default=0.20)
    parser.add_argument("--expected-publishers", type=int, default=1)
    args = parser.parse_args(argv)

    thresholds = StreamThresholds(
        window_sec=args.window_sec,
        min_samples=args.min_samples,
        min_rate_hz=args.min_rate_hz,
        max_header_gap_sec=args.max_header_gap_sec,
        max_receive_gap_sec=args.max_receive_gap_sec,
        expected_publishers=args.expected_publishers,
    )
    try:
        validate_thresholds(thresholds)
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= args.window_sec:
        parser.error("--timeout-sec must be finite and greater than --window-sec")
    if not args.topic.startswith("/") or any(char.isspace() for char in args.topic):
        parser.error("--topic must be an absolute ROS topic without whitespace")
    return args


def _format_metric(value: float) -> str:
    return "unknown" if not math.isfinite(value) else "%.3f" % value


def run_ros_gate(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        print("CURRENT_OBSERVATION_GATE=FAIL", file=sys.stderr)
        print("REASON=missing_ros_python_dependency:%s" % exc, file=sys.stderr)
        return 2

    class PointCloudStreamGateNode(Node):
        def __init__(self) -> None:
            super().__init__("go2_pointcloud_stream_gate")
            self.receipt_times = []
            self.header_stamps_ns = []
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.subscription = self.create_subscription(
                PointCloud2, args.topic, self._callback, qos
            )

        def _callback(self, msg: PointCloud2) -> None:
            self.receipt_times.append(time.monotonic())
            self.header_stamps_ns.append(
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )

    thresholds = StreamThresholds(
        window_sec=args.window_sec,
        min_samples=args.min_samples,
        min_rate_hz=args.min_rate_hz,
        max_header_gap_sec=args.max_header_gap_sec,
        max_receive_gap_sec=args.max_receive_gap_sec,
        expected_publishers=args.expected_publishers,
    )

    rclpy.init(args=[])
    node = PointCloudStreamGateNode()
    try:
        deadline = time.monotonic() + args.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if (
                node.receipt_times
                and now - node.receipt_times[0] >= args.window_sec
            ):
                break
        evaluation_time = time.monotonic()
        topic_types = dict(node.get_topic_names_and_types()).get(args.topic, [])
        result = evaluate_stream(
            receipt_times=node.receipt_times,
            header_stamps_ns=node.header_stamps_ns,
            evaluation_time=evaluation_time,
            publisher_count=node.count_publishers(args.topic),
            topic_types=topic_types,
            thresholds=thresholds,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("CURRENT_OBSERVATION_GATE=%s" % ("PASS" if result.passed else "FAIL"))
    print("TOPIC=%s" % args.topic)
    print("PUBLISHER_COUNT=%d" % result.publisher_count)
    print("SAMPLE_COUNT=%d" % result.sample_count)
    print("RECEIVE_RATE_HZ=%s" % _format_metric(result.receive_rate_hz))
    print("HEADER_RATE_HZ=%s" % _format_metric(result.header_rate_hz))
    print("MAX_RECEIVE_GAP_SEC=%s" % _format_metric(result.max_receive_gap_sec))
    print("MAX_HEADER_GAP_SEC=%s" % _format_metric(result.max_header_gap_sec))
    print("TAIL_AGE_SEC=%s" % _format_metric(result.tail_age_sec))
    if result.reasons:
        print("REASONS=%s" % ";".join(result.reasons))
    return 0 if result.passed else 1


def main(argv: Sequence[str] = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    return run_ros_gate(args)


if __name__ == "__main__":
    raise SystemExit(main())
