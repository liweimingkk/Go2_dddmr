#!/usr/bin/env python3
"""Fail closed until LeGO-LOAM reports valid live odometry synchronization."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


class OdomSyncMonitor(Node):
    def __init__(
        self,
        topic: str,
        max_error: float,
        expected_offset: float | None,
        offset_tolerance: float,
        window_sec: float,
        min_samples: int,
        max_receive_gap_sec: float,
    ) -> None:
        super().__init__("go2_odom_sync_preflight")
        self.max_error = max_error
        self.expected_offset = expected_offset
        self.offset_tolerance = offset_tolerance
        self.window_sec = window_sec
        self.min_samples = min_samples
        self.max_receive_gap_sec = max_receive_gap_sec
        self.accepted: dict[str, str] | None = None
        self.valid_samples: list[tuple[float, dict[str, str]]] = []
        self.last_failure = "no lego_loam/odom_sync diagnostic received"
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(DiagnosticArray, topic, self._on_diagnostics, qos)

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "lego_loam/odom_sync":
                continue
            values = {item.key: item.value for item in status.values}
            try:
                enabled = parse_bool(values["sync_enabled"])
                valid = parse_bool(values["valid"])
                error = float(values["sync_error_sec"])
                configured_offset = float(values["configured_time_offset_sec"])
            except (KeyError, ValueError) as exc:
                self.last_failure = f"malformed odom-sync diagnostic: {exc}"
                self.valid_samples.clear()
                continue

            if not enabled:
                self.last_failure = "odom synchronization is disabled"
                self.valid_samples.clear()
                continue
            if not valid:
                self.last_failure = f"odom synchronization is invalid: {status.message}"
                self.valid_samples.clear()
                continue
            if not math.isfinite(error) or abs(error) > self.max_error:
                self.last_failure = (
                    f"sync error {error!r}s exceeds {self.max_error:.6f}s"
                )
                self.valid_samples.clear()
                continue
            if self.expected_offset is not None and (
                not math.isfinite(configured_offset)
                or abs(configured_offset - self.expected_offset) > self.offset_tolerance
            ):
                self.last_failure = (
                    f"configured offset {configured_offset!r}s does not match expected "
                    f"{self.expected_offset:.9f}s"
                )
                self.valid_samples.clear()
                continue

            now = time.monotonic()
            if (
                self.valid_samples
                and now - self.valid_samples[-1][0] > self.max_receive_gap_sec
            ):
                self.last_failure = (
                    "odom-sync diagnostic receive gap "
                    f"{now - self.valid_samples[-1][0]:.3f}s exceeds "
                    f"{self.max_receive_gap_sec:.3f}s"
                )
                self.valid_samples.clear()
            self.valid_samples.append((now, values))
            if (
                len(self.valid_samples) >= self.min_samples
                and now - self.valid_samples[0][0] >= self.window_sec
            ):
                self.accepted = values
            return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/odom_sync_diagnostics")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-error", type=float, default=0.05)
    parser.add_argument("--expected-offset", type=float)
    parser.add_argument("--offset-tolerance", type=float, default=1e-6)
    parser.add_argument("--window-sec", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--max-receive-gap-sec", type=float, default=0.30)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")
    if args.max_error < 0.0:
        parser.error("--max-error must be nonnegative")
    if args.offset_tolerance < 0.0:
        parser.error("--offset-tolerance must be nonnegative")
    if not math.isfinite(args.window_sec) or args.window_sec <= 0.0:
        parser.error("--window-sec must be finite and greater than zero")
    if args.min_samples < 2:
        parser.error("--min-samples must be at least 2")
    if (
        not math.isfinite(args.max_receive_gap_sec)
        or args.max_receive_gap_sec <= 0.0
    ):
        parser.error("--max-receive-gap-sec must be finite and greater than zero")
    if args.timeout <= args.window_sec:
        parser.error("--timeout must be greater than --window-sec")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = OdomSyncMonitor(
        args.topic,
        args.max_error,
        args.expected_offset,
        args.offset_tolerance,
        args.window_sec,
        args.min_samples,
        args.max_receive_gap_sec,
    )
    deadline = time.monotonic() + args.timeout
    try:
        while node.accepted is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))

        if node.accepted is None:
            print(f"ODOM_SYNC_VALID=False reason={node.last_failure}", file=sys.stderr)
            return 1

        values = node.accepted
        print("ODOM_SYNC_VALID=True")
        print(f"sustained_window_sec={args.window_sec:.3f}")
        print(f"valid_sample_count={len(node.valid_samples)}")
        for key in (
            "configured_time_offset_sec",
            "sync_error_sec",
            "interpolated",
            "bracket_span_sec",
        ):
            if key in values:
                print(f"{key}={values[key]}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
