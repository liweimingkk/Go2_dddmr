#!/usr/bin/env python3
"""Fail closed until LeGO-LOAM reports valid live odometry synchronization."""

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
    ) -> None:
        super().__init__("go2_odom_sync_preflight")
        self.max_error = max_error
        self.expected_offset = expected_offset
        self.offset_tolerance = offset_tolerance
        self.accepted: dict[str, str] | None = None
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
                continue

            if not enabled:
                self.last_failure = "odom synchronization is disabled"
                continue
            if not valid:
                self.last_failure = f"odom synchronization is invalid: {status.message}"
                continue
            if not math.isfinite(error) or abs(error) > self.max_error:
                self.last_failure = (
                    f"sync error {error!r}s exceeds {self.max_error:.6f}s"
                )
                continue
            if self.expected_offset is not None and (
                not math.isfinite(configured_offset)
                or abs(configured_offset - self.expected_offset) > self.offset_tolerance
            ):
                self.last_failure = (
                    f"configured offset {configured_offset!r}s does not match expected "
                    f"{self.expected_offset:.9f}s"
                )
                continue

            self.accepted = values
            return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/odom_sync_diagnostics")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-error", type=float, default=0.05)
    parser.add_argument("--expected-offset", type=float)
    parser.add_argument("--offset-tolerance", type=float, default=1e-6)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")
    if args.max_error < 0.0:
        parser.error("--max-error must be nonnegative")
    if args.offset_tolerance < 0.0:
        parser.error("--offset-tolerance must be nonnegative")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = OdomSyncMonitor(
        args.topic,
        args.max_error,
        args.expected_offset,
        args.offset_tolerance,
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
