#!/usr/bin/env python3
"""Query, disable, or enable the Unitree Go2 built-in obstacle avoidance.

The default action is read-only. Disabling is deliberately guarded by a
stationary check and uses the deterministic obstacle-avoidance SwitchSet API,
not the Sport-mode toggle API. This script never publishes a motion command.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple


OBSTACLE_REQUEST_TOPIC = "/api/obstacles_avoid/request"
OBSTACLE_RESPONSE_TOPIC = "/api/obstacles_avoid/response"
SPORT_STATE_TOPIC = "/sportmodestate"
MULTISTATE_TOPIC = "/multiplestate"

SWITCH_SET_API_ID = 1001
SWITCH_GET_API_ID = 1002

DEFAULT_DISCOVERY_TIMEOUT = 5.0
DEFAULT_RESPONSE_TIMEOUT = 1.8
DEFAULT_VERIFY_TIMEOUT = 3.0
DEFAULT_RETRIES = 3
DEFAULT_STATIONARY_SAMPLES = 12
DEFAULT_STATIONARY_TIMEOUT = 4.0

# Reject sustained low-speed motion while tolerating isolated estimator noise.
MAX_MEDIAN_LINEAR_SPEED = 0.03
MAX_MEDIAN_YAW_SPEED = 0.08
MAX_PEAK_LINEAR_SPEED = 0.12
MAX_PEAK_YAW_SPEED = 0.25


@dataclass(frozen=True)
class MotionSample:
    vx: float
    vy: float
    yaw_speed: float


@dataclass(frozen=True)
class StationaryResult:
    stationary: bool
    median_abs_vx: float
    median_abs_vy: float
    median_abs_yaw: float
    peak_abs_vx: float
    peak_abs_vy: float
    peak_abs_yaw: float


def evaluate_stationary(samples: Sequence[MotionSample]) -> StationaryResult:
    """Evaluate whether a set of SportModeState samples represents rest."""
    if not samples:
        raise ValueError("at least one motion sample is required")

    values = [
        component
        for sample in samples
        for component in (sample.vx, sample.vy, sample.yaw_speed)
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("motion samples must be finite")

    abs_vx = [abs(sample.vx) for sample in samples]
    abs_vy = [abs(sample.vy) for sample in samples]
    abs_yaw = [abs(sample.yaw_speed) for sample in samples]
    median_abs_vx = statistics.median(abs_vx)
    median_abs_vy = statistics.median(abs_vy)
    median_abs_yaw = statistics.median(abs_yaw)
    peak_abs_vx = max(abs_vx)
    peak_abs_vy = max(abs_vy)
    peak_abs_yaw = max(abs_yaw)
    stationary = (
        median_abs_vx <= MAX_MEDIAN_LINEAR_SPEED
        and median_abs_vy <= MAX_MEDIAN_LINEAR_SPEED
        and median_abs_yaw <= MAX_MEDIAN_YAW_SPEED
        and peak_abs_vx <= MAX_PEAK_LINEAR_SPEED
        and peak_abs_vy <= MAX_PEAK_LINEAR_SPEED
        and peak_abs_yaw <= MAX_PEAK_YAW_SPEED
    )
    return StationaryResult(
        stationary=stationary,
        median_abs_vx=median_abs_vx,
        median_abs_vy=median_abs_vy,
        median_abs_yaw=median_abs_yaw,
        peak_abs_vx=peak_abs_vx,
        peak_abs_vy=peak_abs_vy,
        peak_abs_yaw=peak_abs_yaw,
    )


def parse_enable_payload(payload: str) -> bool:
    """Return the strict boolean ``enable`` value from a Unitree JSON reply."""
    try:
        value = json.loads(payload)["enable"]
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise ValueError(f"invalid obstacle-avoidance payload: {payload!r}") from exc
    if not isinstance(value, bool):
        raise ValueError(f"obstacle-avoidance enable is not boolean: {value!r}")
    return value


def parse_multistate_payload(payload: str) -> bool:
    """Return the strict ``obstaclesAvoidSwitch`` value from /multiplestate."""
    try:
        value = json.loads(payload)["obstaclesAvoidSwitch"]
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise ValueError(f"invalid /multiplestate payload: {payload!r}") from exc
    if not isinstance(value, bool):
        raise ValueError(
            f"/multiplestate obstaclesAvoidSwitch is not boolean: {value!r}"
        )
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Query or set Go2 built-in obstacle avoidance. With no action flag, "
            "the script only queries the current state."
        )
    )
    action = parser.add_mutually_exclusive_group()
    action.add_argument(
        "--disable",
        action="store_true",
        help="disable obstacle avoidance after confirming the robot is stationary",
    )
    action.add_argument(
        "--enable",
        action="store_true",
        help="restore obstacle avoidance",
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=DEFAULT_DISCOVERY_TIMEOUT,
        help=f"DDS discovery timeout in seconds (default: {DEFAULT_DISCOVERY_TIMEOUT})",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=DEFAULT_RESPONSE_TIMEOUT,
        help=f"per-request timeout in seconds (default: {DEFAULT_RESPONSE_TIMEOUT})",
    )
    parser.add_argument(
        "--verify-timeout",
        type=float,
        default=DEFAULT_VERIFY_TIMEOUT,
        help=f"/multiplestate verification timeout (default: {DEFAULT_VERIFY_TIMEOUT})",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_RETRIES,
        help=f"maximum SwitchSet attempts (default: {DEFAULT_RETRIES})",
    )
    return parser


def validate_options(args: argparse.Namespace) -> None:
    for name in ("discovery_timeout", "response_timeout", "verify_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0.1 <= value <= 30.0:
            raise ValueError(f"{name.replace('_', ' ')} must be in [0.1, 30] seconds")
    if not 1 <= args.retries <= 10:
        raise ValueError("retries must be in [1, 10]")


class Go2ObstacleAvoidanceClient:
    """Small synchronous client around Unitree's obstacle-avoidance API."""

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.qos import (
                QoSDurabilityPolicy,
                QoSHistoryPolicy,
                QoSProfile,
                QoSReliabilityPolicy,
            )
            from std_msgs.msg import String
            from unitree_api.msg import Request, Response
            from unitree_go.msg import SportModeState
        except ImportError as exc:
            raise RuntimeError(
                "rclpy, unitree_api, and unitree_go are required; source ROS 2 "
                "and dddmr_navigation/.unitree_msg_ws/install/setup.bash first"
            ) from exc

        self.rclpy = rclpy
        self.request_type = Request
        self.owns_rclpy = not rclpy.ok()
        if self.owns_rclpy:
            rclpy.init(args=[])
        self.node = rclpy.create_node("go2_obstacle_avoidance_control")

        reliable = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.node.create_publisher(
            Request, OBSTACLE_REQUEST_TOPIC, reliable
        )
        self.responses: dict[Tuple[int, int], Any] = {}
        self.pending: set[Tuple[int, int]] = set()
        self.latest_multistate: Optional[str] = None
        self.motion_samples: list[MotionSample] = []
        self.collect_motion = False
        self.response_subscription = self.node.create_subscription(
            Response,
            OBSTACLE_RESPONSE_TOPIC,
            self._on_response,
            reliable,
        )
        self.multistate_subscription = self.node.create_subscription(
            String, MULTISTATE_TOPIC, self._on_multistate, best_effort
        )
        self.sport_state_subscription = self.node.create_subscription(
            SportModeState, SPORT_STATE_TOPIC, self._on_sport_state, best_effort
        )

    def _on_response(self, msg: Any) -> None:
        key = (int(msg.header.identity.id), int(msg.header.identity.api_id))
        if key in self.pending:
            self.responses[key] = msg

    def _on_multistate(self, msg: Any) -> None:
        self.latest_multistate = msg.data

    def _on_sport_state(self, msg: Any) -> None:
        if not self.collect_motion or len(msg.velocity) < 2:
            return
        self.motion_samples.append(
            MotionSample(
                vx=float(msg.velocity[0]),
                vy=float(msg.velocity[1]),
                yaw_speed=float(msg.yaw_speed),
            )
        )

    def wait_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"no subscriber discovered on {OBSTACLE_REQUEST_TOPIC}"
                )
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def stationary_check(
        self,
        sample_count: int = DEFAULT_STATIONARY_SAMPLES,
        timeout: float = DEFAULT_STATIONARY_TIMEOUT,
    ) -> StationaryResult:
        self.motion_samples.clear()
        self.collect_motion = True
        deadline = time.monotonic() + timeout
        try:
            while len(self.motion_samples) < sample_count:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"received only {len(self.motion_samples)}/{sample_count} "
                        f"samples from {SPORT_STATE_TOPIC}; refusing to disable"
                    )
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.collect_motion = False
        return evaluate_stationary(self.motion_samples)

    def _call(self, api_id: int, parameter: str, timeout: float) -> Any:
        request_id = int(time.monotonic_ns() % ((1 << 63) - 1))
        key = (request_id, api_id)
        request = self.request_type()
        request.header.identity.id = request_id
        request.header.identity.api_id = api_id
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        request.parameter = parameter
        self.pending.add(key)
        self.publisher.publish(request)
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                self.rclpy.spin_once(self.node, timeout_sec=0.05)
                response = self.responses.pop(key, None)
                if response is not None:
                    return response
        finally:
            self.pending.discard(key)
            self.responses.pop(key, None)
        raise RuntimeError(f"API {api_id} response timed out")

    def get_enabled(self, timeout: float) -> bool:
        response = self._call(SWITCH_GET_API_ID, "", timeout)
        if int(response.header.status.code) != 0:
            raise RuntimeError(
                f"SwitchGet failed with status {response.header.status.code}"
            )
        return parse_enable_payload(response.data)

    def set_enabled(self, enabled: bool, timeout: float, retries: int) -> bool:
        parameter = json.dumps({"enable": enabled}, separators=(",", ":"))
        errors = []
        for attempt in range(1, retries + 1):
            try:
                response = self._call(SWITCH_SET_API_ID, parameter, timeout)
                status = int(response.header.status.code)
                if status != 0:
                    errors.append(f"attempt {attempt}: status {status}")
                    continue
                if self.get_enabled(timeout) == enabled:
                    return enabled
                errors.append(f"attempt {attempt}: SwitchGet state mismatch")
            except RuntimeError as exc:
                errors.append(f"attempt {attempt}: {exc}")
        raise RuntimeError("SwitchSet verification failed; " + "; ".join(errors))

    def verify_multistate(self, expected: bool, timeout: float) -> bool:
        self.latest_multistate = None
        deadline = time.monotonic() + timeout
        last_error: Optional[ValueError] = None
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_multistate is None:
                continue
            payload = self.latest_multistate
            self.latest_multistate = None
            try:
                actual = parse_multistate_payload(payload)
            except ValueError as exc:
                last_error = exc
                continue
            if actual == expected:
                return actual
            last_error = ValueError(
                f"/multiplestate reports {actual}, expected {expected}"
            )
        if last_error is not None:
            raise RuntimeError(str(last_error))
        raise RuntimeError(f"no usable message received from {MULTISTATE_TOPIC}")

    def close(self) -> None:
        self.node.destroy_node()
        if self.owns_rclpy and self.rclpy.ok():
            self.rclpy.shutdown()


def format_stationary(result: StationaryResult) -> str:
    return (
        "median |vx,vy,yaw|="
        f"{result.median_abs_vx:.4f},"
        f"{result.median_abs_vy:.4f},"
        f"{result.median_abs_yaw:.4f}; peak="
        f"{result.peak_abs_vx:.4f},"
        f"{result.peak_abs_vy:.4f},"
        f"{result.peak_abs_yaw:.4f}"
    )


def run(args: argparse.Namespace) -> int:
    validate_options(args)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    if os.environ["RMW_IMPLEMENTATION"] != "rmw_cyclonedds_cpp":
        raise ValueError("RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp")
    if os.environ["ROS_DOMAIN_ID"] != "0":
        raise ValueError("ROS_DOMAIN_ID must be 0")

    client = Go2ObstacleAvoidanceClient()
    try:
        client.wait_ready(args.discovery_timeout)
        if not args.disable and not args.enable:
            enabled = client.get_enabled(args.response_timeout)
            print(f"Go2 obstacle avoidance: {'ENABLED' if enabled else 'DISABLED'}")
            return 0

        target = bool(args.enable)
        if args.disable:
            stationary = client.stationary_check()
            print(f"Stationary check: {format_stationary(stationary)}")
            if not stationary.stationary:
                raise RuntimeError(
                    "robot is not confirmed stationary; refusing to disable "
                    "obstacle avoidance"
                )

        client.set_enabled(target, args.response_timeout, args.retries)
        client.verify_multistate(target, args.verify_timeout)
        print(
            "Go2 obstacle avoidance set and verified: "
            f"{'ENABLED' if target else 'DISABLED'}"
        )
        return 0
    finally:
        client.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
