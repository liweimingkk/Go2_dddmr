#!/usr/bin/env python3
"""Send a bounded P2P dry-run goal and summarize no-motion navigation output."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


@dataclass
class TwistStats:
    count: int = 0
    max_abs_x: float = 0.0
    max_abs_y: float = 0.0
    max_abs_yaw: float = 0.0
    max_forward_x: float = 0.0
    min_forward_x: float = 0.0
    nonzero_count: int = 0

    def add(self, msg: Twist, zero_epsilon: float) -> None:
        x = float(msg.linear.x)
        y = float(msg.linear.y)
        yaw = float(msg.angular.z)
        self.count += 1
        self.max_abs_x = max(self.max_abs_x, abs(x))
        self.max_abs_y = max(self.max_abs_y, abs(y))
        self.max_abs_yaw = max(self.max_abs_yaw, abs(yaw))
        self.max_forward_x = max(self.max_forward_x, x)
        self.min_forward_x = min(self.min_forward_x, x)
        if abs(x) > zero_epsilon or abs(y) > zero_epsilon or abs(yaw) > zero_epsilon:
            self.nonzero_count += 1


@dataclass
class ProbeState:
    dry: TwistStats = field(default_factory=TwistStats)
    safe: TwistStats = field(default_factory=TwistStats)
    decisions: list[str] = field(default_factory=list)
    global_path_sizes: list[int] = field(default_factory=list)
    awared_path_sizes: list[int] = field(default_factory=list)
    prune_plan_sizes: list[int] = field(default_factory=list)
    best_path_source: str = ""
    best_path_size: int = 0
    best_path_forward_m: float = 0.0
    best_path_lateral_m: float = 0.0
    best_path_lateral_ratio: float = 0.0
    feedback_count: int = 0
    goal_accepted: bool = False
    cancel_accepted: bool = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "No-motion dry-run goal probe. Requires go2_xt16_navigation.launch "
            "with /cmd_vel remapped away from the real robot."
        )
    )
    parser.add_argument("--distance", type=float, default=0.5, help="Target distance ahead of base_link in meters")
    parser.add_argument("--heading-offset-deg", type=float, default=0.0, help="Relative target direction offset")
    parser.add_argument("--target-x", type=float, default=None, help="Absolute map-frame target x")
    parser.add_argument("--target-y", type=float, default=None, help="Absolute map-frame target y")
    parser.add_argument("--target-z", type=float, default=None, help="Absolute map-frame target z; defaults to current z")
    parser.add_argument("--target-yaw-deg", type=float, default=None, help="Target yaw; defaults to target direction")
    parser.add_argument("--duration", type=float, default=8.0, help="Seconds to collect output after goal acceptance")
    parser.add_argument("--tf-timeout-sec", type=float, default=8.0)
    parser.add_argument("--action-timeout-sec", type=float, default=8.0)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--dry-topic", default="/dddmr_go2/dry_run_cmd_vel")
    parser.add_argument("--safe-topic", default="/dddmr_go2/safe_cmd_vel")
    parser.add_argument("--decision-topic", default="/dddmr_go2/p2p_decision")
    parser.add_argument("--global-path-topic", default="/global_path")
    parser.add_argument("--awared-path-topic", default="/awared_global_path")
    parser.add_argument("--prune-plan-topic", default="/prune_plan")
    parser.add_argument("--zero-epsilon", type=float, default=0.001)
    parser.add_argument("--max-allowed-x", type=float, default=0.30)
    parser.add_argument("--max-allowed-y", type=float, default=0.0)
    parser.add_argument("--max-allowed-yaw", type=float, default=0.25)
    parser.add_argument("--min-path-forward", type=float, default=0.05)
    parser.add_argument("--max-path-lateral-ratio", type=float, default=1.5)
    return parser.parse_args()


def quaternion_to_yaw(quat) -> float:
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class DryRunGoalProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("probe_go2_xt16_dry_run_goal")
        self.args = args
        self.state = ProbeState()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, PToPMoveBase, "p2p_move_base")
        self.reference_x: float | None = None
        self.reference_y: float | None = None
        self.reference_yaw: float | None = None
        self.create_subscription(Twist, args.dry_topic, self._dry_cb, 20)
        self.create_subscription(Twist, args.safe_topic, self._safe_cb, 20)
        self.create_subscription(String, args.decision_topic, self._decision_cb, 20)
        self.create_subscription(Path, args.global_path_topic, self._global_path_cb, 10)
        self.create_subscription(Path, args.awared_path_topic, self._awared_path_cb, 10)
        self.create_subscription(Path, args.prune_plan_topic, self._prune_plan_cb, 10)

    def _dry_cb(self, msg: Twist) -> None:
        self.state.dry.add(msg, self.args.zero_epsilon)

    def _safe_cb(self, msg: Twist) -> None:
        self.state.safe.add(msg, self.args.zero_epsilon)

    def _decision_cb(self, msg: String) -> None:
        if not self.state.decisions or self.state.decisions[-1] != msg.data:
            self.state.decisions.append(msg.data)

    def _global_path_cb(self, msg: Path) -> None:
        self.state.global_path_sizes.append(len(msg.poses))
        self._record_path("global_path", msg)

    def _awared_path_cb(self, msg: Path) -> None:
        self.state.awared_path_sizes.append(len(msg.poses))
        self._record_path("awared_global_path", msg)

    def _prune_plan_cb(self, msg: Path) -> None:
        self.state.prune_plan_sizes.append(len(msg.poses))
        self._record_path("prune_plan", msg)

    def _record_path(self, source: str, msg: Path) -> None:
        if self.reference_x is None or self.reference_y is None or self.reference_yaw is None:
            return
        if len(msg.poses) < 2:
            return
        first = msg.poses[0].pose.position
        last = msg.poses[-1].pose.position
        # Use the path's own span rather than the exact base pose because DDDMR
        # may snap the start and goal to nearby graph/ground points.
        dx = float(last.x - first.x)
        dy = float(last.y - first.y)
        forward = dx * math.cos(self.reference_yaw) + dy * math.sin(self.reference_yaw)
        lateral = -dx * math.sin(self.reference_yaw) + dy * math.cos(self.reference_yaw)
        lateral_ratio = abs(lateral) / max(abs(forward), 1e-6)
        if len(msg.poses) >= self.state.best_path_size:
            self.state.best_path_source = source
            self.state.best_path_size = len(msg.poses)
            self.state.best_path_forward_m = forward
            self.state.best_path_lateral_m = lateral
            self.state.best_path_lateral_ratio = lateral_ratio

    def lookup_current_pose(self):
        deadline = time.monotonic() + self.args.tf_timeout_sec
        last_error = ""
        while time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(self.args.map_frame, self.args.base_frame, Time())
            except Exception as exc:  # tf2 exposes multiple exception types.
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"could not lookup {self.args.map_frame}->{self.args.base_frame}: {last_error}")

    def make_goal(self, transform) -> PoseStamped:
        trans = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        target_heading = yaw + math.radians(self.args.heading_offset_deg)
        goal = PoseStamped()
        goal.header.frame_id = self.args.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        if self.args.target_x is None and self.args.target_y is None:
            goal.pose.position.x = trans.x + self.args.distance * math.cos(target_heading)
            goal.pose.position.y = trans.y + self.args.distance * math.sin(target_heading)
            goal.pose.position.z = trans.z
        else:
            goal.pose.position.x = self.args.target_x
            goal.pose.position.y = self.args.target_y
            goal.pose.position.z = trans.z if self.args.target_z is None else self.args.target_z
        target_yaw = (
            math.radians(self.args.target_yaw_deg)
            if self.args.target_yaw_deg is not None
            else math.atan2(goal.pose.position.y - trans.y, goal.pose.position.x - trans.x)
        )
        goal.pose.orientation = yaw_to_quaternion(target_yaw)
        return goal

    def send_goal_and_collect(self, goal: PoseStamped) -> None:
        if not self.action_client.wait_for_server(timeout_sec=self.args.action_timeout_sec):
            raise RuntimeError("p2p_move_base action server not available")

        action_goal = PToPMoveBase.Goal()
        action_goal.target_pose = goal
        send_future = self.action_client.send_goal_async(action_goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.args.action_timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("p2p_move_base goal rejected")
        self.state.goal_accepted = True

        deadline = time.monotonic() + self.args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        cancel_response = cancel_future.result()
        self.state.cancel_accepted = bool(cancel_response and cancel_response.goals_canceling)

    def _feedback_cb(self, _feedback_msg) -> None:
        self.state.feedback_count += 1


def bool_text(value: bool) -> str:
    return "true" if value else "false"


def limit_ok(stats: TwistStats, args: argparse.Namespace) -> bool:
    return (
        stats.max_abs_x <= args.max_allowed_x + 1e-6
        and stats.max_abs_y <= args.max_allowed_y + 1e-6
        and stats.max_abs_yaw <= args.max_allowed_yaw + 1e-6
    )


def main() -> int:
    args = parse_args()
    absolute_target = args.target_x is not None or args.target_y is not None
    if absolute_target and (args.target_x is None or args.target_y is None):
        raise SystemExit("--target-x and --target-y must be supplied together")
    if not absolute_target and args.distance <= 0.0:
        raise SystemExit("--distance must be > 0")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0")

    rclpy.init()
    node = DryRunGoalProbe(args)
    try:
        transform = node.lookup_current_pose()
        start = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        node.reference_x = float(start.x)
        node.reference_y = float(start.y)
        goal = node.make_goal(transform)
        target_dx = float(goal.pose.position.x - start.x)
        target_dy = float(goal.pose.position.y - start.y)
        target_distance = math.hypot(target_dx, target_dy)
        if target_distance <= 1e-6:
            raise RuntimeError("target is too close to current pose")
        node.reference_yaw = math.atan2(target_dy, target_dx)
        node.send_goal_and_collect(goal)
    finally:
        state = node.state
        node.destroy_node()
        rclpy.shutdown()

    dry_limit_ok = limit_ok(state.dry, args)
    safe_limit_ok = limit_ok(state.safe, args)
    got_global_path = bool(state.global_path_sizes and max(state.global_path_sizes) > 1)
    got_awared_path = bool(state.awared_path_sizes and max(state.awared_path_sizes) > 1)
    got_prune = bool(state.prune_plan_sizes and max(state.prune_plan_sizes) > 1)
    got_path = got_global_path or got_awared_path or got_prune
    path_direction_ok = (
        state.best_path_size > 1
        and state.best_path_forward_m > args.min_path_forward
        and state.best_path_lateral_ratio <= args.max_path_lateral_ratio
    )
    got_dry_nonzero = state.dry.nonzero_count > 0
    got_safe_samples = state.safe.count > 0
    passed = (
        state.goal_accepted
        and state.cancel_accepted
        and got_path
        and path_direction_ok
        and got_dry_nonzero
        and dry_limit_ok
        and safe_limit_ok
    )

    print(f"DRY_RUN_GOAL_START_X={start.x:.6f}")
    print(f"DRY_RUN_GOAL_START_Y={start.y:.6f}")
    print(f"DRY_RUN_GOAL_START_Z={start.z:.6f}")
    print(f"DRY_RUN_GOAL_START_YAW_DEG={math.degrees(yaw):.3f}")
    print(f"DRY_RUN_GOAL_TARGET_X={goal.pose.position.x:.6f}")
    print(f"DRY_RUN_GOAL_TARGET_Y={goal.pose.position.y:.6f}")
    print(f"DRY_RUN_GOAL_TARGET_Z={goal.pose.position.z:.6f}")
    print(f"DRY_RUN_GOAL_MODE={'absolute' if absolute_target else 'relative'}")
    print(f"DRY_RUN_GOAL_DISTANCE_M={target_distance:.3f}")
    print(f"DRY_RUN_GOAL_HEADING_OFFSET_DEG={args.heading_offset_deg:.3f}")
    print(f"DRY_RUN_PATH_REFERENCE_HEADING_DEG={math.degrees(node.reference_yaw):.3f}")
    print(f"DRY_RUN_GOAL_ACCEPTED={bool_text(state.goal_accepted)}")
    print(f"DRY_RUN_GOAL_CANCEL_ACCEPTED={bool_text(state.cancel_accepted)}")
    print(f"DRY_RUN_FEEDBACK_COUNT={state.feedback_count}")
    print(f"DRY_RUN_DECISIONS={','.join(state.decisions)}")
    print(f"DRY_RUN_GLOBAL_PATH_MAX_SIZE={max(state.global_path_sizes) if state.global_path_sizes else 0}")
    print(f"DRY_RUN_AWARED_PATH_MAX_SIZE={max(state.awared_path_sizes) if state.awared_path_sizes else 0}")
    print(f"DRY_RUN_PRUNE_PLAN_MAX_SIZE={max(state.prune_plan_sizes) if state.prune_plan_sizes else 0}")
    print(f"DRY_RUN_BEST_PATH_SOURCE={state.best_path_source}")
    print(f"DRY_RUN_BEST_PATH_SIZE={state.best_path_size}")
    print(f"DRY_RUN_PATH_FORWARD_M={state.best_path_forward_m:.6f}")
    print(f"DRY_RUN_PATH_LATERAL_M={state.best_path_lateral_m:.6f}")
    print(f"DRY_RUN_PATH_LATERAL_RATIO={state.best_path_lateral_ratio:.6f}")
    print(f"DRY_RUN_PATH_DIRECTION_OK={bool_text(path_direction_ok)}")
    print(f"DRY_RUN_CMD_COUNT={state.dry.count}")
    print(f"DRY_RUN_CMD_NONZERO_COUNT={state.dry.nonzero_count}")
    print(f"DRY_RUN_CMD_MAX_ABS_X={state.dry.max_abs_x:.6f}")
    print(f"DRY_RUN_CMD_MAX_ABS_Y={state.dry.max_abs_y:.6f}")
    print(f"DRY_RUN_CMD_MAX_ABS_YAW={state.dry.max_abs_yaw:.6f}")
    print(f"DRY_RUN_CMD_MAX_FORWARD_X={state.dry.max_forward_x:.6f}")
    print(f"DRY_RUN_CMD_MIN_FORWARD_X={state.dry.min_forward_x:.6f}")
    print(f"DRY_RUN_CMD_LIMIT_OK={bool_text(dry_limit_ok)}")
    print(f"SAFE_CMD_COUNT={state.safe.count}")
    print(f"SAFE_CMD_NONZERO_COUNT={state.safe.nonzero_count}")
    print(f"SAFE_CMD_MAX_ABS_X={state.safe.max_abs_x:.6f}")
    print(f"SAFE_CMD_MAX_ABS_Y={state.safe.max_abs_y:.6f}")
    print(f"SAFE_CMD_MAX_ABS_YAW={state.safe.max_abs_yaw:.6f}")
    print(f"SAFE_CMD_LIMIT_OK={bool_text(safe_limit_ok)}")
    print(f"DRY_RUN_GOT_PATH={bool_text(got_path)}")
    print(f"DRY_RUN_GOT_GLOBAL_PATH={bool_text(got_global_path)}")
    print(f"DRY_RUN_GOT_AWARED_PATH={bool_text(got_awared_path)}")
    print(f"DRY_RUN_GOT_PRUNE_PLAN={bool_text(got_prune)}")
    print(f"DRY_RUN_GOT_SAFE_SAMPLES={bool_text(got_safe_samples)}")
    print(f"DRY_RUN_GOAL_STATUS={'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
