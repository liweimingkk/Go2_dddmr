#!/usr/bin/env python3
"""Probe planner-only candidate goals without publishing velocity commands."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from dddmr_sys_core.action import GetPlan
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


@dataclass
class CandidateResult:
    index: int
    distance: float
    angle_deg: float
    target_x: float
    target_y: float
    target_z: float
    status: int
    accepted: bool
    path_size: int
    path_forward_m: float
    path_lateral_m: float
    path_lateral_ratio: float

    @property
    def passed(self) -> bool:
        return self.accepted and self.status == GoalStatus.STATUS_SUCCEEDED and self.path_size > 1


def parse_float_list(raw: str) -> list[float]:
    return [float(item.strip()) for item in raw.split(",") if item.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Planner-only Go2 XT16 candidate probe. It sends GetPlan action "
            "requests and never publishes velocity or sport commands."
        )
    )
    parser.add_argument("--action-name", default="/get_plan", help="Planner action, usually /get_plan or /get_dwa_plan")
    parser.add_argument("--distances", default="0.25,0.4,0.6,0.8,1.0")
    parser.add_argument("--angles-deg", default="-120,-90,-60,-30,0,30,60,90,120,150,180")
    parser.add_argument("--tf-timeout-sec", type=float, default=10.0)
    parser.add_argument("--action-timeout-sec", type=float, default=8.0)
    parser.add_argument("--result-timeout-sec", type=float, default=6.0)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--min-path-size", type=int, default=2)
    parser.add_argument("--min-path-forward", type=float, default=0.05)
    parser.add_argument("--max-path-lateral-ratio", type=float, default=2.0)
    parser.add_argument("--stop-after-first", action="store_true")
    return parser.parse_args()


def quaternion_to_yaw(quat) -> float:
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def status_name(status: int) -> str:
    names = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return names.get(status, str(status))


class PlanCandidateProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("probe_go2_xt16_plan_candidates")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, GetPlan, args.action_name)

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

    def make_pose(self, x: float, y: float, z: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.args.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def query(self, index: int, start: PoseStamped, target: PoseStamped, distance: float, angle_deg: float) -> CandidateResult:
        action_goal = GetPlan.Goal()
        action_goal.start = start
        action_goal.goal = target
        action_goal.activate_threading = True

        send_future = self.action_client.send_goal_async(action_goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.args.action_timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return CandidateResult(
                index=index,
                distance=distance,
                angle_deg=angle_deg,
                target_x=float(target.pose.position.x),
                target_y=float(target.pose.position.y),
                target_z=float(target.pose.position.z),
                status=GoalStatus.STATUS_UNKNOWN,
                accepted=False,
                path_size=0,
                path_forward_m=0.0,
                path_lateral_m=0.0,
                path_lateral_ratio=0.0,
            )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.result_timeout_sec)
        wrapped = result_future.result()
        if wrapped is None:
            return CandidateResult(
                index=index,
                distance=distance,
                angle_deg=angle_deg,
                target_x=float(target.pose.position.x),
                target_y=float(target.pose.position.y),
                target_z=float(target.pose.position.z),
                status=GoalStatus.STATUS_UNKNOWN,
                accepted=True,
                path_size=0,
                path_forward_m=0.0,
                path_lateral_m=0.0,
                path_lateral_ratio=0.0,
            )

        path = wrapped.result.path
        path_size = len(path.poses)
        forward = 0.0
        lateral = 0.0
        lateral_ratio = 0.0
        if path_size > 1:
            first = path.poses[0].pose.position
            last = path.poses[-1].pose.position
            dx = float(last.x - first.x)
            dy = float(last.y - first.y)
            ref_yaw = math.atan2(target.pose.position.y - start.pose.position.y, target.pose.position.x - start.pose.position.x)
            forward = dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)
            lateral = -dx * math.sin(ref_yaw) + dy * math.cos(ref_yaw)
            lateral_ratio = abs(lateral) / max(abs(forward), 1e-6)

        return CandidateResult(
            index=index,
            distance=distance,
            angle_deg=angle_deg,
            target_x=float(target.pose.position.x),
            target_y=float(target.pose.position.y),
            target_z=float(target.pose.position.z),
            status=int(wrapped.status),
            accepted=True,
            path_size=path_size,
            path_forward_m=forward,
            path_lateral_m=lateral,
            path_lateral_ratio=lateral_ratio,
        )


def candidate_ok(candidate: CandidateResult, args: argparse.Namespace) -> bool:
    return (
        candidate.passed
        and candidate.path_size >= args.min_path_size
        and candidate.path_forward_m > args.min_path_forward
        and candidate.path_lateral_ratio <= args.max_path_lateral_ratio
    )


def main() -> int:
    args = parse_args()
    distances = parse_float_list(args.distances)
    angles = parse_float_list(args.angles_deg)
    if not distances or not angles:
        raise SystemExit("--distances and --angles-deg must not be empty")

    rclpy.init()
    node = PlanCandidateProbe(args)
    best: CandidateResult | None = None
    try:
        if not node.action_client.wait_for_server(timeout_sec=args.action_timeout_sec):
            raise RuntimeError(f"planner action server not available: {args.action_name}")

        transform = node.lookup_current_pose()
        trans = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        start = node.make_pose(float(trans.x), float(trans.y), float(trans.z), yaw)

        print(f"PLAN_PROBE_ACTION={args.action_name}")
        print(f"PLAN_PROBE_START_X={trans.x:.6f}")
        print(f"PLAN_PROBE_START_Y={trans.y:.6f}")
        print(f"PLAN_PROBE_START_Z={trans.z:.6f}")
        print(f"PLAN_PROBE_START_YAW_DEG={math.degrees(yaw):.3f}")

        index = 0
        for distance in distances:
            if distance <= 0.0:
                continue
            for angle_deg in angles:
                target_yaw = yaw + math.radians(angle_deg)
                target = node.make_pose(
                    float(trans.x + distance * math.cos(target_yaw)),
                    float(trans.y + distance * math.sin(target_yaw)),
                    float(trans.z),
                    target_yaw,
                )
                candidate = node.query(index, start, target, distance, angle_deg)
                ok = candidate_ok(candidate, args)
                if ok and (
                    best is None
                    or candidate.path_size > best.path_size
                    or (candidate.path_size == best.path_size and candidate.distance < best.distance)
                ):
                    best = candidate

                print(
                    "PLAN_CANDIDATE "
                    f"index={candidate.index} "
                    f"distance={candidate.distance:.3f} "
                    f"angle_deg={candidate.angle_deg:.1f} "
                    f"target_x={candidate.target_x:.6f} "
                    f"target_y={candidate.target_y:.6f} "
                    f"target_z={candidate.target_z:.6f} "
                    f"accepted={str(candidate.accepted).lower()} "
                    f"status={status_name(candidate.status)} "
                    f"path_size={candidate.path_size} "
                    f"path_forward_m={candidate.path_forward_m:.6f} "
                    f"path_lateral_m={candidate.path_lateral_m:.6f} "
                    f"path_lateral_ratio={candidate.path_lateral_ratio:.6f} "
                    f"ok={str(ok).lower()}"
                )
                index += 1
                if ok and args.stop_after_first:
                    raise StopIteration

    except StopIteration:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if best is None:
        print("PLAN_BEST_STATUS=FAIL")
        return 1

    print("PLAN_BEST_STATUS=PASS")
    print(f"PLAN_BEST_INDEX={best.index}")
    print(f"PLAN_BEST_TARGET_X={best.target_x:.6f}")
    print(f"PLAN_BEST_TARGET_Y={best.target_y:.6f}")
    print(f"PLAN_BEST_TARGET_Z={best.target_z:.6f}")
    print(f"PLAN_BEST_DISTANCE_M={best.distance:.3f}")
    print(f"PLAN_BEST_ANGLE_DEG={best.angle_deg:.1f}")
    print(f"PLAN_BEST_PATH_SIZE={best.path_size}")
    print(f"PLAN_BEST_PATH_FORWARD_M={best.path_forward_m:.6f}")
    print(f"PLAN_BEST_PATH_LATERAL_RATIO={best.path_lateral_ratio:.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
