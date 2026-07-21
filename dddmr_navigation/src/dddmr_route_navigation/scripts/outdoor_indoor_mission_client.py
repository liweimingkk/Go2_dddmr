#!/usr/bin/env python3
"""Send one outdoor-route plus indoor-goal mission action."""

from __future__ import annotations

import argparse
import math
from typing import Optional, Sequence

import rclpy
from dddmr_sys_core.action import OutdoorIndoorMission
from rclpy.action import ActionClient
from rclpy.node import Node


class MissionClient(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__("outdoor_indoor_mission_client")
        self.client = ActionClient(self, OutdoorIndoorMission, action_name)
        self.exit_code = 1

    def send(self, args: argparse.Namespace) -> None:
        if not self.client.wait_for_server(timeout_sec=args.wait_timeout):
            raise RuntimeError(f"mission action unavailable: {args.action_name}")
        goal = OutdoorIndoorMission.Goal()
        goal.mission_id = args.mission_id
        goal.route_id = args.route_id
        goal.indoor_goal.header.frame_id = args.frame_id
        goal.indoor_goal.header.stamp = self.get_clock().now().to_msg()
        goal.indoor_goal.pose.position.x = args.x
        goal.indoor_goal.pose.position.y = args.y
        goal.indoor_goal.pose.position.z = args.z
        goal.indoor_goal.pose.orientation.z = math.sin(args.yaw * 0.5)
        goal.indoor_goal.pose.orientation.w = math.cos(args.yaw * 0.5)
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(self.accepted)

    def feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "phase=%s outdoor_progress=%.3f detail=%s"
            % (feedback.phase, feedback.outdoor_progress, feedback.detail)
        )

    def accepted(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("mission goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("mission goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.finished)

    def finished(self, future) -> None:
        wrapped = future.result()
        result = wrapped.result
        self.exit_code = 0 if result.status == OutdoorIndoorMission.Result.SUCCESS else 1
        self.get_logger().info(
            "mission result action_status=%s result_status=%s message=%s"
            % (wrapped.status, result.status, result.message)
        )
        rclpy.shutdown()


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("mission_id")
    value.add_argument("route_id")
    value.add_argument("x", type=float)
    value.add_argument("y", type=float)
    value.add_argument("z", type=float)
    value.add_argument("yaw", type=float)
    value.add_argument("--frame-id", default="map")
    value.add_argument("--action-name", default="/outdoor_indoor_mission")
    value.add_argument("--wait-timeout", type=float, default=10.0)
    return value


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    for name in ("x", "y", "z", "yaw", "wait_timeout"):
        if not math.isfinite(getattr(args, name)):
            raise SystemExit(f"{name} must be finite")
    if args.wait_timeout <= 0.0:
        raise SystemExit("wait_timeout must be positive")
    rclpy.init()
    node = MissionClient(args.action_name)
    try:
        node.send(args)
        rclpy.spin(node)
        return node.exit_code
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
