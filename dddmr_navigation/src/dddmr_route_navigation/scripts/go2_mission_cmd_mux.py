#!/usr/bin/env python3
"""Fail-closed velocity and decision mux for one-map outdoor/indoor missions."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from mission_command_mux_policy import select_command


class MissionCommandMux(Node):
    def __init__(self) -> None:
        super().__init__("go2_mission_cmd_mux")
        self.outdoor_command_topic = self._string_parameter(
            "outdoor_command_topic", "/dddmr_go2/outdoor_cmd_vel"
        )
        self.indoor_command_topic = self._string_parameter(
            "indoor_command_topic", "/dddmr_go2/indoor_cmd_vel"
        )
        self.outdoor_decision_topic = self._string_parameter(
            "outdoor_decision_topic", "/dddmr_go2/outdoor_decision"
        )
        self.indoor_decision_topic = self._string_parameter(
            "indoor_decision_topic", "/dddmr_go2/indoor_decision"
        )
        self.selection_topic = self._string_parameter(
            "selection_topic", "/outdoor_indoor_mission/control_mode"
        )
        self.output_command_topic = self._string_parameter(
            "output_command_topic", "/dddmr_go2/dry_run_cmd_vel"
        )
        self.output_decision_topic = self._string_parameter(
            "output_decision_topic", "/dddmr_go2/p2p_decision"
        )
        self.publish_rate_hz = self._positive_parameter("publish_rate_hz", 50.0)
        self.mode_timeout_sec = self._positive_parameter("mode_timeout_sec", 0.30)
        self.command_timeout_sec = self._positive_parameter("command_timeout_sec", 0.20)
        self.decision_timeout_sec = self._positive_parameter("decision_timeout_sec", 0.30)
        self.max_x = self._nonnegative_parameter("max_x", 0.35)
        self.max_y = self._nonnegative_parameter("max_y", 0.0)
        self.max_yaw = self._nonnegative_parameter("max_yaw", 0.50)
        self.zero_epsilon = self._nonnegative_parameter("zero_epsilon", 0.001)
        self.allowed_decisions = {
            "outdoor": self._decision_set(
                "outdoor_allowed_motion_decisions",
                "d_controlling,d_align_heading,d_align_goal_heading",
            ),
            "indoor": self._decision_set(
                "indoor_allowed_motion_decisions",
                "d_controlling,d_align_heading,d_align_goal_heading,d_recovery_waitdone",
            ),
        }

        self.mode = "none"
        self.mode_time: Optional[float] = None
        self.commands: Dict[str, Tuple[Tuple[float, float, float], float]] = {}
        self.decisions: Dict[str, Tuple[str, float]] = {}
        self.last_status = ""

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.command_publisher = self.create_publisher(
            Twist, self.output_command_topic, 10
        )
        self.decision_publisher = self.create_publisher(
            String, self.output_decision_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, "~/status", transient_qos
        )
        self.stopped_publisher = self.create_publisher(Bool, "~/stopped", 10)
        self.create_subscription(
            Twist,
            self.outdoor_command_topic,
            lambda message: self._command_callback("outdoor", message),
            10,
        )
        self.create_subscription(
            Twist,
            self.indoor_command_topic,
            lambda message: self._command_callback("indoor", message),
            10,
        )
        self.create_subscription(
            String,
            self.outdoor_decision_topic,
            lambda message: self._decision_callback("outdoor", message),
            10,
        )
        self.create_subscription(
            String,
            self.indoor_decision_topic,
            lambda message: self._decision_callback("indoor", message),
            10,
        )
        self.create_subscription(String, self.selection_topic, self._selection_callback, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self.get_logger().warn(
            "Mission command mux starts in NONE and requires a fresh mode heartbeat; "
            "stale mode, command, or decision input produces zero velocity"
        )

    def _string_parameter(self, name: str, default: str) -> str:
        value = str(self.declare_parameter(name, default).value).strip()
        if not value:
            raise ValueError(f"{name} must not be empty")
        return value

    def _positive_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _nonnegative_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _decision_set(self, name: str, default: str) -> Set[str]:
        value = self._string_parameter(name, default)
        decisions = {item.strip() for item in value.split(",") if item.strip()}
        if not decisions:
            raise ValueError(f"{name} must contain at least one decision")
        return decisions

    def _command_callback(self, mode: str, message: Twist) -> None:
        self.commands[mode] = (
            (message.linear.x, message.linear.y, message.angular.z),
            time.monotonic(),
        )

    def _decision_callback(self, mode: str, message: String) -> None:
        self.decisions[mode] = (message.data, time.monotonic())

    def _selection_callback(self, message: String) -> None:
        requested = message.data.strip().lower()
        if requested not in {"none", "outdoor", "indoor"}:
            self.get_logger().error("Rejecting invalid mission control mode %r" % message.data)
            requested = "none"
        if requested != self.mode and requested in {"outdoor", "indoor"}:
            # A newly selected controller must publish a command and decision
            # after the handoff. Never replay a still-fresh pair cached from a
            # previous mission or phase.
            self.commands.pop(requested, None)
            self.decisions.pop(requested, None)
        self.mode = requested
        self.mode_time = time.monotonic()

    def _timer_callback(self) -> None:
        now = time.monotonic()
        command, command_time = self.commands.get(self.mode, ((0.0, 0.0, 0.0), -math.inf))
        decision, decision_time = self.decisions.get(
            self.mode, ("d_mission_stopped", -math.inf)
        )
        mode_age = math.inf if self.mode_time is None else now - self.mode_time
        output, output_decision, reason = select_command(
            mode=self.mode,
            mode_age_sec=mode_age,
            command=command,
            command_age_sec=now - command_time,
            decision=decision,
            decision_age_sec=now - decision_time,
            mode_timeout_sec=self.mode_timeout_sec,
            command_timeout_sec=self.command_timeout_sec,
            decision_timeout_sec=self.decision_timeout_sec,
            allowed_motion_decisions=self.allowed_decisions.get(self.mode, set()),
            max_x=self.max_x,
            max_y=self.max_y,
            max_yaw=self.max_yaw,
            zero_epsilon=self.zero_epsilon,
        )
        command_message = Twist()
        command_message.linear.x = output[0]
        command_message.linear.y = output[1]
        command_message.angular.z = output[2]
        decision_message = String()
        decision_message.data = output_decision
        stopped_message = Bool()
        stopped_message.data = output == (0.0, 0.0, 0.0)
        self.command_publisher.publish(command_message)
        self.decision_publisher.publish(decision_message)
        self.stopped_publisher.publish(stopped_message)

        status = json.dumps(
            {
                "mode": self.mode,
                "reason": reason,
                "decision": output_decision,
                "stopped": stopped_message.data,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        if status != self.last_status:
            status_message = String()
            status_message.data = status
            self.status_publisher.publish(status_message)
            self.get_logger().info(status)
            self.last_status = status


def main() -> None:
    rclpy.init()
    node = MissionCommandMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
