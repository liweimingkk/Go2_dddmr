#!/usr/bin/python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from go2_nav_gate_policy import (
    localization_block_reason,
    localization_health_block_reason,
)


class Go2NavCmdGate(Node):
    def __init__(self) -> None:
        super().__init__("go2_nav_cmd_gate")

        self.input_topic = self.declare_parameter(
            "input_topic", "/dddmr_go2/dry_run_cmd_vel"
        ).get_parameter_value().string_value
        self.output_topic = self.declare_parameter(
            "output_topic", "/dddmr_go2/safe_cmd_vel"
        ).get_parameter_value().string_value
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.cmd_timeout_sec = float(
            self.declare_parameter("cmd_timeout_sec", 0.20).value
        )
        self.max_x = float(self.declare_parameter("max_x", 0.10).value)
        self.max_y = float(self.declare_parameter("max_y", 0.0).value)
        self.max_yaw = float(self.declare_parameter("max_yaw", 0.25).value)
        self.zero_epsilon = float(self.declare_parameter("zero_epsilon", 0.001).value)
        self.log_period_sec = float(
            self.declare_parameter("log_period_sec", 0.50).value
        )
        self.require_localization_tracking = bool(
            self.declare_parameter("require_localization_tracking", False).value
        )
        self.localization_status_topic = self.declare_parameter(
            "localization_status_topic", "/localization_status"
        ).get_parameter_value().string_value
        self.localization_status_timeout_sec = float(
            self.declare_parameter("localization_status_timeout_sec", 0.75).value
        )
        self.require_localization_health = bool(
            self.declare_parameter("require_localization_health", False).value
        )
        self.localization_health_topic = self.declare_parameter(
            "localization_health_topic", "/localization_health"
        ).get_parameter_value().string_value
        self.localization_health_timeout_sec = float(
            self.declare_parameter("localization_health_timeout_sec", 0.75).value
        )

        self.latest_cmd: Optional[Twist] = None
        self.last_cmd_time = None
        self.localization_status: Optional[str] = None
        self.last_localization_status_time = None
        self.localization_health: Optional[str] = None
        self.last_localization_health_time = None
        self.last_log_time = self.get_clock().now()
        self.last_output_key = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_cb, 10)
        self.create_subscription(
            String,
            self.localization_status_topic,
            self.localization_status_cb,
            10,
        )
        self.create_subscription(
            String,
            self.localization_health_topic,
            self.localization_health_cb,
            10,
        )
        self.create_timer(self.timer_period_sec(), self.timer_cb)

        self.get_logger().warn(
            "Go2 nav cmd gate: %s -> %s enabled=%s max_x=%.3f max_y=%.3f "
            "max_yaw=%.3f publish_rate_hz=%.3f timeout=%.3f "
            "require_localization_tracking=%s localization_timeout=%.3f "
            "require_localization_health=%s health_timeout=%.3f"
            % (
                self.input_topic,
                self.output_topic,
                self.enabled,
                self.max_x,
                self.max_y,
                self.max_yaw,
                self.publish_rate_hz,
                self.cmd_timeout_sec,
                self.require_localization_tracking,
                self.localization_status_timeout_sec,
                self.require_localization_health,
                self.localization_health_timeout_sec,
            )
        )

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def localization_status_cb(self, msg: String) -> None:
        self.localization_status = msg.data
        self.last_localization_status_time = self.get_clock().now()

    def localization_health_cb(self, msg: String) -> None:
        self.localization_health = msg.data
        self.last_localization_health_time = self.get_clock().now()

    def timer_cb(self) -> None:
        now = self.get_clock().now()
        reason = "pass"
        localization_reason = self.localization_block_reason(now)
        if localization_reason is None:
            localization_reason = self.localization_health_block_reason(now)
        if not self.enabled:
            output = Twist()
            reason = "disabled"
        elif localization_reason is not None:
            output = Twist()
            reason = localization_reason
        elif self.latest_cmd is None or self.last_cmd_time is None:
            output = Twist()
            reason = "no_input"
        else:
            age = (now - self.last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout_sec:
                output = Twist()
                reason = "timeout %.3fs" % age
            else:
                output = self.clamp_twist(self.latest_cmd)

        self.pub.publish(output)
        self.log_if_needed(now, output, reason)

    def localization_block_reason(self, now) -> Optional[str]:
        status_age_sec = None
        if self.last_localization_status_time is not None:
            status_age_sec = (
                now - self.last_localization_status_time
            ).nanoseconds / 1e9
        return localization_block_reason(
            self.require_localization_tracking,
            self.localization_status,
            status_age_sec,
            self.localization_status_timeout_sec,
        )

    def clamp_twist(self, msg: Twist) -> Twist:
        output = Twist()
        output.linear.x = self.zero_or_clamp(msg.linear.x, -self.max_x, self.max_x)
        output.linear.y = self.zero_or_clamp(msg.linear.y, -self.max_y, self.max_y)
        output.linear.z = 0.0
        output.angular.x = 0.0
        output.angular.y = 0.0
        output.angular.z = self.zero_or_clamp(
            msg.angular.z, -self.max_yaw, self.max_yaw
        )
        return output

    def localization_health_block_reason(self, now) -> Optional[str]:
        health_age_sec = None
        if self.last_localization_health_time is not None:
            health_age_sec = (
                now - self.last_localization_health_time
            ).nanoseconds / 1e9
        return localization_health_block_reason(
            self.require_localization_health,
            self.localization_health,
            health_age_sec,
            self.localization_health_timeout_sec,
        )

    def log_if_needed(self, now, output: Twist, reason: str) -> None:
        output_key = (
            round(output.linear.x, 4),
            round(output.linear.y, 4),
            round(output.angular.z, 4),
            reason.split(" ", 1)[0],
        )
        should_log = (now - self.last_log_time).nanoseconds / 1e9 >= self.log_period_sec
        if output_key != self.last_output_key or should_log:
            self.get_logger().info(
                "safe_cmd_vel reason=%s x=%.3f y=%.3f yaw=%.3f"
                % (reason, output.linear.x, output.linear.y, output.angular.z)
            )
            self.last_output_key = output_key
            self.last_log_time = now

    def zero_or_clamp(self, value: float, low: float, high: float) -> float:
        if not math.isfinite(value) or abs(value) <= self.zero_epsilon:
            return 0.0
        result = max(low, min(high, value))
        return 0.0 if result == 0.0 else result

    def timer_period_sec(self) -> float:
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "Invalid publish_rate_hz=%.3f; falling back to 50.0"
                % self.publish_rate_hz
            )
            self.publish_rate_hz = 50.0
        return 1.0 / self.publish_rate_hz


def main() -> None:
    rclpy.init()
    node = Go2NavCmdGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
