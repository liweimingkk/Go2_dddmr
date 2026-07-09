#!/usr/bin/python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


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

        self.latest_cmd: Optional[Twist] = None
        self.last_cmd_time = None
        self.last_log_time = self.get_clock().now()
        self.last_output_key = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_cb, 10)
        self.create_timer(self.timer_period_sec(), self.timer_cb)

        self.get_logger().warn(
            "Go2 nav cmd gate: %s -> %s enabled=%s max_x=%.3f max_y=%.3f "
            "max_yaw=%.3f publish_rate_hz=%.3f timeout=%.3f"
            % (
                self.input_topic,
                self.output_topic,
                self.enabled,
                self.max_x,
                self.max_y,
                self.max_yaw,
                self.publish_rate_hz,
                self.cmd_timeout_sec,
            )
        )

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_cb(self) -> None:
        now = self.get_clock().now()
        reason = "pass"
        if not self.enabled:
            output = Twist()
            reason = "disabled"
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
