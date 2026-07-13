#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from go2_gait_monitor_policy import GaitStateContract

try:
    from unitree_go.msg import SportModeState
except ImportError as exc:  # Host-only Unitree dependency; fail clearly at runtime.
    raise RuntimeError(
        "go2_gait_state_monitor.py requires unitree_go/msg/SportModeState "
        "from the Go2 host ROS environment"
    ) from exc


class Go2GaitStateMonitor(Node):
    """Report whether Sport mode/gait remain at a latched or configured value.

    This node is read-only. It never publishes a Unitree request or changes gait.
    """

    def __init__(self) -> None:
        super().__init__("go2_gait_state_monitor")
        self.input_topic = self.declare_parameter(
            "input_topic", "/sportmodestate"
        ).get_parameter_value().string_value
        self.output_topic = self.declare_parameter(
            "output_topic", "/dddmr_go2/gait_unchanged"
        ).get_parameter_value().string_value
        self.reason_topic = self.declare_parameter(
            "reason_topic", "/dddmr_go2/gait_monitor_reason"
        ).get_parameter_value().string_value
        self.expected_mode = int(self.declare_parameter("expected_mode", -1).value)
        self.expected_gait_type = int(
            self.declare_parameter("expected_gait_type", -1).value
        )
        self.timeout_sec = float(
            self.declare_parameter("status_timeout_sec", 0.30).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 20.0).value
        )
        if (
            not math.isfinite(self.timeout_sec)
            or self.timeout_sec <= 0.0
            or not math.isfinite(self.publish_rate_hz)
            or self.publish_rate_hz <= 0.0
        ):
            raise ValueError("gait monitor timeout and rate must be finite and positive")

        self.contract = GaitStateContract(
            expected_mode=self.expected_mode,
            expected_gait_type=self.expected_gait_type,
            timeout_sec=self.timeout_sec,
        )
        self.last_reported_reason = ""

        self.state_pub = self.create_publisher(Bool, self.output_topic, 10)
        self.reason_pub = self.create_publisher(String, self.reason_topic, 10)
        self.create_subscription(SportModeState, self.input_topic, self.status_cb, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self.timer_cb)

        self.get_logger().warn(
            "Read-only gait monitor: input=%s output=%s expected_mode=%d "
            "expected_gait_type=%d timeout=%.3f"
            % (
                self.input_topic,
                self.output_topic,
                self.expected_mode,
                self.expected_gait_type,
                self.timeout_sec,
            )
        )

    def status_cb(self, msg: SportModeState) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.contract.observe(int(msg.mode), int(msg.gait_type), now_sec)

    def timer_cb(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        result = self.contract.evaluate(now_sec)

        state_msg = Bool()
        state_msg.data = result.unchanged
        self.state_pub.publish(state_msg)
        reason_msg = String()
        reason_msg.data = result.reason
        self.reason_pub.publish(reason_msg)
        if result.reason != self.last_reported_reason:
            message = (
                "NORMAL_GAIT_CONTRACT reason=%s baseline_mode=%s "
                "baseline_gait_type=%s current_mode=%s current_gait_type=%s"
                % (
                    result.reason,
                    result.baseline_mode,
                    result.baseline_gait_type,
                    result.current_mode,
                    result.current_gait_type,
                )
            )
            if result.reason in ("mode_changed", "gait_changed", "invalid_status"):
                self.get_logger().error(message)
            elif result.unchanged:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
            self.last_reported_reason = result.reason


def main() -> None:
    rclpy.init()
    node = Go2GaitStateMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
