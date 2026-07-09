#!/usr/bin/python3

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003


class Go2SportCmdVelDryRun(Node):
    def __init__(self) -> None:
        super().__init__("go2_sport_cmd_vel_dry_run")

        self.cmd_vel_topic = self.declare_parameter(
            "cmd_vel_topic", "/dddmr_go2/dry_run_cmd_vel"
        ).get_parameter_value().string_value
        self.axis_mode = self.declare_parameter(
            "axis_mode", "standard"
        ).get_parameter_value().string_value

        self.max_x = self.declare_parameter("max_x", 0.10).value
        self.max_y = self.declare_parameter("max_y", 0.0).value
        self.max_yaw = self.declare_parameter("max_yaw", 0.25).value
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.cmd_timeout_sec = float(
            self.declare_parameter("cmd_timeout_sec", 0.20).value
        )
        self.cmd_timeout_sec = float(
            self.declare_parameter("stale_timeout_sec", self.cmd_timeout_sec).value
        )
        self.zero_epsilon = float(self.declare_parameter("zero_epsilon", 0.001).value)
        self.stop_keepalive_hz = float(
            self.declare_parameter("stop_keepalive_hz", 2.0).value
        )
        self.linear_deadband = self.declare_parameter("linear_deadband", 0.01).value
        self.angular_deadband = self.declare_parameter("angular_deadband", 0.02).value
        self.x_sign = self.declare_parameter("x_sign", 1.0).value
        self.y_sign = self.declare_parameter("y_sign", 1.0).value
        self.yaw_sign = self.declare_parameter("yaw_sign", 1.0).value
        self.log_period_sec = self.declare_parameter("log_period_sec", 0.50).value
        self.enable_yaw_arc_shim = bool(
            self.declare_parameter("enable_yaw_arc_shim", False).value
        )
        self.yaw_arc_shim_mode = self.declare_parameter(
            "yaw_arc_shim_mode", "off"
        ).get_parameter_value().string_value
        if self.yaw_arc_shim_mode not in ("off", "preview", "live"):
            self.get_logger().warn(
                "Unknown yaw_arc_shim_mode=%s, falling back to off"
                % self.yaw_arc_shim_mode
            )
            self.yaw_arc_shim_mode = "off"
        self.yaw_arc_forward_x = float(
            self.declare_parameter("yaw_arc_forward_x", 0.03).value
        )
        self.yaw_arc_min_abs_yaw = float(
            self.declare_parameter("yaw_arc_min_abs_yaw", 0.20).value
        )
        self.yaw_arc_trigger_abs_yaw = float(
            self.declare_parameter("yaw_arc_trigger_abs_yaw", 0.03).value
        )
        self.yaw_arc_allowed_decisions = self.parse_allowed_decisions(
            self.declare_parameter(
                "yaw_arc_allowed_decisions",
                "d_align_heading",
            ).value
        )
        self.decision_topic = self.declare_parameter(
            "decision_topic", "/dddmr_go2/p2p_decision"
        ).get_parameter_value().string_value
        self.decision_timeout_sec = float(
            self.declare_parameter("decision_timeout_sec", 0.30).value
        )
        self.zero_yaw_only_when_shim_disallowed = bool(
            self.declare_parameter("zero_yaw_only_when_shim_disallowed", True).value
        )
        self.max_continuous_yaw_arc_sec = float(
            self.declare_parameter("max_continuous_yaw_arc_sec", 4.0).value
        )

        self.latest_cmd: Optional[Twist] = None
        self.last_cmd_time = None
        self.current_decision = ""
        self.last_decision_time = None
        self.last_log_time = self.get_clock().now()
        self.stop_logged = True
        self.in_stop_state = True
        self.last_stop_time = None
        self.yaw_arc_started_time = None
        self.yaw_arc_fault_latched = False

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10)
        self.create_subscription(String, self.decision_topic, self.decision_cb, 10)
        self.create_timer(self.timer_period_sec(), self.publish_timer_cb)

        self.get_logger().warn(
            "DRY RUN ONLY: subscribing to %s and logging Unitree Sport Move "
            "requests; no /api/sport/request publisher is created."
            % self.cmd_vel_topic
        )
        self.get_logger().info(
            "limits: max_x=%.3f max_y=%.3f max_yaw=%.3f axis_mode=%s "
            "publish_rate_hz=%.3f cmd_timeout_sec=%.3f zero_epsilon=%.4f "
            "stop_keepalive_hz=%.3f"
            % (
                self.max_x,
                self.max_y,
                self.max_yaw,
                self.axis_mode,
                self.publish_rate_hz,
                self.cmd_timeout_sec,
                self.zero_epsilon,
                self.stop_keepalive_hz,
            )
        )
        self.get_logger().info(
            "yaw_arc_shim: enabled=%s mode=%s forward_x=%.3f min_abs_yaw=%.3f "
            "trigger_abs_yaw=%.3f allowed_decisions=%s decision_topic=%s "
            "decision_timeout=%.3f zero_disallowed=%s max_continuous=%.3f"
            % (
                self.enable_yaw_arc_shim,
                self.yaw_arc_shim_mode,
                self.yaw_arc_forward_x,
                self.yaw_arc_min_abs_yaw,
                self.yaw_arc_trigger_abs_yaw,
                sorted(self.yaw_arc_allowed_decisions),
                self.decision_topic,
                self.decision_timeout_sec,
                self.zero_yaw_only_when_shim_disallowed,
                self.max_continuous_yaw_arc_sec,
            )
        )

    def decision_cb(self, msg: String) -> None:
        self.current_decision = msg.data
        self.last_decision_time = self.get_clock().now()

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def publish_timer_cb(self) -> None:
        now = self.get_clock().now()
        if self.latest_cmd is None or self.last_cmd_time is None:
            return

        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout_sec:
            self.handle_stop("cmd_vel timeout %.3fs" % age, now)
            return

        original = self.map_axis_and_clamp(self.latest_cmd)
        x, y, yaw, shim_info = self.apply_yaw_arc_shim(*original, now=now)
        if self.is_zero_command(x, y, yaw):
            self.handle_stop(
                "zero cmd_vel decision=%s original_sport=%s transformed_sport=%s shim=%s"
                % (
                    self.current_decision or "<none>",
                    self.format_payload(*original),
                    self.format_payload(x, y, yaw),
                    shim_info,
                ),
                now,
            )
            return

        self.stop_logged = False
        self.in_stop_state = False
        self.last_stop_time = None
        payload = {"x": x, "y": y, "z": yaw}
        if self.should_log(now):
            self.get_logger().info(
                "DRY_RUN would publish /api/sport/request: api_id=%d parameter=%s "
                "decision=%s original_sport=%s transformed_sport=%s shim=%s"
                % (
                    MOVE_API_ID,
                    json.dumps(payload, separators=(",", ":")),
                    self.current_decision or "<none>",
                    self.format_payload(*original),
                    self.format_payload(x, y, yaw),
                    shim_info,
                )
            )
            self.last_log_time = now

    def handle_stop(self, reason: str, now) -> None:
        if self.stop_logged and not self.stop_keepalive_due(now):
            return

        if self.should_log(now) or not self.stop_logged:
            self.get_logger().warn(
                "DRY_RUN %s: would publish /api/sport/request api_id=%d StopMove"
                % (reason, STOP_MOVE_API_ID)
            )
            self.last_log_time = now
        self.stop_logged = True
        self.in_stop_state = True
        self.last_stop_time = now

    def stop_keepalive_due(self, now) -> bool:
        if not self.in_stop_state:
            return False
        if self.stop_keepalive_hz <= 0.0:
            return False
        if self.last_cmd_time is None:
            return False
        if self.last_stop_time is None:
            return True
        interval = 1.0 / self.stop_keepalive_hz
        return (now - self.last_stop_time).nanoseconds / 1e9 >= interval

    def is_zero_command(self, x: float, y: float, yaw: float) -> bool:
        return (
            abs(x) <= self.zero_epsilon
            and abs(y) <= self.zero_epsilon
            and abs(yaw) <= self.zero_epsilon
        )

    def timer_period_sec(self) -> float:
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "Invalid publish_rate_hz=%.3f; falling back to 50.0"
                % self.publish_rate_hz
            )
            self.publish_rate_hz = 50.0
        return 1.0 / self.publish_rate_hz

    def should_log(self, now) -> bool:
        return (now - self.last_log_time).nanoseconds / 1e9 >= self.log_period_sec

    def to_sport_command(self, msg: Twist) -> Tuple[float, float, float]:
        x, y, yaw = self.map_axis_and_clamp(msg)
        x, y, yaw, _ = self.apply_yaw_arc_shim(x, y, yaw)
        return x, y, yaw

    def map_axis_and_clamp(self, msg: Twist) -> Tuple[float, float, float]:
        if self.axis_mode == "standard":
            x = msg.linear.x
            y = msg.linear.y
            yaw = msg.angular.z
        elif self.axis_mode == "go2_raw_y_forward":
            x = msg.linear.y
            y = -msg.linear.x
            yaw = msg.angular.z
        else:
            self.get_logger().warn(
                "Unknown axis_mode=%s, falling back to standard" % self.axis_mode
            )
            x = msg.linear.x
            y = msg.linear.y
            yaw = msg.angular.z

        x = self.apply_deadband(x * self.x_sign, self.linear_deadband)
        y = self.apply_deadband(y * self.y_sign, self.linear_deadband)
        yaw = self.apply_deadband(yaw * self.yaw_sign, self.angular_deadband)

        return (
            self.clamp(x, -self.max_x, self.max_x),
            self.clamp(y, -self.max_y, self.max_y),
            self.clamp(yaw, -self.max_yaw, self.max_yaw),
        )

    def apply_yaw_arc_shim(
        self, x: float, y: float, yaw: float, now=None
    ) -> Tuple[float, float, float, str]:
        if now is None:
            now = self.get_clock().now()

        yaw_only = (
            abs(x) <= self.linear_deadband
            and abs(y) <= self.linear_deadband
            and abs(yaw) >= self.yaw_arc_trigger_abs_yaw
        )
        zero_cmd = x == 0.0 and y == 0.0 and yaw == 0.0

        if zero_cmd or not yaw_only:
            self.yaw_arc_started_time = None
            self.yaw_arc_fault_latched = False

        if not self.enable_yaw_arc_shim or self.yaw_arc_shim_mode == "off" or not yaw_only:
            return x, y, yaw, "none"

        allowed, decision_info = self.is_decision_fresh_and_allowed(now)
        if not allowed:
            self.yaw_arc_started_time = None
            if (
                self.yaw_arc_shim_mode == "live"
                and self.zero_yaw_only_when_shim_disallowed
            ):
                return 0.0, 0.0, 0.0, "blocked_%s" % decision_info
            return x, y, yaw, "pass_no_shim_%s" % decision_info

        shim_yaw = math.copysign(max(abs(yaw), self.yaw_arc_min_abs_yaw), yaw)
        shim_yaw = self.clamp(shim_yaw, -self.max_yaw, self.max_yaw)
        shim_x = self.clamp(self.yaw_arc_forward_x, 0.0, self.max_x)

        if self.yaw_arc_shim_mode == "preview":
            self.yaw_arc_started_time = None
            return (
                x,
                y,
                yaw,
                "preview x=%.3f yaw=%.3f %s" % (shim_x, shim_yaw, decision_info),
            )

        if self.yaw_arc_fault_latched:
            return 0.0, 0.0, 0.0, "fault_yaw_arc_timeout_latched_%s" % decision_info

        if self.yaw_arc_started_time is None:
            self.yaw_arc_started_time = now

        elapsed = (now - self.yaw_arc_started_time).nanoseconds / 1e9
        if (
            self.max_continuous_yaw_arc_sec > 0.0
            and elapsed > self.max_continuous_yaw_arc_sec
        ):
            self.yaw_arc_fault_latched = True
            return (
                0.0,
                0.0,
                0.0,
                "fault_yaw_arc_timeout elapsed=%.3f limit=%.3f %s"
                % (elapsed, self.max_continuous_yaw_arc_sec, decision_info),
            )

        return (
            shim_x,
            0.0,
            shim_yaw,
            "live x=%.3f yaw=%.3f elapsed=%.3f %s"
            % (shim_x, shim_yaw, elapsed, decision_info),
        )

    def is_decision_fresh_and_allowed(self, now) -> Tuple[bool, str]:
        if self.last_decision_time is None:
            return False, "no_decision"

        age = (now - self.last_decision_time).nanoseconds / 1e9
        if age > self.decision_timeout_sec:
            return (
                False,
                "stale_decision=%s age=%.3f"
                % (self.current_decision or "<none>", age),
            )
        if self.current_decision not in self.yaw_arc_allowed_decisions:
            return (
                False,
                "blocked_state=%s age=%.3f"
                % (self.current_decision or "<none>", age),
            )
        return True, "decision=%s age=%.3f" % (self.current_decision, age)

    @staticmethod
    def parse_allowed_decisions(value) -> set:
        if isinstance(value, str):
            return {item.strip() for item in value.split(",") if item.strip()}
        try:
            return {str(item) for item in value}
        except TypeError:
            return {str(value)}

    @staticmethod
    def format_payload(x: float, y: float, yaw: float) -> str:
        return json.dumps({"x": x, "y": y, "z": yaw}, separators=(",", ":"))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        if not math.isfinite(value) or abs(value) < deadband:
            return 0.0
        return value

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        result = max(low, min(high, value))
        return 0.0 if result == 0.0 else result


def main() -> None:
    rclpy.init()
    node = Go2SportCmdVelDryRun()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
