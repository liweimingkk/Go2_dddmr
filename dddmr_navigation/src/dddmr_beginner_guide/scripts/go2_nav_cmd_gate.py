#!/usr/bin/python3

import math
from typing import Optional

import rclpy
from dddmr_sys_core.msg import TerrainStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from go2_nav_gate_policy import (
    MotionProgressMonitor,
    TerrainFaultLatch,
    TerrainGateLimits,
    TerrainGateStatus,
    localization_block_reason,
    terrain_motion_gate,
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
        self.require_terrain_safe = bool(
            self.declare_parameter("require_terrain_safe", False).value
        )
        self.terrain_status_topic = self.declare_parameter(
            "terrain_status_topic", "/dddmr_terrain/supervised_status"
        ).get_parameter_value().string_value
        self.terrain_status_timeout_sec = float(
            self.declare_parameter("terrain_status_timeout_sec", 0.30).value
        )
        self.terrain_min_confidence = float(
            self.declare_parameter("terrain_min_confidence", 0.90).value
        )
        self.terrain_min_support_ratio = float(
            self.declare_parameter("terrain_min_support_ratio", 0.80).value
        )
        self.stair_max_heading_error_rad = float(
            self.declare_parameter("stair_max_heading_error_rad", 0.13962634).value
        )
        self.stair_max_lateral_error_m = float(
            self.declare_parameter("stair_max_lateral_error_m", 0.10).value
        )
        self.stair_align_max_forward_mps = float(
            self.declare_parameter("stair_align_max_forward_mps", 0.0).value
        )
        self.stair_align_max_yaw_rps = float(
            self.declare_parameter("stair_align_max_yaw_rps", 0.25).value
        )
        self.stair_committed_max_yaw_rps = float(
            self.declare_parameter("stair_committed_max_yaw_rps", 0.0).value
        )
        self.stair_committed_max_forward_mps = float(
            self.declare_parameter("stair_committed_max_forward_mps", 0.0).value
        )
        self.ramp_max_up_slope_rad = float(
            self.declare_parameter("ramp_max_up_slope_rad", 0.0).value
        )
        self.ramp_max_down_slope_rad = float(
            self.declare_parameter("ramp_max_down_slope_rad", 0.0).value
        )
        self.ramp_max_cross_slope_rad = float(
            self.declare_parameter("ramp_max_cross_slope_rad", 0.0).value
        )
        self.ramp_up_max_x_mps = float(
            self.declare_parameter("ramp_up_max_x_mps", 0.0).value
        )
        self.ramp_down_max_x_mps = float(
            self.declare_parameter("ramp_down_max_x_mps", 0.0).value
        )
        self.ramp_max_yaw_rps = float(
            self.declare_parameter("ramp_max_yaw_rps", 0.0).value
        )
        self.terrain_max_body_roll_rad = float(
            self.declare_parameter("terrain_max_body_roll_rad", 0.0).value
        )
        self.terrain_max_body_pitch_rad = float(
            self.declare_parameter("terrain_max_body_pitch_rad", 0.0).value
        )
        self.require_odom_progress = bool(
            self.declare_parameter("require_odom_progress", False).value
        )
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/dddmr_go2/robot_odom_standard"
        ).get_parameter_value().string_value
        self.odom_timeout_sec = float(
            self.declare_parameter("odom_timeout_sec", 0.30).value
        )
        progress_minimum_command_mps = float(
            self.declare_parameter("progress_minimum_command_mps", 0.10).value
        )
        progress_minimum_measured_mps = float(
            self.declare_parameter("progress_minimum_measured_mps", 0.05).value
        )
        progress_minimum_speed_ratio = float(
            self.declare_parameter("progress_minimum_speed_ratio", 0.50).value
        )
        progress_mismatch_timeout_sec = float(
            self.declare_parameter("progress_mismatch_timeout_sec", 0.50).value
        )

        self.latest_cmd: Optional[Twist] = None
        self.last_cmd_time = None
        self.localization_status: Optional[str] = None
        self.last_localization_status_time = None
        self.terrain_status: Optional[TerrainStatus] = None
        self.last_terrain_status_time = None
        self.odom: Optional[Odometry] = None
        self.last_odom_time = None
        self.progress_monitor = MotionProgressMonitor(
            progress_minimum_command_mps,
            progress_minimum_measured_mps,
            progress_minimum_speed_ratio,
            progress_mismatch_timeout_sec,
        )
        self.terrain_fault_latch = TerrainFaultLatch()
        self.last_output_motion_requested = False
        self.last_log_time = self.get_clock().now()
        self.last_output_key = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.speed_limit_pub = self.create_publisher(
            Float32, "/dddmr_terrain/speed_limit", 10
        )
        self.create_subscription(Twist, self.input_topic, self.cmd_cb, 10)
        self.create_subscription(
            String,
            self.localization_status_topic,
            self.localization_status_cb,
            10,
        )
        self.create_subscription(
            TerrainStatus,
            self.terrain_status_topic,
            self.terrain_status_cb,
            10,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_service(Trigger, "~/reset_terrain_fault", self.reset_fault_cb)
        self.create_timer(self.timer_period_sec(), self.timer_cb)

        self.get_logger().warn(
            "Go2 nav cmd gate: %s -> %s enabled=%s max_x=%.3f max_y=%.3f "
            "max_yaw=%.3f publish_rate_hz=%.3f timeout=%.3f "
            "require_localization_tracking=%s localization_timeout=%.3f"
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
            )
        )
        self.get_logger().warn(
            "terrain_gate required=%s topic=%s timeout=%.3f min_confidence=%.3f "
            "min_support=%.3f stair_heading=%.3f stair_lateral=%.3f "
            "align=(x=%.3f,yaw=%.3f) committed_yaw=%.3f"
            % (
                self.require_terrain_safe,
                self.terrain_status_topic,
                self.terrain_status_timeout_sec,
                self.terrain_min_confidence,
                self.terrain_min_support_ratio,
                self.stair_max_heading_error_rad,
                self.stair_max_lateral_error_m,
                self.stair_align_max_forward_mps,
                self.stair_align_max_yaw_rps,
                self.stair_committed_max_yaw_rps,
            )
        )

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def localization_status_cb(self, msg: String) -> None:
        self.localization_status = msg.data
        self.last_localization_status_time = self.get_clock().now()

    def terrain_status_cb(self, msg: TerrainStatus) -> None:
        self.terrain_status = msg
        self.last_terrain_status_time = self.get_clock().now()

    def odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = self.get_clock().now()

    def timer_cb(self) -> None:
        now = self.get_clock().now()
        reason = "pass"
        localization_reason = self.localization_block_reason(now)
        if not self.enabled:
            output = Twist()
            reason = "disabled"
        elif self.require_terrain_safe and self.terrain_fault_latch.latched:
            output = Twist()
            reason = "terrain_fault_latched:%s" % self.terrain_fault_latch.reason
        elif localization_reason is not None:
            output = Twist()
            reason = localization_reason
            if self.require_terrain_safe:
                motion_requested = self.latest_cmd is not None and (
                    self.twist_requests_motion(self.latest_cmd)
                )
                self.terrain_fault_latch.observe(
                    False, reason, motion_requested
                )
        elif self.latest_cmd is None or self.last_cmd_time is None:
            output = Twist()
            reason = "no_input"
        else:
            age = (now - self.last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout_sec:
                output = Twist()
                reason = "timeout %.3fs" % age
                if self.require_terrain_safe:
                    self.terrain_fault_latch.observe(
                        False,
                        "command_timeout",
                        self.twist_requests_motion(self.latest_cmd),
                    )
            else:
                terrain_result = self.terrain_gate_result(now, self.latest_cmd)
                motion_requested = self.twist_requests_motion(self.latest_cmd)
                progress_reason = None
                if terrain_result.allowed and self.require_odom_progress:
                    progress_reason = self.odom_progress_reason(
                        now, terrain_result.x
                    )
                allowed = terrain_result.allowed and progress_reason is None
                reason = progress_reason or terrain_result.reason
                if self.require_terrain_safe:
                    self.terrain_fault_latch.observe(
                        allowed, reason, motion_requested
                    )
                    if self.terrain_fault_latch.latched:
                        allowed = False
                        reason = "terrain_fault_latched:%s" % (
                            self.terrain_fault_latch.reason
                        )
                if not allowed:
                    output = Twist()
                else:
                    terrain_filtered = Twist()
                    terrain_filtered.linear.x = terrain_result.x
                    terrain_filtered.linear.y = terrain_result.y
                    terrain_filtered.angular.z = terrain_result.yaw
                    if terrain_result.reason in (
                        "stair_approach",
                        "stair_align",
                        "stair_committed",
                    ):
                        # The pure policy already checked these commands against
                        # both stair and output limits. Preserve the exact
                        # collision-scored command instead of reshaping it here.
                        output = terrain_filtered
                    else:
                        output = self.clamp_twist(terrain_filtered)
                    reason = terrain_result.reason

        self.pub.publish(output)
        self.last_output_motion_requested = self.twist_requests_motion(output)
        speed_limit = 0.0
        if reason == "ramp_up":
            speed_limit = self.ramp_up_max_x_mps
        elif reason == "ramp_down":
            speed_limit = self.ramp_down_max_x_mps
        elif reason == "ramp_level":
            speed_limit = min(self.ramp_up_max_x_mps, self.ramp_down_max_x_mps)
        elif reason == "stair_committed":
            speed_limit = self.stair_committed_max_forward_mps
        elif reason == "stair_align":
            speed_limit = self.stair_align_max_forward_mps
        elif reason in (
            "stair_approach",
            "terrain_pass",
            "terrain_optional",
            "pass",
        ):
            speed_limit = self.max_x
        speed_limit_message = Float32()
        speed_limit_message.data = float(max(0.0, speed_limit))
        self.speed_limit_pub.publish(speed_limit_message)
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

    def terrain_gate_result(self, now, msg: Twist):
        status_age_sec = None
        policy_status = None
        if self.terrain_status is not None and self.last_terrain_status_time is not None:
            receive_age = (
                now - self.last_terrain_status_time
            ).nanoseconds / 1e9
            reported_age = float(self.terrain_status.data_age_sec)
            status_age_sec = (
                max(receive_age, reported_age)
                if math.isfinite(reported_age) and reported_age >= 0.0
                else float("inf")
            )
            policy_status = TerrainGateStatus(
                terrain_class=int(self.terrain_status.terrain_class),
                traversal_state=int(self.terrain_status.traversal_state),
                allow_forward=bool(self.terrain_status.allow_forward),
                allow_reverse=bool(self.terrain_status.allow_reverse),
                drop_detected=bool(self.terrain_status.drop_detected),
                dynamic_obstacle=bool(self.terrain_status.dynamic_obstacle),
                gait_unchanged=bool(self.terrain_status.gait_unchanged),
                confidence=float(self.terrain_status.confidence),
                support_ratio=float(self.terrain_status.support_ratio),
                heading_error_rad=float(self.terrain_status.heading_error_rad),
                lateral_error_m=float(self.terrain_status.lateral_error_m),
                longitudinal_slope_rad=float(
                    self.terrain_status.longitudinal_slope_rad
                ),
                cross_slope_rad=float(self.terrain_status.cross_slope_rad),
                body_roll_rad=float(self.terrain_status.body_roll_rad),
                body_pitch_rad=float(self.terrain_status.body_pitch_rad),
            )
        limits = TerrainGateLimits(
            timeout_sec=self.terrain_status_timeout_sec,
            min_confidence=self.terrain_min_confidence,
            min_support_ratio=self.terrain_min_support_ratio,
            max_heading_error_rad=self.stair_max_heading_error_rad,
            max_lateral_error_m=self.stair_max_lateral_error_m,
            align_max_forward_mps=self.stair_align_max_forward_mps,
            align_max_yaw_rps=self.stair_align_max_yaw_rps,
            committed_max_yaw_rps=self.stair_committed_max_yaw_rps,
            zero_epsilon=self.zero_epsilon,
            committed_max_forward_mps=self.stair_committed_max_forward_mps,
            ramp_max_up_slope_rad=self.ramp_max_up_slope_rad,
            ramp_max_down_slope_rad=self.ramp_max_down_slope_rad,
            ramp_max_cross_slope_rad=self.ramp_max_cross_slope_rad,
            ramp_up_max_x_mps=self.ramp_up_max_x_mps,
            ramp_down_max_x_mps=self.ramp_down_max_x_mps,
            ramp_max_yaw_rps=self.ramp_max_yaw_rps,
            max_body_roll_rad=self.terrain_max_body_roll_rad,
            max_body_pitch_rad=self.terrain_max_body_pitch_rad,
            output_max_x_mps=self.max_x,
            output_max_y_mps=self.max_y,
            output_max_yaw_rps=self.max_yaw,
        )
        return terrain_motion_gate(
            self.require_terrain_safe,
            policy_status,
            status_age_sec,
            limits,
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )

    def odom_progress_reason(self, now, commanded_x_mps: float) -> Optional[str]:
        if self.odom is None or self.last_odom_time is None:
            self.progress_monitor.reset()
            return "odom_no_status"
        age = (now - self.last_odom_time).nanoseconds / 1e9
        if (
            not math.isfinite(age)
            or not math.isfinite(self.odom_timeout_sec)
            or self.odom_timeout_sec <= 0.0
            or age < 0.0
            or age > self.odom_timeout_sec
        ):
            self.progress_monitor.reset()
            return "odom_status_stale"
        return self.progress_monitor.update(
            now.nanoseconds / 1e9,
            commanded_x_mps,
            float(self.odom.twist.twist.linear.x),
        )

    def twist_requests_motion(self, msg: Twist) -> bool:
        return any(
            math.isfinite(value) and abs(value) > self.zero_epsilon
            for value in (msg.linear.x, msg.linear.y, msg.angular.z)
        )

    def reset_fault_cb(self, request, response):
        del request
        now = self.get_clock().now()
        stopped = not self.last_output_motion_requested and (
            self.latest_cmd is None or not self.twist_requests_motion(self.latest_cmd)
        )
        healthy = False
        if stopped:
            zero = Twist()
            terrain_result = self.terrain_gate_result(now, zero)
            healthy = terrain_result.allowed and self.localization_block_reason(now) is None
            if healthy and self.require_odom_progress:
                healthy = self.odom_progress_reason(now, 0.0) is None
        response.success = self.terrain_fault_latch.reset(stopped, healthy)
        response.message = (
            "terrain command-gate fault reset; re-arming required"
            if response.success
            else "reset refused: stop command and healthy fresh inputs are required"
        )
        if response.success:
            zero = Twist()
            self.pub.publish(zero)
            self.last_output_motion_requested = False
            speed_limit = Float32()
            speed_limit.data = 0.0
            self.speed_limit_pub.publish(speed_limit)
        return response

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
