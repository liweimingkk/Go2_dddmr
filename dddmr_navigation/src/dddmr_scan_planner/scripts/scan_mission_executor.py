#!/usr/bin/env python3
"""Fail-closed sequential mission executor for the Go2 XT16 SCAN chain."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from scan_mission_io import (
    ArrivalWindow,
    MissionValidationError,
    Waypoint,
    load_initial_pose,
    load_mission,
    normalize_angle,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


class ScanMissionExecutor(Node):
    TERMINAL_STATES = {"COMPLETE", "FAILED"}
    PREARM_STATES = {
        "WAITING_PLANNER",
        "WAITING_INPUTS",
        "STABILIZING",
        "READY",
    }

    def __init__(self) -> None:
        super().__init__("scan_mission_executor")
        mission_file = self.declare_parameter("mission_file", "").value
        allowed_root = self.declare_parameter(
            "allowed_root", "/root/dddmr_bags"
        ).value
        if not isinstance(mission_file, str) or not mission_file:
            raise MissionValidationError("mission_file is required")
        self.mission = load_mission(mission_file, allowed_root=allowed_root)
        self.initial_pose = load_initial_pose(self.mission.initial_pose_path)
        self.goal_topic = self.declare_parameter(
            "goal_topic", "/goal_pose_3d"
        ).value
        if (
            not isinstance(self.goal_topic, str)
            or not self.goal_topic.startswith("/")
            or any(character.isspace() for character in self.goal_topic)
        ):
            raise MissionValidationError(
                "goal_topic must be a non-empty absolute ROS topic"
            )

        self.position_tolerance = self._positive_parameter(
            "position_tolerance", 0.15, maximum=1.0
        )
        self.height_tolerance = self._positive_parameter(
            "height_tolerance", 0.20, maximum=1.0
        )
        self.yaw_tolerance = self._positive_parameter(
            "yaw_tolerance", 0.15, maximum=math.pi
        )
        self.arrival_stable_sec = self._positive_parameter(
            "arrival_stable_sec", 0.50, maximum=10.0
        )
        self.planning_timeout_sec = self._positive_parameter(
            "planning_timeout_sec", 10.0, maximum=60.0
        )
        self.waypoint_timeout_sec = self._positive_parameter(
            "waypoint_timeout_sec", 120.0, maximum=3600.0
        )
        self.initial_pose_timeout_sec = self._positive_parameter(
            "initial_pose_timeout_sec", 60.0, maximum=300.0
        )
        self.initial_pose_map_settle_sec = self._positive_parameter(
            "initial_pose_map_settle_sec", 1.0, maximum=30.0
        )
        self.initial_pose_retry_sec = self._positive_parameter(
            "initial_pose_retry_sec", 5.0, maximum=60.0
        )
        self.initial_pose_xy_tolerance = self._positive_parameter(
            "initial_pose_xy_tolerance", 0.75, maximum=5.0
        )
        self.initial_pose_z_tolerance = self._positive_parameter(
            "initial_pose_z_tolerance", 0.35, maximum=2.0
        )
        self.initial_pose_yaw_tolerance = self._positive_parameter(
            "initial_pose_yaw_tolerance", 0.50, maximum=math.pi
        )
        self.planner_ready_timeout_sec = self._positive_parameter(
            "planner_ready_timeout_sec", 60.0, maximum=300.0
        )
        self.prearm_stable_sec = self._positive_parameter(
            "prearm_stable_sec", 1.0, maximum=10.0
        )
        self.input_timeout_sec = self._positive_parameter(
            "input_timeout_sec", 0.75, maximum=5.0
        )
        self.guard_failure_grace_sec = self._positive_parameter(
            "guard_failure_grace_sec", 0.50, maximum=5.0
        )
        self.zero_epsilon = self._positive_parameter(
            "zero_epsilon", 0.001, maximum=0.05
        )
        self.body_height = self._positive_parameter(
            "body_height", 0.32, maximum=1.0
        )
        self.planner_ready_topic = self.declare_parameter(
            "planner_ready_topic", "/weighted_ground"
        ).value
        if (
            not isinstance(self.planner_ready_topic, str)
            or not self.planner_ready_topic.startswith("/")
            or any(
                character.isspace()
                for character in self.planner_ready_topic
            )
        ):
            raise MissionValidationError(
                "planner_ready_topic must be a non-empty absolute ROS topic"
            )
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)

        self.state = "LOADING_INITIAL_POSE"
        self.failure_reason = ""
        self.enabled = False
        self.current_index = -1
        self.body_pose: Optional[Odometry] = None
        self.mcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.raw_command: Optional[Twist] = None
        self.localization_status: Optional[str] = None
        self.localization_health: Optional[str] = None
        self.route_ready = False
        self.guard_status: Optional[str] = None
        self.planner_ready = False
        self.map_ground_ready = False
        self.map_ground_ready_at = 0.0
        self.initial_pose_sent = False
        self.initial_pose_sent_at = 0.0
        self.initial_pose_attempts = 0
        self.initial_pose_confirmed = False
        self.saw_post_seed_nontracking = False
        self.initial_deadline = time.monotonic() + self.initial_pose_timeout_sec
        self.planner_deadline = (
            time.monotonic() + self.planner_ready_timeout_sec
        )
        self.prearm_ready_since: Optional[float] = None
        self.goal_sent_at = 0.0
        self.goal_route_ready_at = 0.0
        self.route_not_ready_after_goal = False
        self.guard_seen_ok = False
        self.waypoint_deadline = 0.0
        self.dwell_deadline = 0.0
        self.guard_failure_since: Optional[float] = None
        self.receipt = {}
        self.arrival_window = ArrivalWindow(self.arrival_stable_sec)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, "/scan_multi_point/state", transient_qos
        )
        self.index_pub = self.create_publisher(
            Int32, "/scan_multi_point/current_waypoint", transient_qos
        )
        self.enabled_pub = self.create_publisher(
            Bool, "/scan_multi_point/enabled", transient_qos
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initial_3d_pose", 10
        )

        self.create_subscription(
            Odometry,
            "/scan_planner/body_pose",
            self.body_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/mcl_pose",
            self.mcl_pose_callback,
            10,
        )
        self.create_subscription(
            Twist, "/scan_planner/raw_cmd_vel", self.raw_command_callback, 10
        )
        self.create_subscription(
            Bool, "/scan_planner/route_ready", self.route_ready_callback, transient_qos
        )
        self.create_subscription(
            String, "/localization_status", self.localization_status_callback,
            transient_qos,
        )
        self.create_subscription(
            String, "/localization_health", self.localization_health_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/scan_planner/command_guard_status",
            self.guard_status_callback,
            10,
        )
        self.map_ground_subscription = self.create_subscription(
            PointCloud2, "/map1/mapground", self.map_ground_callback, transient_qos
        )
        self.create_subscription(
            PointCloud2,
            self.planner_ready_topic,
            self.planner_ready_callback,
            transient_qos,
        )
        self.create_service(Trigger, "/scan_multi_point/arm", self.arm_callback)
        self.create_service(Trigger, "/scan_multi_point/cancel", self.cancel_callback)
        self.create_timer(0.05, self.timer_callback)
        self.publish_state()
        self.get_logger().info(
            "Loaded mission %s with %d waypoint(s); fixed initial pose %s"
            % (
                self.mission.mission_id,
                len(self.mission.waypoints),
                self.mission.initial_pose_path,
            )
        )

    def _positive_parameter(
        self, name: str, default: float, *, maximum: float
    ) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0 or value > maximum:
            raise MissionValidationError(
                f"{name} must be finite and within (0, {maximum}]"
            )
        return value

    def body_callback(self, message: Odometry) -> None:
        if message.header.frame_id == "map":
            self.body_pose = message
            self.receipt["body"] = time.monotonic()

    def mcl_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id == self.initial_pose.frame_id:
            self.mcl_pose = message
            self.receipt["mcl_pose"] = time.monotonic()

    def raw_command_callback(self, message: Twist) -> None:
        self.raw_command = message
        self.receipt["raw_command"] = time.monotonic()

    def route_ready_callback(self, message: Bool) -> None:
        self.route_ready = bool(message.data)
        self.receipt["route_ready"] = time.monotonic()
        if not self.route_ready and self.goal_sent_at > 0.0:
            self.route_not_ready_after_goal = True
        if (
            self.route_ready
            and self.route_not_ready_after_goal
            and self.goal_sent_at > 0.0
        ):
            self.goal_route_ready_at = time.monotonic()

    def localization_status_callback(self, message: String) -> None:
        self.localization_status = message.data
        self.receipt["localization_status"] = time.monotonic()
        if self.initial_pose_sent and message.data != "TRACKING":
            self.saw_post_seed_nontracking = True

    def localization_health_callback(self, message: String) -> None:
        self.localization_health = message.data
        self.receipt["localization_health"] = time.monotonic()

    def guard_status_callback(self, message: String) -> None:
        self.guard_status = message.data
        self.receipt["guard_status"] = time.monotonic()

    def map_ground_callback(self, message: PointCloud2) -> None:
        if message.width * message.height > 0:
            self.map_ground_ready = True
            self.map_ground_ready_at = time.monotonic()
            self.receipt["map_ground"] = self.map_ground_ready_at
            self.get_logger().info(
                "Map ground received; waiting %.2fs for MCL ground-tree setup"
                % self.initial_pose_map_settle_sec
            )
            if self.map_ground_subscription is not None:
                self.destroy_subscription(self.map_ground_subscription)
                self.map_ground_subscription = None

    def planner_ready_callback(self, message: PointCloud2) -> None:
        if (
            message.header.frame_id == self.initial_pose.frame_id
            and message.width * message.height > 0
        ):
            first_ready = not self.planner_ready
            self.planner_ready = True
            self.receipt["planner_ready"] = time.monotonic()
            if first_ready:
                self.get_logger().info(
                    "Global planner weighted ground is ready"
                )

    def arm_callback(self, _request, response):
        now = time.monotonic()
        if (
            self.state != "READY"
            or not self.planner_ready
            or not self.localization_is_fresh_and_healthy(now)
        ):
            if self.state == "READY":
                self.prearm_ready_since = None
                self.update_prearm_readiness(now)
            response.success = False
            response.message = (
                f"mission is not READY (state={self.state}; "
                f"{self.prearm_wait_reason(now)})"
            )
            return response
        self.start_mission()
        response.success = True
        response.message = f"armed mission {self.mission.mission_id}"
        return response

    def cancel_callback(self, _request, response):
        if self.state in self.TERMINAL_STATES:
            response.success = False
            response.message = f"mission already terminal (state={self.state})"
            return response
        self.fail("cancelled by operator")
        response.success = True
        response.message = "mission cancelled and command output disabled"
        return response

    def publish_state(self) -> None:
        state_message = String()
        state_message.data = self.state
        self.state_pub.publish(state_message)
        index_message = Int32()
        index_message.data = self.current_index
        self.index_pub.publish(index_message)
        self.publish_enabled()

    def publish_enabled(self) -> None:
        enabled_message = Bool()
        enabled_message.data = self.enabled
        self.enabled_pub.publish(enabled_message)

    def transition(self, state: str) -> None:
        if state != self.state:
            self.get_logger().info(f"Mission state {self.state} -> {state}")
            self.state = state
        self.publish_state()

    def fail(self, reason: str) -> None:
        if self.state in self.TERMINAL_STATES:
            return
        self.failure_reason = reason
        self.enabled = False
        self.arrival_window.reset()
        self.get_logger().error(
            f"Mission {self.mission.mission_id} failed: {reason}"
        )
        self.transition("FAILED")

    def mcl_is_fresh_and_healthy(self, now: float) -> bool:
        return (
            self.localization_status == "TRACKING"
            and self.localization_health == "HEALTHY"
            and self.is_fresh("localization_status", now)
            and self.is_fresh("localization_health", now)
        )

    def localization_is_fresh_and_healthy(self, now: float) -> bool:
        return self.mcl_is_fresh_and_healthy(now) and self.is_fresh(
            "body", now
        )

    def is_fresh(self, name: str, now: float) -> bool:
        timestamp = self.receipt.get(name)
        return timestamp is not None and 0.0 <= now - timestamp <= self.input_timeout_sec

    def input_age(self, name: str, now: float) -> float:
        timestamp = self.receipt.get(name)
        if timestamp is None:
            return math.inf
        return now - timestamp

    def prearm_wait_reason(self, now: float) -> str:
        if not self.planner_ready:
            return "global planner weighted ground is not ready"
        if (
            self.localization_status != "TRACKING"
            or not self.is_fresh("localization_status", now)
        ):
            return (
                "localization status is not fresh TRACKING "
                f"(value={self.localization_status or 'missing'}, "
                f"age={self.input_age('localization_status', now):.3f}s)"
            )
        if (
            self.localization_health != "HEALTHY"
            or not self.is_fresh("localization_health", now)
        ):
            return (
                "localization health is not fresh HEALTHY "
                f"(value={self.localization_health or 'missing'}, "
                f"age={self.input_age('localization_health', now):.3f}s)"
            )
        if not self.is_fresh("body", now):
            return (
                "SCAN body pose is stale "
                f"(age={self.input_age('body', now):.3f}s)"
            )
        if self.prearm_ready_since is None:
            return "pre-arm inputs have not started their stable window"
        stable_for = now - self.prearm_ready_since
        if stable_for < self.prearm_stable_sec:
            return (
                "pre-arm inputs are stabilizing "
                f"({stable_for:.3f}/{self.prearm_stable_sec:.3f}s)"
            )
        return "all pre-arm inputs are ready"

    def update_prearm_readiness(self, now: float) -> None:
        if not self.planner_ready:
            self.prearm_ready_since = None
            if now > self.planner_deadline:
                self.fail(
                    "timed out waiting for global planner weighted ground"
                )
            else:
                self.transition("WAITING_PLANNER")
            return
        if not self.localization_is_fresh_and_healthy(now):
            self.prearm_ready_since = None
            self.transition("WAITING_INPUTS")
            return
        if self.prearm_ready_since is None:
            self.prearm_ready_since = now
        if now - self.prearm_ready_since < self.prearm_stable_sec:
            self.transition("STABILIZING")
            return
        was_ready = self.state == "READY"
        self.enabled = False
        self.transition("READY")
        if self.auto_arm and not was_ready:
            self.start_mission()

    def initial_pose_errors(self) -> tuple[float, float, float]:
        if self.mcl_pose is None:
            return (math.inf, math.inf, math.inf)
        pose = self.mcl_pose.pose.pose
        horizontal = math.hypot(
            float(pose.position.x) - self.initial_pose.position[0],
            float(pose.position.y) - self.initial_pose.position[1],
        )
        height = abs(float(pose.position.z) - self.initial_pose.position[2])
        pose_yaw = yaw_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        initial_yaw = yaw_from_quaternion(self.initial_pose.orientation)
        yaw_error = abs(normalize_angle(initial_yaw - pose_yaw))
        return (horizontal, height, yaw_error)

    def confirm_initial_pose(self, now: float) -> None:
        if self.initial_pose_confirmed or not self.initial_pose_sent:
            return
        receipt = self.receipt.get("mcl_pose", 0.0)
        if (
            receipt <= self.initial_pose_sent_at
            or not self.is_fresh("mcl_pose", now)
        ):
            return
        horizontal, height, yaw_error = self.initial_pose_errors()
        if (
            horizontal <= self.initial_pose_xy_tolerance
            and height <= self.initial_pose_z_tolerance
            and yaw_error <= self.initial_pose_yaw_tolerance
        ):
            self.initial_pose_confirmed = True
            self.get_logger().info(
                "MCL accepted fixed initial pose "
                "(xy_error=%.3f z_error=%.3f yaw_error=%.3f)"
                % (horizontal, height, yaw_error)
            )

    def publish_initial_pose(self) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.initial_pose.frame_id
        message.pose.pose.position.x = self.initial_pose.position[0]
        message.pose.pose.position.y = self.initial_pose.position[1]
        message.pose.pose.position.z = self.initial_pose.position[2]
        message.pose.pose.orientation.x = self.initial_pose.orientation[0]
        message.pose.pose.orientation.y = self.initial_pose.orientation[1]
        message.pose.pose.orientation.z = self.initial_pose.orientation[2]
        message.pose.pose.orientation.w = self.initial_pose.orientation[3]
        message.pose.covariance = list(self.initial_pose.covariance)
        self.initial_pose_attempts += 1
        self.initial_pose_sent = True
        self.initial_pose_sent_at = time.monotonic()
        self.initial_pose_confirmed = False
        self.saw_post_seed_nontracking = False
        self.initial_pose_pub.publish(message)
        self.get_logger().info(
            "Published fixed initial pose attempt %d at [%.3f, %.3f, %.3f]"
            % (self.initial_pose_attempts, *self.initial_pose.position)
        )
        self.transition("LOCALIZING")

    def start_mission(self) -> None:
        self.enabled = True
        self.current_index = 0
        self.publish_goal()

    def current_waypoint(self) -> Waypoint:
        return self.mission.waypoints[self.current_index]

    def publish_goal(self) -> None:
        waypoint = self.current_waypoint()
        qx, qy, qz, qw = quaternion_from_yaw(waypoint.yaw)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = waypoint.x
        message.pose.position.y = waypoint.y
        message.pose.position.z = waypoint.z
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        now = time.monotonic()
        self.route_ready = False
        self.route_not_ready_after_goal = False
        self.goal_route_ready_at = 0.0
        self.guard_seen_ok = False
        self.goal_sent_at = now
        self.waypoint_deadline = now + self.waypoint_timeout_sec
        self.guard_failure_since = None
        self.arrival_window.reset()
        self.goal_pub.publish(message)
        self.get_logger().info(
            "Submitted waypoint %d/%d %s: [%.3f, %.3f, %.3f] yaw=%.3f dwell=%.1fs"
            % (
                self.current_index + 1,
                len(self.mission.waypoints),
                waypoint.waypoint_id,
                waypoint.x,
                waypoint.y,
                waypoint.z,
                waypoint.yaw,
                waypoint.dwell_sec,
            )
        )
        self.transition("NAVIGATING")

    def command_is_stopped(self, now: float) -> bool:
        if self.raw_command is None or not self.is_fresh("raw_command", now):
            return False
        values = (
            self.raw_command.linear.x,
            self.raw_command.linear.y,
            self.raw_command.angular.z,
        )
        return all(
            math.isfinite(value) and abs(value) <= self.zero_epsilon
            for value in values
        )

    def pose_errors(self) -> tuple[float, float, float]:
        if self.body_pose is None:
            return (math.inf, math.inf, math.inf)
        waypoint = self.current_waypoint()
        pose = self.body_pose.pose.pose
        horizontal = math.hypot(
            float(pose.position.x) - waypoint.x,
            float(pose.position.y) - waypoint.y,
        )
        height = abs(float(pose.position.z) - (waypoint.z + self.body_height))
        yaw = yaw_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        yaw_error = abs(normalize_angle(waypoint.yaw - yaw))
        return (horizontal, height, yaw_error)

    def check_guard(self, now: float) -> bool:
        guard_is_ok = (
            self.guard_status == "ok"
            and self.is_fresh("guard_status", now)
            and self.receipt.get("guard_status", 0.0) > self.goal_sent_at
        )
        if guard_is_ok:
            self.guard_seen_ok = True
            self.guard_failure_since = None
            return True
        if not self.guard_seen_ok:
            if (
                self.goal_route_ready_at > self.goal_sent_at
                and now - self.goal_route_ready_at > self.planning_timeout_sec
            ):
                self.fail(
                    "SCAN command guard never enabled for the current trajectory: "
                    f"{self.guard_status or 'missing status'}"
                )
            return False
        if self.guard_failure_since is None:
            self.guard_failure_since = now
            return False
        if now - self.guard_failure_since > self.guard_failure_grace_sec:
            self.fail(
                "SCAN command guard is not healthy: "
                f"{self.guard_status or 'missing status'}"
            )
        return False

    def timer_callback(self) -> None:
        now = time.monotonic()
        # This is intentionally a heartbeat, not only a latched arm bit. The
        # command guard closes if the executor stops refreshing it.
        self.publish_enabled()
        if self.state in self.TERMINAL_STATES:
            self.publish_state()
            return

        if self.state == "LOADING_INITIAL_POSE":
            if now > self.initial_deadline:
                self.fail("timed out waiting for map ground and initial-pose subscriber")
                return
            if (
                self.map_ground_ready
                and now - self.map_ground_ready_at
                >= self.initial_pose_map_settle_sec
                and self.initial_pose_pub.get_subscription_count() > 0
            ):
                self.publish_initial_pose()
            return

        if self.state == "LOCALIZING":
            if now > self.initial_deadline:
                if not self.initial_pose_confirmed:
                    self.fail(
                        "MCL never confirmed the saved fixed initial pose"
                    )
                else:
                    self.fail(
                        "fixed initial pose did not produce healthy TRACKING"
                    )
                return
            self.confirm_initial_pose(now)
            if (
                self.saw_post_seed_nontracking
                and self.initial_pose_confirmed
                and self.mcl_is_fresh_and_healthy(now)
            ):
                self.enabled = False
                self.update_prearm_readiness(now)
                return
            if (
                not self.initial_pose_confirmed
                and now - self.initial_pose_sent_at
                >= self.initial_pose_retry_sec
            ):
                horizontal, height, yaw_error = self.initial_pose_errors()
                self.get_logger().warning(
                    "Saved initial pose is not confirmed "
                    "(xy_error=%.3f z_error=%.3f yaw_error=%.3f); retrying"
                    % (horizontal, height, yaw_error)
                )
                self.publish_initial_pose()
            return

        if self.state in self.PREARM_STATES:
            self.update_prearm_readiness(now)
            return

        if not self.localization_is_fresh_and_healthy(now):
            self.fail("localization is not fresh TRACKING/HEALTHY")
            return

        if self.state in ("NAVIGATING", "ALIGNING"):
            if now > self.waypoint_deadline:
                self.fail(
                    f"waypoint {self.current_waypoint().waypoint_id} timed out"
                )
                return
            if self.goal_route_ready_at <= self.goal_sent_at:
                if now - self.goal_sent_at > self.planning_timeout_sec:
                    self.fail(
                        f"planning timed out for {self.current_waypoint().waypoint_id}"
                    )
                return
            if not self.check_guard(now):
                self.arrival_window.reset()
                return
            horizontal, height, yaw_error = self.pose_errors()
            position_complete = (
                horizontal <= self.position_tolerance
                and height <= self.height_tolerance
            )
            pose_complete = position_complete and yaw_error <= self.yaw_tolerance
            if position_complete and not pose_complete:
                self.transition("ALIGNING")
            elif not position_complete and self.state == "ALIGNING":
                self.transition("NAVIGATING")
            if self.arrival_window.update(
                now, pose_complete, self.command_is_stopped(now)
            ):
                waypoint = self.current_waypoint()
                self.dwell_deadline = now + waypoint.dwell_sec
                self.enabled = True
                self.transition("DWELLING")
            return

        if self.state == "DWELLING":
            if not self.check_guard(now):
                return
            if not self.command_is_stopped(now):
                self.fail("controller produced motion during waypoint dwell")
                return
            horizontal, height, yaw_error = self.pose_errors()
            if (
                horizontal > self.position_tolerance
                or height > self.height_tolerance
                or yaw_error > self.yaw_tolerance
            ):
                self.fail("waypoint pose left tolerance during dwell")
                return
            if now < self.dwell_deadline:
                return
            if self.current_index + 1 >= len(self.mission.waypoints):
                self.enabled = False
                self.transition("COMPLETE")
                self.get_logger().info(
                    f"Mission {self.mission.mission_id} completed"
                )
                return
            self.current_index += 1
            self.publish_goal()


def main() -> None:
    rclpy.init()
    try:
        node = ScanMissionExecutor()
    except (MissionValidationError, OSError, ValueError) as error:
        get_logger("scan_mission_executor").fatal(str(error))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok(context=node.context):
            node.fail("executor interrupted")
    finally:
        if rclpy.ok(context=node.context):
            node.enabled = False
            node.publish_state()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
