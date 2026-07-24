#!/usr/bin/env python3
"""Fail-closed sequential mission executor for the DDDMR P2P action."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.action import ActionClient
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger

from waypoint_mission_io import (
    ArrivalWindow,
    MissionValidationError,
    UnhealthyGraceWindow,
    Waypoint,
    load_initial_pose,
    load_mission,
    normalize_angle,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


class P2PMissionExecutor(Node):
    TERMINAL_STATES = {"COMPLETE", "FAILED"}
    PREARM_STATES = {
        "WAITING_PLANNER",
        "WAITING_INPUTS",
        "WAITING_P2P",
        "STABILIZING",
        "READY",
    }

    def __init__(self) -> None:
        super().__init__("p2p_mission_executor")
        mission_file = self.declare_parameter("mission_file", "").value
        allowed_root = self.declare_parameter(
            "allowed_root", "/root/dddmr_bags"
        ).value
        if not isinstance(mission_file, str) or not mission_file:
            raise MissionValidationError("mission_file is required")
        self.mission = load_mission(mission_file, allowed_root=allowed_root)
        self.initial_pose = load_initial_pose(self.mission.initial_pose_path)

        self.position_tolerance = self._positive_parameter(
            "position_tolerance", 0.30, maximum=1.0
        )
        self.yaw_tolerance = self._positive_parameter(
            "yaw_tolerance", 0.15, maximum=math.pi
        )
        self.arrival_stable_sec = self._positive_parameter(
            "arrival_stable_sec", 0.50, maximum=10.0
        )
        self.dwell_exit_position_tolerance = self._positive_parameter(
            "dwell_exit_position_tolerance", 0.40, maximum=2.0
        )
        self.dwell_exit_yaw_tolerance = self._positive_parameter(
            "dwell_exit_yaw_tolerance", 0.25, maximum=math.pi
        )
        self.dwell_pose_grace_sec = self._positive_parameter(
            "dwell_pose_grace_sec", 1.0, maximum=30.0
        )
        if self.dwell_exit_position_tolerance < self.position_tolerance:
            raise MissionValidationError(
                "dwell_exit_position_tolerance must be greater than or equal "
                "to position_tolerance"
            )
        if self.dwell_exit_yaw_tolerance < self.yaw_tolerance:
            raise MissionValidationError(
                "dwell_exit_yaw_tolerance must be greater than or equal to "
                "yaw_tolerance"
            )
        self.waypoint_timeout_sec = self._positive_parameter(
            "waypoint_timeout_sec", 180.0, maximum=3600.0
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
            "planner_ready_timeout_sec", 90.0, maximum=300.0
        )
        self.prearm_stable_sec = self._positive_parameter(
            "prearm_stable_sec", 1.0, maximum=10.0
        )
        self.input_timeout_sec = self._positive_parameter(
            "input_timeout_sec", 0.75, maximum=5.0
        )
        self.localization_health_grace_sec = self._positive_parameter(
            "localization_health_grace_sec", 1.50, maximum=30.0
        )
        self.zero_epsilon = self._positive_parameter(
            "zero_epsilon", 0.001, maximum=0.05
        )
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        self.p2p_enable_service = self.declare_parameter(
            "p2p_enable_service", "/p2p_move_base/set_enabled"
        ).value
        if (
            not isinstance(self.p2p_enable_service, str)
            or not self.p2p_enable_service.startswith("/")
            or any(character.isspace() for character in self.p2p_enable_service)
        ):
            raise MissionValidationError(
                "p2p_enable_service must be a non-empty absolute ROS service"
            )

        self.state = "LOADING_INITIAL_POSE"
        self.failure_reason = ""
        self.current_index = -1
        self.mcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.safe_command: Optional[Twist] = None
        self.localization_status: Optional[str] = None
        self.localization_health: Optional[str] = None
        self.map_ground_ready = False
        self.map_ground_ready_at = 0.0
        self.planner_ready = False
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
        self.waypoint_deadline = 0.0
        self.dwell_deadline = 0.0
        self.receipt = {}
        self.arrival_window = ArrivalWindow(self.arrival_stable_sec)
        self.localization_health_grace = UnhealthyGraceWindow(
            self.localization_health_grace_sec
        )
        self.dwell_pose_grace = UnhealthyGraceWindow(
            self.dwell_pose_grace_sec
        )
        self.goal_handle = None
        self.goal_serial = 0

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, "/p2p_multi_point/state", transient_qos
        )
        self.index_pub = self.create_publisher(
            Int32, "/p2p_multi_point/current_waypoint", transient_qos
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initial_3d_pose", 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/mcl_pose", self.mcl_pose_callback, 10
        )
        self.create_subscription(
            Twist,
            "/dddmr_go2/safe_cmd_vel",
            self.safe_command_callback,
            10,
        )
        self.create_subscription(
            String,
            "/localization_status",
            self.localization_status_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/localization_health",
            self.localization_health_callback,
            transient_qos,
        )
        self.map_ground_subscription = self.create_subscription(
            PointCloud2,
            "/map1/mapground",
            self.map_ground_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            "/weighted_ground",
            self.planner_ready_callback,
            transient_qos,
        )
        self.action_client = ActionClient(self, PToPMoveBase, "/p2p_move_base")
        self.enable_client = self.create_client(
            SetBool, self.p2p_enable_service
        )
        self.create_service(Trigger, "/p2p_multi_point/arm", self.arm_callback)
        self.create_service(
            Trigger, "/p2p_multi_point/cancel", self.cancel_callback
        )
        self.create_timer(0.05, self.timer_callback)
        self.publish_state()
        self.get_logger().info(
            "Loaded P2P mission %s with %d waypoint(s); fixed initial pose %s"
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

    def mcl_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id == self.initial_pose.frame_id:
            self.mcl_pose = message
            self.receipt["mcl_pose"] = time.monotonic()

    def safe_command_callback(self, message: Twist) -> None:
        self.safe_command = message
        self.receipt["safe_command"] = time.monotonic()

    def localization_status_callback(self, message: String) -> None:
        self.localization_status = message.data.strip().upper()
        self.receipt["localization_status"] = time.monotonic()
        if self.initial_pose_sent and self.localization_status != "TRACKING":
            self.saw_post_seed_nontracking = True

    def localization_health_callback(self, message: String) -> None:
        self.localization_health = message.data.strip().upper()
        self.receipt["localization_health"] = time.monotonic()

    def map_ground_callback(self, message: PointCloud2) -> None:
        if message.width * message.height <= 0:
            return
        self.map_ground_ready = True
        self.map_ground_ready_at = time.monotonic()
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
            if not self.planner_ready:
                self.get_logger().info("Global planner weighted ground is ready")
            self.planner_ready = True

    def arm_callback(self, _request, response):
        now = time.monotonic()
        if self.state != "READY" or not self.prearm_ready(now):
            if self.state == "READY":
                self.prearm_ready_since = None
                self.update_prearm_readiness(now)
            response.success = False
            response.message = (
                f"mission is not READY (state={self.state}; "
                f"{self.prearm_wait_reason(now)})"
            )
            return response
        self.current_index = 0
        self.transition("ARMING")
        self.request_p2p_enabled(True, self.p2p_enabled_callback)
        response.success = True
        response.message = f"arming mission {self.mission.mission_id}"
        return response

    def cancel_callback(self, _request, response):
        if self.state in self.TERMINAL_STATES:
            response.success = False
            response.message = f"mission already terminal (state={self.state})"
            return response
        self.fail("cancelled by operator")
        response.success = True
        response.message = "mission cancelled and P2P goals disabled"
        return response

    def p2p_enabled_callback(self, future) -> None:
        if self.state != "ARMING":
            return
        try:
            result = future.result()
        except Exception as error:  # rclpy future transports service errors.
            self.fail(f"could not enable P2P goals: {error}")
            return
        if result is None or not result.success:
            message = "no service response" if result is None else result.message
            self.fail(f"P2P goal enable was rejected: {message}")
            return
        self.send_current_goal()

    def request_p2p_enabled(self, enabled: bool, callback=None) -> None:
        if not self.enable_client.service_is_ready():
            if enabled:
                self.fail("P2P goal-enable service is unavailable")
            return
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_client.call_async(request)
        if callback is not None:
            future.add_done_callback(callback)

    def publish_state(self) -> None:
        state_message = String()
        state_message.data = self.state
        self.state_pub.publish(state_message)
        index_message = Int32()
        index_message.data = self.current_index
        self.index_pub.publish(index_message)

    def transition(self, state: str) -> None:
        if state != self.state:
            self.get_logger().info(f"P2P mission state {self.state} -> {state}")
            self.state = state
        self.publish_state()

    def stop_active_goal(self) -> None:
        self.goal_serial += 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as error:  # best-effort cleanup after a failure.
                self.get_logger().warning(f"P2P cancel request failed: {error}")
        self.goal_handle = None
        self.request_p2p_enabled(False)

    def fail(self, reason: str) -> None:
        if self.state in self.TERMINAL_STATES:
            return
        self.failure_reason = reason
        self.arrival_window.reset()
        self.localization_health_grace.reset()
        self.dwell_pose_grace.reset()
        self.get_logger().error(
            f"P2P mission {self.mission.mission_id} failed: {reason}"
        )
        self.transition("FAILED")
        self.stop_active_goal()

    def is_fresh(self, name: str, now: float) -> bool:
        timestamp = self.receipt.get(name)
        return (
            timestamp is not None
            and 0.0 <= now - timestamp <= self.input_timeout_sec
        )

    def input_age(self, name: str, now: float) -> float:
        timestamp = self.receipt.get(name)
        return math.inf if timestamp is None else now - timestamp

    def localization_is_fresh_and_healthy(self, now: float) -> bool:
        return (
            self.localization_tracking_inputs_are_fresh(now)
            and self.localization_health == "HEALTHY"
        )

    def localization_tracking_inputs_are_fresh(self, now: float) -> bool:
        return (
            self.localization_status == "TRACKING"
            and self.is_fresh("localization_status", now)
            and self.is_fresh("localization_health", now)
            and self.is_fresh("mcl_pose", now)
        )

    def localization_snapshot(self, now: float) -> str:
        return (
            f"status={self.localization_status or 'missing'} "
            f"status_age={self.input_age('localization_status', now):.3f}s "
            f"health={self.localization_health or 'missing'} "
            f"health_age={self.input_age('localization_health', now):.3f}s "
            f"mcl_pose_age={self.input_age('mcl_pose', now):.3f}s"
        )

    def extend_active_deadline(self, held_sec: float) -> None:
        if held_sec <= 0.0:
            return
        if self.state in {"NAVIGATING", "ALIGNING", "SETTLING"}:
            self.waypoint_deadline += held_sec
        elif self.state == "DWELLING":
            self.dwell_deadline += held_sec

    def active_localization_allows_progress(self, now: float) -> bool:
        if not self.localization_tracking_inputs_are_fresh(now):
            self.localization_health_grace.reset()
            self.fail(
                "localization tracking input failed: "
                + self.localization_snapshot(now)
            )
            return False

        if self.localization_health == "HEALTHY":
            if self.localization_health_grace.active:
                held_sec = self.localization_health_grace.mark_healthy(now)
                self.extend_active_deadline(held_sec)
                self.get_logger().info(
                    "Localization health recovered after %.3fs; "
                    "resuming P2P mission in state %s"
                    % (held_sec, self.state)
                )
            return True

        first_unhealthy_sample = not self.localization_health_grace.active
        expired = self.localization_health_grace.mark_unhealthy(now)
        held_sec = self.localization_health_grace.elapsed(now)
        if first_unhealthy_sample:
            self.arrival_window.reset()
            self.dwell_pose_grace.reset()
            self.get_logger().warning(
                "Holding P2P mission for transient localization health %s "
                "(grace=%.3fs; %s)"
                % (
                    self.localization_health or "missing",
                    self.localization_health_grace_sec,
                    self.localization_snapshot(now),
                )
            )
        if expired:
            self.fail(
                "localization health remained unhealthy for %.3fs "
                "(grace=%.3fs; %s)"
                % (
                    held_sec,
                    self.localization_health_grace_sec,
                    self.localization_snapshot(now),
                )
            )
        return False

    def prearm_ready(self, now: float) -> bool:
        return (
            self.planner_ready
            and self.action_client.server_is_ready()
            and self.enable_client.service_is_ready()
            and self.localization_is_fresh_and_healthy(now)
        )

    def prearm_wait_reason(self, now: float) -> str:
        if not self.planner_ready:
            return "global planner weighted ground is not ready"
        if not self.action_client.server_is_ready():
            return "P2P action server is not ready"
        if not self.enable_client.service_is_ready():
            return "P2P goal-enable service is not ready"
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
        if not self.is_fresh("mcl_pose", now):
            return (
                "mcl_pose is stale "
                f"(age={self.input_age('mcl_pose', now):.3f}s)"
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
                self.fail("timed out waiting for global planner weighted ground")
            else:
                self.transition("WAITING_PLANNER")
            return
        if (
            not self.action_client.server_is_ready()
            or not self.enable_client.service_is_ready()
        ):
            self.prearm_ready_since = None
            self.transition("WAITING_P2P")
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
        self.transition("READY")
        if self.auto_arm and not was_ready:
            self.current_index = 0
            self.transition("ARMING")
            self.request_p2p_enabled(True, self.p2p_enabled_callback)

    def initial_pose_errors(self) -> tuple[float, float, float]:
        if self.mcl_pose is None:
            return (math.inf, math.inf, math.inf)
        pose = self.mcl_pose.pose.pose
        horizontal = math.hypot(
            float(pose.position.x) - self.initial_pose.position[0],
            float(pose.position.y) - self.initial_pose.position[1],
        )
        height = abs(float(pose.position.z) - self.initial_pose.position[2])
        yaw = yaw_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        initial_yaw = yaw_from_quaternion(self.initial_pose.orientation)
        return horizontal, height, abs(normalize_angle(initial_yaw - yaw))

    def confirm_initial_pose(self, now: float) -> None:
        if self.initial_pose_confirmed or not self.initial_pose_sent:
            return
        receipt = self.receipt.get("mcl_pose", 0.0)
        if receipt <= self.initial_pose_sent_at or not self.is_fresh("mcl_pose", now):
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

    def current_waypoint(self) -> Waypoint:
        return self.mission.waypoints[self.current_index]

    def send_current_goal(self) -> None:
        if self.current_index < 0 or self.state in self.TERMINAL_STATES:
            return
        waypoint = self.current_waypoint()
        qx, qy, qz, qw = quaternion_from_yaw(waypoint.yaw)
        goal = PToPMoveBase.Goal()
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        goal.target_pose.pose.position.z = waypoint.z
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        self.goal_serial += 1
        serial = self.goal_serial
        self.goal_handle = None
        self.arrival_window.reset()
        self.dwell_pose_grace.reset()
        self.waypoint_deadline = time.monotonic() + self.waypoint_timeout_sec
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.feedback_callback(
                serial, feedback
            ),
        )
        future.add_done_callback(
            lambda response: self.goal_response_callback(serial, response)
        )
        self.get_logger().info(
            "Submitted P2P waypoint %d/%d %s: "
            "[%.3f, %.3f, %.3f] yaw=%.3f dwell=%.1fs"
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

    def goal_response_callback(self, serial: int, future) -> None:
        if serial != self.goal_serial or self.state in self.TERMINAL_STATES:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self.fail(f"P2P goal request failed: {error}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.fail(f"P2P rejected waypoint {self.current_waypoint().waypoint_id}")
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.goal_result_callback(serial, result)
        )

    def feedback_callback(self, serial: int, feedback_message) -> None:
        if serial != self.goal_serial or self.state not in {
            "NAVIGATING",
            "ALIGNING",
        }:
            return
        decision = feedback_message.feedback.current_decision
        if decision == "d_align_goal_heading":
            self.transition("ALIGNING")
        elif self.state == "ALIGNING":
            self.transition("NAVIGATING")

    def goal_result_callback(self, serial: int, future) -> None:
        if serial != self.goal_serial or self.state in self.TERMINAL_STATES:
            return
        try:
            wrapped = future.result()
        except Exception as error:
            self.fail(f"P2P result failed: {error}")
            return
        self.goal_handle = None
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            detail = ""
            if wrapped is not None and wrapped.result is not None:
                detail = wrapped.result.result
            self.fail(
                f"P2P failed waypoint {self.current_waypoint().waypoint_id}"
                + (f": {detail}" if detail else "")
            )
            return
        self.arrival_window.reset()
        self.transition("SETTLING")

    def waypoint_pose_errors(self) -> tuple[float, float]:
        if self.mcl_pose is None:
            return (math.inf, math.inf)
        waypoint = self.current_waypoint()
        pose = self.mcl_pose.pose.pose
        horizontal = math.hypot(
            float(pose.position.x) - waypoint.x,
            float(pose.position.y) - waypoint.y,
        )
        yaw = yaw_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        yaw_error = abs(normalize_angle(waypoint.yaw - yaw))
        return horizontal, yaw_error

    def pose_is_within_tolerance(self) -> bool:
        horizontal, yaw_error = self.waypoint_pose_errors()
        return (
            horizontal <= self.position_tolerance
            and yaw_error <= self.yaw_tolerance
        )

    def command_is_stopped(self, now: float) -> bool:
        if self.safe_command is None or not self.is_fresh("safe_command", now):
            return False
        values = (
            self.safe_command.linear.x,
            self.safe_command.linear.y,
            self.safe_command.angular.z,
        )
        return all(
            math.isfinite(value) and abs(value) <= self.zero_epsilon
            for value in values
        )

    def complete(self) -> None:
        self.localization_health_grace.reset()
        self.dwell_pose_grace.reset()
        self.transition("COMPLETE")
        self.stop_active_goal()
        self.get_logger().info(f"P2P mission {self.mission.mission_id} completed")

    def timer_callback(self) -> None:
        now = time.monotonic()
        if self.state in self.TERMINAL_STATES:
            self.publish_state()
            return
        if self.state == "LOADING_INITIAL_POSE":
            if now > self.initial_deadline:
                self.fail(
                    "timed out waiting for map ground and initial-pose subscriber"
                )
            elif (
                self.map_ground_ready
                and now - self.map_ground_ready_at
                >= self.initial_pose_map_settle_sec
                and self.initial_pose_pub.get_subscription_count() > 0
            ):
                self.publish_initial_pose()
            return
        if self.state == "LOCALIZING":
            if now > self.initial_deadline:
                reason = (
                    "MCL never confirmed the saved fixed initial pose"
                    if not self.initial_pose_confirmed
                    else "fixed initial pose did not produce healthy TRACKING"
                )
                self.fail(reason)
                return
            self.confirm_initial_pose(now)
            if (
                self.saw_post_seed_nontracking
                and self.initial_pose_confirmed
                and self.localization_is_fresh_and_healthy(now)
            ):
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
        if not self.active_localization_allows_progress(now):
            return
        if self.state == "ARMING":
            return
        if self.state in {"NAVIGATING", "ALIGNING", "SETTLING"}:
            if now > self.waypoint_deadline:
                self.fail(
                    f"waypoint {self.current_waypoint().waypoint_id} timed out"
                )
                return
        if self.state == "SETTLING":
            if self.arrival_window.update(
                now,
                self.pose_is_within_tolerance(),
                self.command_is_stopped(now),
            ):
                waypoint = self.current_waypoint()
                self.dwell_deadline = now + waypoint.dwell_sec
                horizontal, yaw_error = self.waypoint_pose_errors()
                self.get_logger().info(
                    "Waypoint %s settled; starting %.3fs dwell "
                    "(xy_error=%.3f yaw_error=%.3f)"
                    % (
                        waypoint.waypoint_id,
                        waypoint.dwell_sec,
                        horizontal,
                        yaw_error,
                    )
                )
                self.transition("DWELLING")
            return
        if self.state == "DWELLING":
            if not self.command_is_stopped(now):
                self.fail("controller produced motion during waypoint dwell")
                return
            horizontal, yaw_error = self.waypoint_pose_errors()
            within_exit_tolerance = (
                horizontal <= self.dwell_exit_position_tolerance
                and yaw_error <= self.dwell_exit_yaw_tolerance
            )
            if not within_exit_tolerance:
                first_excursion = not self.dwell_pose_grace.active
                expired = self.dwell_pose_grace.mark_unhealthy(now)
                held_sec = self.dwell_pose_grace.elapsed(now)
                if first_excursion:
                    self.get_logger().warning(
                        "Waypoint %s pose exceeded dwell exit tolerance; "
                        "holding for grace "
                        "(xy_error=%.3f limit=%.3f yaw_error=%.3f "
                        "limit=%.3f grace=%.3fs)"
                        % (
                            self.current_waypoint().waypoint_id,
                            horizontal,
                            self.dwell_exit_position_tolerance,
                            yaw_error,
                            self.dwell_exit_yaw_tolerance,
                            self.dwell_pose_grace_sec,
                        )
                    )
                if expired:
                    self.fail(
                        "waypoint pose remained outside dwell exit tolerance "
                        "for %.3fs "
                        "(xy_error=%.3f limit=%.3f yaw_error=%.3f "
                        "limit=%.3f)"
                        % (
                            held_sec,
                            horizontal,
                            self.dwell_exit_position_tolerance,
                            yaw_error,
                            self.dwell_exit_yaw_tolerance,
                        )
                    )
                return
            if self.dwell_pose_grace.active:
                held_sec = self.dwell_pose_grace.mark_healthy(now)
                waypoint = self.current_waypoint()
                self.dwell_deadline = now + waypoint.dwell_sec
                self.get_logger().info(
                    "Waypoint %s pose recovered after %.3fs; "
                    "restarting %.3fs dwell "
                    "(xy_error=%.3f yaw_error=%.3f)"
                    % (
                        waypoint.waypoint_id,
                        held_sec,
                        waypoint.dwell_sec,
                        horizontal,
                        yaw_error,
                    )
                )
            if now < self.dwell_deadline:
                return
            if self.current_index + 1 >= len(self.mission.waypoints):
                self.complete()
                return
            self.current_index += 1
            self.send_current_goal()


def main() -> None:
    rclpy.init()
    try:
        node = P2PMissionExecutor()
    except (MissionValidationError, OSError, ValueError) as error:
        get_logger("p2p_mission_executor").fatal(str(error))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok(context=node.context):
            node.fail("executor interrupted")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
