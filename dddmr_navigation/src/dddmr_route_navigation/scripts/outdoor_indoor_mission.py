#!/usr/bin/env python3
"""One-map mission action coordinating recorded-route and indoor navigation."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from dddmr_sys_core.action import OutdoorIndoorMission, PToPMoveBase
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool

from outdoor_indoor_mission_lib import (
    ContinuousStopDetector,
    load_validated_route,
    normalized_state,
    validate_planar_goal,
)


class MissionFailure(RuntimeError):
    def __init__(self, result_code: int, message: str) -> None:
        super().__init__(message)
        self.result_code = result_code


class MissionCanceled(RuntimeError):
    pass


class OutdoorIndoorMissionServer(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("outdoor_indoor_mission", **node_kwargs)
        self.callback_group = ReentrantCallbackGroup()
        self.route_directory = Path(self._string_parameter("route_directory", "/root/dddmr_bags/routes"))
        self.active_map_directory = Path(self._string_parameter("active_map_directory", "."))
        self.expected_frame = self._string_parameter("expected_frame", "map")
        self.route_topic = self._string_parameter("route_topic", "/recorded_route")
        self.route_status_topic = self._string_parameter(
            "route_status_topic", "/recorded_route_controller/status"
        )
        self.route_progress_topic = self._string_parameter(
            "route_progress_topic", "/recorded_route_controller/progress"
        )
        self.route_enable_service = self._string_parameter(
            "route_enable_service", "/recorded_route_controller/set_enabled"
        )
        self.p2p_action_name = self._string_parameter("p2p_action_name", "/p2p_move_base")
        self.p2p_enable_service = self._string_parameter(
            "p2p_enable_service", "/p2p_move_base/set_enabled"
        )
        self.control_mode_topic = self._string_parameter(
            "control_mode_topic", "/outdoor_indoor_mission/control_mode"
        )
        self.localization_status_topic = self._string_parameter(
            "localization_status_topic", "/localization_status"
        )
        self.localization_health_topic = self._string_parameter(
            "localization_health_topic", "/localization_health"
        )
        self.observation_topic = self._string_parameter(
            "observation_topic", "/perception_3d_local/lidar/current_observation"
        )
        self.odom_topic = self._string_parameter(
            "odom_topic", "/dddmr_go2/robot_odom_standard"
        )
        self.action_name = self._string_parameter(
            "action_name", "/outdoor_indoor_mission"
        )

        self.readiness_timeout_sec = self._positive_parameter("readiness_timeout_sec", 30.0)
        self.route_ready_timeout_sec = self._positive_parameter("route_ready_timeout_sec", 10.0)
        self.service_timeout_sec = self._positive_parameter("service_timeout_sec", 5.0)
        self.outdoor_timeout_sec = self._positive_parameter("outdoor_timeout_sec", 900.0)
        self.transition_timeout_sec = self._positive_parameter("transition_timeout_sec", 15.0)
        self.indoor_timeout_sec = self._positive_parameter("indoor_timeout_sec", 300.0)
        self.transition_stop_duration_sec = self._positive_parameter(
            "transition_stop_duration_sec", 1.0
        )
        self.localization_timeout_sec = self._positive_parameter(
            "localization_timeout_sec", 0.75
        )
        self.localization_health_timeout_sec = self._positive_parameter(
            "localization_health_timeout_sec", 0.75
        )
        self.observation_receive_timeout_sec = self._positive_parameter(
            "observation_receive_timeout_sec", 0.35
        )
        self.observation_sensor_timeout_sec = self._positive_parameter(
            "observation_sensor_timeout_sec", 0.35
        )
        self.observation_future_tolerance_sec = self._nonnegative_parameter(
            "observation_future_tolerance_sec", 0.05
        )
        self.odom_timeout_sec = self._positive_parameter("odom_timeout_sec", 0.75)
        self.stop_max_linear_speed = self._nonnegative_parameter(
            "stop_max_linear_speed", 0.03
        )
        self.stop_max_angular_speed = self._nonnegative_parameter(
            "stop_max_angular_speed", 0.05
        )
        self.max_route_segment_length = self._positive_parameter(
            "max_route_segment_length", 2.0
        )
        self.feedback_period_sec = self._positive_parameter("feedback_period_sec", 0.20)
        self.mode_heartbeat_period_sec = self._positive_parameter(
            "mode_heartbeat_period_sec", 0.10
        )

        self.lock = threading.RLock()
        self.goal_reserved = False
        self.phase = "IDLE"
        self.detail = "waiting for a mission"
        self.control_mode = "none"
        self.route_status = ""
        self.route_progress = 0.0
        self.localization_status = ""
        self.localization_received_monotonic: Optional[float] = None
        self.localization_health = ""
        self.localization_health_received_monotonic: Optional[float] = None
        self.observation_received_monotonic: Optional[float] = None
        self.observation_stamp_ros: Optional[float] = None
        self.odom_received_monotonic: Optional[float] = None
        self.odom_linear_speed = math.inf
        self.odom_angular_speed = math.inf
        self.current_p2p_goal = None

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.route_publisher = self.create_publisher(
            PathMessage, self.route_topic, transient_qos
        )
        self.control_mode_publisher = self.create_publisher(
            String, self.control_mode_topic, 10
        )
        self.status_publisher = self.create_publisher(String, "~/status", transient_qos)
        self.create_subscription(
            String,
            self.route_status_topic,
            self._route_status_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float64,
            self.route_progress_topic,
            self._route_progress_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            self.localization_status_topic,
            self._localization_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            self.localization_health_topic,
            self._localization_health_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PointCloud2,
            self.observation_topic,
            self._observation_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self.route_enable_client = self.create_client(
            SetBool, self.route_enable_service, callback_group=self.callback_group
        )
        self.p2p_enable_client = self.create_client(
            SetBool, self.p2p_enable_service, callback_group=self.callback_group
        )
        self.p2p_client = ActionClient(
            self, PToPMoveBase, self.p2p_action_name, callback_group=self.callback_group
        )
        self.action_server = ActionServer(
            self,
            OutdoorIndoorMission,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.create_timer(
            self.mode_heartbeat_period_sec,
            self._mode_heartbeat,
            callback_group=self.callback_group,
        )
        self._publish_status()
        self.get_logger().warn(
            "Outdoor/indoor mission server starts IDLE with command mux mode NONE. "
            "Every mission validates the route against the active unified-map fingerprint."
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

    def _goal_callback(self, goal_request) -> GoalResponse:
        with self.lock:
            if self.goal_reserved:
                self.get_logger().warn("Rejecting mission because another mission is active")
                return GoalResponse.REJECT
            if not goal_request.mission_id.strip() or not goal_request.route_id.strip():
                self.get_logger().warn("Rejecting mission with an empty mission_id or route_id")
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().warn("Mission cancellation requested")
        return CancelResponse.ACCEPT

    def _route_status_callback(self, message: String) -> None:
        with self.lock:
            self.route_status = message.data

    def _route_progress_callback(self, message: Float64) -> None:
        with self.lock:
            self.route_progress = max(0.0, min(1.0, float(message.data)))

    def _localization_callback(self, message: String) -> None:
        with self.lock:
            self.localization_status = message.data.strip().upper()
            self.localization_received_monotonic = time.monotonic()

    def _localization_health_callback(self, message: String) -> None:
        with self.lock:
            self.localization_health = message.data.strip().upper()
            self.localization_health_received_monotonic = time.monotonic()

    def _observation_callback(self, message: PointCloud2) -> None:
        with self.lock:
            self.observation_received_monotonic = time.monotonic()
            stamp = message.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                self.observation_stamp_ros = None
            else:
                self.observation_stamp_ros = stamp.sec + stamp.nanosec / 1.0e9

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        with self.lock:
            self.odom_received_monotonic = time.monotonic()
            self.odom_linear_speed = math.sqrt(
                linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
            )
            self.odom_angular_speed = math.sqrt(
                angular.x * angular.x + angular.y * angular.y + angular.z * angular.z
            )

    def _mode_heartbeat(self) -> None:
        with self.lock:
            mode = self.control_mode
        message = String()
        message.data = mode
        self.control_mode_publisher.publish(message)

    def _set_phase(self, phase: str, detail: str) -> None:
        with self.lock:
            self.phase = phase
            self.detail = detail
        self.get_logger().info(f"Mission phase {phase}: {detail}")
        self._publish_status()

    def _set_control_mode(self, mode: str) -> None:
        if mode not in {"none", "outdoor", "indoor"}:
            raise ValueError(f"invalid control mode {mode!r}")
        with self.lock:
            self.control_mode = mode
        self._mode_heartbeat()

    def _publish_status(self) -> None:
        with self.lock:
            status = {
                "phase": self.phase,
                "detail": self.detail,
                "control_mode": self.control_mode,
                "outdoor_progress": self.route_progress,
            }
        message = String()
        message.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self.status_publisher.publish(message)

    def _publish_feedback(self, goal_handle) -> None:
        with self.lock:
            phase = self.phase
            detail = self.detail
            progress = self.route_progress
        feedback = OutdoorIndoorMission.Feedback()
        feedback.phase = phase
        feedback.outdoor_progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)
        self._publish_status()

    def _health_reason(self) -> Optional[str]:
        now_monotonic = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds / 1.0e9
        with self.lock:
            localization_status = self.localization_status
            localization_time = self.localization_received_monotonic
            localization_health = self.localization_health
            localization_health_time = self.localization_health_received_monotonic
            observation_time = self.observation_received_monotonic
            observation_stamp = self.observation_stamp_ros
            odom_time = self.odom_received_monotonic
        if localization_time is None:
            return "no localization status received"
        localization_age = now_monotonic - localization_time
        if not 0.0 <= localization_age <= self.localization_timeout_sec:
            return f"localization status stale: age={localization_age:.3f}s"
        if localization_status != "TRACKING":
            return f"localization state is {localization_status or '<empty>'}, not TRACKING"
        if localization_health_time is None:
            return "no localization health received"
        localization_health_age = now_monotonic - localization_health_time
        if not 0.0 <= localization_health_age <= self.localization_health_timeout_sec:
            return f"localization health stale: age={localization_health_age:.3f}s"
        if localization_health != "HEALTHY":
            return f"localization health is {localization_health or '<empty>'}, not HEALTHY"
        if observation_time is None:
            return "no local obstacle observation received"
        observation_receive_age = now_monotonic - observation_time
        if not 0.0 <= observation_receive_age <= self.observation_receive_timeout_sec:
            return f"local obstacle observation stale on receipt: age={observation_receive_age:.3f}s"
        if observation_stamp is None:
            return "local obstacle observation has no sensor timestamp"
        sensor_age = now_ros - observation_stamp
        if sensor_age < -self.observation_future_tolerance_sec:
            return f"local obstacle observation is from the future: age={sensor_age:.3f}s"
        if sensor_age > self.observation_sensor_timeout_sec:
            return f"local obstacle observation sensor data stale: age={sensor_age:.3f}s"
        if odom_time is None:
            return "no standardized odometry received"
        odom_age = now_monotonic - odom_time
        if not 0.0 <= odom_age <= self.odom_timeout_sec:
            return f"standardized odometry stale: age={odom_age:.3f}s"
        return None

    def _check_cancel(self, goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            raise MissionCanceled("mission canceled by client")

    def _wait_until(self, goal_handle, timeout_sec: float, predicate, failure_message: str) -> None:
        deadline = time.monotonic() + timeout_sec
        next_feedback = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            outcome = predicate()
            if outcome is True:
                return
            if isinstance(outcome, str):
                raise MissionFailure(OutdoorIndoorMission.Result.TRANSITION_FAILED, outcome)
            now = time.monotonic()
            if now >= next_feedback:
                self._publish_feedback(goal_handle)
                next_feedback = now + self.feedback_period_sec
            time.sleep(0.02)
        raise MissionFailure(OutdoorIndoorMission.Result.TRANSITION_FAILED, failure_message)

    def _wait_future(self, goal_handle, future, timeout_sec: float, failure_code: int, message: str):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            if future.done():
                try:
                    return future.result()
                except Exception as exception:
                    raise MissionFailure(failure_code, f"{message}: {exception}") from exception
            time.sleep(0.02)
        raise MissionFailure(failure_code, message)

    def _set_route_enabled(self, goal_handle, enabled: bool) -> None:
        if not self.route_enable_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise MissionFailure(
                OutdoorIndoorMission.Result.TRANSITION_FAILED,
                f"route enable service unavailable: {self.route_enable_service}",
            )
        request = SetBool.Request()
        request.data = enabled
        response = self._wait_future(
            goal_handle,
            self.route_enable_client.call_async(request),
            self.service_timeout_sec,
            OutdoorIndoorMission.Result.TRANSITION_FAILED,
            "route enable service timed out",
        )
        if response is None or not response.success:
            reason = "empty response" if response is None else response.message
            raise MissionFailure(
                OutdoorIndoorMission.Result.OUTDOOR_FAILED if enabled else OutdoorIndoorMission.Result.TRANSITION_FAILED,
                f"route controller rejected {'enable' if enabled else 'disable'}: {reason}",
            )

    def _set_p2p_enabled(self, goal_handle, enabled: bool) -> None:
        if not self.p2p_enable_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise MissionFailure(
                OutdoorIndoorMission.Result.TRANSITION_FAILED,
                f"P2P enable service unavailable: {self.p2p_enable_service}",
            )
        request = SetBool.Request()
        request.data = enabled
        response = self._wait_future(
            goal_handle,
            self.p2p_enable_client.call_async(request),
            self.service_timeout_sec,
            OutdoorIndoorMission.Result.TRANSITION_FAILED,
            "P2P enable service timed out",
        )
        if response is None or not response.success:
            reason = "empty response" if response is None else response.message
            raise MissionFailure(
                OutdoorIndoorMission.Result.TRANSITION_FAILED,
                f"P2P controller rejected {'enable' if enabled else 'disable'}: {reason}",
            )

    def _route_message(self, document) -> PathMessage:
        message = PathMessage()
        message.header.frame_id = str(document["frame_id"])
        message.header.stamp = self.get_clock().now().to_msg()
        for point in document["points"]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point["x"]
            pose.pose.position.y = point["y"]
            pose.pose.position.z = point["z"]
            pose.pose.orientation.x = point["qx"]
            pose.pose.orientation.y = point["qy"]
            pose.pose.orientation.z = point["qz"]
            pose.pose.orientation.w = point["qw"]
            message.poses.append(pose)
        return message

    def _wait_for_health(self, goal_handle) -> None:
        def ready():
            return self._health_reason() is None

        self._wait_until(
            goal_handle,
            self.readiness_timeout_sec,
            ready,
            "timed out waiting for TRACKING localization, fresh perception, and fresh odometry",
        )

    def _wait_for_measured_stop(self, goal_handle, timeout_sec: float) -> None:
        detector = ContinuousStopDetector(
            self.transition_stop_duration_sec,
            self.odom_timeout_sec,
            self.stop_max_linear_speed,
            self.stop_max_angular_speed,
        )
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            health_reason = self._health_reason()
            if health_reason is not None:
                raise MissionFailure(
                    OutdoorIndoorMission.Result.TRANSITION_FAILED,
                    f"cannot confirm stopped state: {health_reason}",
                )
            now = time.monotonic()
            with self.lock:
                odom_time = self.odom_received_monotonic
                linear_speed = self.odom_linear_speed
                angular_speed = self.odom_angular_speed
            if odom_time is not None and detector.update(
                now, odom_time, linear_speed, angular_speed
            ):
                return
            self._publish_feedback(goal_handle)
            time.sleep(min(0.05, self.feedback_period_sec))
        raise MissionFailure(
            OutdoorIndoorMission.Result.TRANSITION_FAILED,
            "timed out waiting for measured zero velocity",
        )

    def _monitor_outdoor(self, goal_handle) -> None:
        deadline = time.monotonic() + self.outdoor_timeout_sec
        next_feedback = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            with self.lock:
                state = normalized_state(self.route_status)
                status = self.route_status
            if state == "COMPLETED":
                return
            if state in {"FAULT", "DISABLED"}:
                raise MissionFailure(
                    OutdoorIndoorMission.Result.OUTDOOR_FAILED,
                    f"outdoor route stopped in {status or state}",
                )
            health_reason = self._health_reason()
            if health_reason is not None:
                raise MissionFailure(
                    OutdoorIndoorMission.Result.OUTDOOR_FAILED,
                    f"outdoor health gate failed: {health_reason}",
                )
            now = time.monotonic()
            if now >= next_feedback:
                self._publish_feedback(goal_handle)
                next_feedback = now + self.feedback_period_sec
            time.sleep(0.02)
        raise MissionFailure(
            OutdoorIndoorMission.Result.OUTDOOR_FAILED,
            "outdoor route exceeded its mission timeout",
        )

    def _run_indoor(self, goal_handle, target: PoseStamped) -> None:
        if not self.p2p_client.wait_for_server(timeout_sec=self.service_timeout_sec):
            raise MissionFailure(
                OutdoorIndoorMission.Result.INDOOR_FAILED,
                f"indoor action unavailable: {self.p2p_action_name}",
            )
        p2p_goal = PToPMoveBase.Goal()
        p2p_goal.target_pose = target
        p2p_goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        self._set_p2p_enabled(goal_handle, True)
        self._set_control_mode("indoor")
        client_goal = self._wait_future(
            goal_handle,
            self.p2p_client.send_goal_async(p2p_goal),
            self.service_timeout_sec,
            OutdoorIndoorMission.Result.INDOOR_FAILED,
            "timed out sending indoor goal",
        )
        if client_goal is None or not client_goal.accepted:
            raise MissionFailure(
                OutdoorIndoorMission.Result.INDOOR_FAILED,
                "p2p_move_base rejected the indoor goal",
            )
        with self.lock:
            self.current_p2p_goal = client_goal
        result_future = client_goal.get_result_async()
        deadline = time.monotonic() + self.indoor_timeout_sec
        next_feedback = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            if result_future.done():
                wrapped_result = result_future.result()
                if wrapped_result is not None and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                    return
                status = None if wrapped_result is None else wrapped_result.status
                raise MissionFailure(
                    OutdoorIndoorMission.Result.INDOOR_FAILED,
                    f"indoor navigation ended with action status {status}",
                )
            health_reason = self._health_reason()
            if health_reason is not None:
                client_goal.cancel_goal_async()
                raise MissionFailure(
                    OutdoorIndoorMission.Result.INDOOR_FAILED,
                    f"indoor health gate failed: {health_reason}",
                )
            now = time.monotonic()
            if now >= next_feedback:
                self._publish_feedback(goal_handle)
                next_feedback = now + self.feedback_period_sec
            time.sleep(0.02)
        client_goal.cancel_goal_async()
        raise MissionFailure(
            OutdoorIndoorMission.Result.INDOOR_FAILED,
            "indoor navigation exceeded its mission timeout",
        )

    def _best_effort_shutdown(self) -> None:
        self._set_control_mode("none")
        with self.lock:
            p2p_goal = self.current_p2p_goal
            self.current_p2p_goal = None
        if p2p_goal is not None:
            try:
                p2p_goal.cancel_goal_async()
            except Exception as exception:
                self.get_logger().error(f"Failed to request P2P cancel: {exception}")

        pending = []
        for label, client in (
            ("route", self.route_enable_client),
            ("P2P", self.p2p_enable_client),
        ):
            if not client.service_is_ready():
                self.get_logger().warn(
                    f"Cannot confirm {label} disable because its service is unavailable"
                )
                continue
            try:
                request = SetBool.Request()
                request.data = False
                pending.append((label, client.call_async(request)))
            except Exception as exception:
                self.get_logger().error(f"Failed to request {label} disable: {exception}")

        deadline = time.monotonic() + self.service_timeout_sec
        while (
            rclpy.ok()
            and any(not future.done() for _, future in pending)
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
        for label, future in pending:
            if not future.done():
                self.get_logger().error(f"Timed out confirming {label} disable")
                continue
            try:
                response = future.result()
            except Exception as exception:
                self.get_logger().error(f"Failed to confirm {label} disable: {exception}")
                continue
            if response is None or not response.success:
                reason = "empty response" if response is None else response.message
                self.get_logger().error(f"{label} disable was not confirmed: {reason}")

    def _execute_callback(self, goal_handle):
        result = OutdoorIndoorMission.Result()
        request = goal_handle.request
        try:
            self._set_control_mode("none")
            with self.lock:
                self.route_status = ""
                self.route_progress = 0.0
            self._set_phase("VALIDATING", f"validating mission {request.mission_id}")
            validate_planar_goal(
                request.indoor_goal.header.frame_id,
                (
                    request.indoor_goal.pose.position.x,
                    request.indoor_goal.pose.position.y,
                    request.indoor_goal.pose.position.z,
                ),
                (
                    request.indoor_goal.pose.orientation.x,
                    request.indoor_goal.pose.orientation.y,
                    request.indoor_goal.pose.orientation.z,
                    request.indoor_goal.pose.orientation.w,
                ),
                self.expected_frame,
            )
            route_path, document, map_sha256 = load_validated_route(
                self.route_directory,
                request.route_id,
                self.active_map_directory,
                self.expected_frame,
                self.max_route_segment_length,
            )
            self.get_logger().info(
                f"Validated route {route_path} against unified map {map_sha256}"
            )
            self._wait_for_health(goal_handle)
            self._set_p2p_enabled(goal_handle, False)

            route_message = self._route_message(document)
            self.route_publisher.publish(route_message)

            def route_ready():
                with self.lock:
                    state = normalized_state(self.route_status)
                    status = self.route_status
                if state == "READY":
                    return True
                if state == "FAULT":
                    return f"route controller rejected route: {status}"
                return False

            self._wait_until(
                goal_handle,
                self.route_ready_timeout_sec,
                route_ready,
                "timed out waiting for route controller READY",
            )
            self._set_route_enabled(goal_handle, True)
            self._set_control_mode("outdoor")
            self._set_phase("OUTDOOR_TRACKING", f"following route {request.route_id}")
            self._monitor_outdoor(goal_handle)

            self._set_control_mode("none")
            self._set_phase(
                "TRANSITION_STOP",
                "outdoor route complete; disabling route control and confirming stop",
            )
            self._set_route_enabled(goal_handle, False)
            self._wait_for_measured_stop(goal_handle, self.transition_timeout_sec)

            self._set_phase("INDOOR_NAVIGATION", "sending indoor point-to-point goal")
            self._run_indoor(goal_handle, request.indoor_goal)
            self._set_control_mode("none")
            self._set_p2p_enabled(goal_handle, False)
            self._set_phase("COMPLETED", "indoor goal reached; confirming final stop")
            self._wait_for_measured_stop(goal_handle, self.transition_timeout_sec)
            result.status = OutdoorIndoorMission.Result.SUCCESS
            result.message = "outdoor route and indoor goal completed on the unified map"
            goal_handle.succeed()
            return result
        except MissionCanceled as exception:
            self._set_control_mode("none")
            self._set_phase("CANCELED", str(exception))
            result.status = OutdoorIndoorMission.Result.CANCELED
            result.message = str(exception)
            goal_handle.canceled()
            return result
        except MissionFailure as exception:
            self._set_control_mode("none")
            self._set_phase("FAILED", str(exception))
            result.status = exception.result_code
            result.message = str(exception)
            goal_handle.abort()
            return result
        except (OSError, ValueError, json.JSONDecodeError) as exception:
            self._set_control_mode("none")
            self._set_phase("FAILED", str(exception))
            result.status = OutdoorIndoorMission.Result.REJECTED
            result.message = str(exception)
            goal_handle.abort()
            return result
        except Exception as exception:
            self._set_control_mode("none")
            self._set_phase("FAILED", f"unexpected mission error: {exception}")
            result.status = OutdoorIndoorMission.Result.TRANSITION_FAILED
            result.message = f"unexpected mission error: {exception}"
            goal_handle.abort()
            return result
        finally:
            self._best_effort_shutdown()
            with self.lock:
                self.goal_reserved = False
                self.current_p2p_goal = None


def main() -> None:
    rclpy.init()
    node = OutdoorIndoorMissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.action_server.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
