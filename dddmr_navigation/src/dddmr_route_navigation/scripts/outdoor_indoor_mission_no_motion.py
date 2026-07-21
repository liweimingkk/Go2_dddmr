#!/usr/bin/env python3
"""Run a synthetic, network-isolated outdoor-to-indoor mission with no motion topics."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import List, Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from dddmr_sys_core.action import OutdoorIndoorMission, PToPMoveBase
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool

from outdoor_indoor_mission import OutdoorIndoorMissionServer
from outdoor_indoor_mission_lib import load_validated_route


FORBIDDEN_MOTION_TOPICS = {
    "/cmd_vel",
    "/api/sport/request",
    "/dddmr_go2/dry_run_cmd_vel",
    "/dddmr_go2/indoor_cmd_vel",
    "/dddmr_go2/outdoor_cmd_vel",
}


def wait_future(future, timeout_sec: float, description: str):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise RuntimeError(f"timed out waiting for {description}")


def require_subsequence(values: Sequence[str], expected: Sequence[str], label: str) -> None:
    cursor = 0
    for value in values:
        if cursor < len(expected) and value == expected[cursor]:
            cursor += 1
    if cursor != len(expected):
        raise RuntimeError(f"{label} did not contain {list(expected)}: {list(values)}")


def yaw_from_point(point) -> float:
    qx, qy, qz, qw = (float(point[key]) for key in ("qx", "qy", "qz", "qw"))
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def angle_error(lhs: float, rhs: float) -> float:
    return abs(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


class NoMotionPlant(Node):
    """Mock health, route, and P2P endpoints without any velocity publisher."""

    def __init__(self) -> None:
        super().__init__("outdoor_indoor_no_motion")
        self.callback_group = ReentrantCallbackGroup()
        self.mission_id = self._string_parameter(
            "mission_id", "my-route-a-return-no-motion"
        )
        self.route_id = self._string_parameter("route_id", "my_route_a")
        self.route_directory = Path(
            self._string_parameter("route_directory", "/root/dddmr_bags/routes")
        )
        self.active_map_directory = Path(
            self._string_parameter("active_map_directory", "/root/dddmr_bags")
        )
        self.expected_frame = self._string_parameter("expected_frame", "map")
        self.return_goal_x = self._finite_parameter("return_goal_x", 0.0)
        self.return_goal_y = self._finite_parameter("return_goal_y", 0.0)
        self.return_goal_z = self._finite_parameter("return_goal_z", 0.0)
        self.return_goal_yaw = self._finite_parameter("return_goal_yaw", 0.0)
        self.route_complete_delay_sec = self._positive_parameter(
            "route_complete_delay_sec", 0.30
        )
        self.p2p_complete_delay_sec = self._positive_parameter(
            "p2p_complete_delay_sec", 0.20
        )
        self.stop_confirmation_sec = self._positive_parameter(
            "stop_confirmation_sec", 0.20
        )
        self.test_timeout_sec = self._positive_parameter("test_timeout_sec", 12.0)

        _, self.route_document, self.map_sha256 = load_validated_route(
            self.route_directory,
            self.route_id,
            self.active_map_directory,
            self.expected_frame,
        )
        self._validate_return_goal()

        self.lock = threading.RLock()
        self.route_loaded = False
        self.route_error = ""
        self.route_enabled_at: Optional[float] = None
        self.route_completed = False
        self.p2p_enabled = False
        self.p2p_goal: Optional[PoseStamped] = None
        self.route_enable_count = 0
        self.route_disable_count = 0
        self.p2p_enable_count = 0
        self.p2p_disable_count = 0
        self.phases: List[str] = []
        self.modes: List[str] = []

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.route_status_publisher = self.create_publisher(
            String, "/recorded_route_controller/status", transient_qos
        )
        self.route_progress_publisher = self.create_publisher(
            Float64, "/recorded_route_controller/progress", 10
        )
        self.localization_status_publisher = self.create_publisher(
            String, "/localization_status", 10
        )
        self.localization_health_publisher = self.create_publisher(
            String, "/localization_health", 10
        )
        self.observation_publisher = self.create_publisher(
            PointCloud2, "/perception_3d_local/lidar/current_observation", 10
        )
        self.odom_publisher = self.create_publisher(
            Odometry, "/dddmr_go2/robot_odom_standard", 10
        )
        self.create_subscription(
            PathMessage,
            "/recorded_route",
            self._route_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/outdoor_indoor_mission/status",
            self._mission_status_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/outdoor_indoor_mission/control_mode",
            self._mode_callback,
            10,
            callback_group=self.callback_group,
        )
        self.route_enable_service = self.create_service(
            SetBool,
            "/recorded_route_controller/set_enabled",
            self._route_enable_callback,
            callback_group=self.callback_group,
        )
        self.p2p_enable_service = self.create_service(
            SetBool,
            "/p2p_move_base/set_enabled",
            self._p2p_enable_callback,
            callback_group=self.callback_group,
        )
        self.p2p_action_server = ActionServer(
            self,
            PToPMoveBase,
            "/p2p_move_base",
            execute_callback=self._execute_p2p,
            goal_callback=self._p2p_goal_callback,
            cancel_callback=self._p2p_cancel_callback,
            callback_group=self.callback_group,
        )
        self.mission_client = ActionClient(
            self,
            OutdoorIndoorMission,
            "/outdoor_indoor_mission",
            callback_group=self.callback_group,
        )
        self.create_timer(0.05, self._tick, callback_group=self.callback_group)

    def _string_parameter(self, name: str, default: str) -> str:
        value = str(self.declare_parameter(name, default).value).strip()
        if not value:
            raise ValueError(f"{name} must not be empty")
        return value

    def _finite_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _positive_parameter(self, name: str, default: float) -> float:
        value = self._finite_parameter(name, default)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _validate_return_goal(self) -> None:
        start = self.route_document["points"][0]
        configured = (self.return_goal_x, self.return_goal_y, self.return_goal_z)
        recorded = tuple(float(start[axis]) for axis in ("x", "y", "z"))
        if any(abs(lhs - rhs) > 1.0e-6 for lhs, rhs in zip(configured, recorded)):
            raise ValueError(
                f"configured return goal {configured} is not route start {recorded}"
            )
        recorded_yaw = yaw_from_point(start)
        if angle_error(self.return_goal_yaw, recorded_yaw) > 1.0e-6:
            raise ValueError(
                f"configured return yaw {self.return_goal_yaw} is not route start yaw "
                f"{recorded_yaw}"
            )

    def mission_parameter_overrides(self) -> List[Parameter]:
        return [
            Parameter("route_directory", value=str(self.route_directory)),
            Parameter("active_map_directory", value=str(self.active_map_directory)),
            Parameter("expected_frame", value=self.expected_frame),
            Parameter("readiness_timeout_sec", value=3.0),
            Parameter("route_ready_timeout_sec", value=2.0),
            Parameter("service_timeout_sec", value=2.0),
            Parameter("outdoor_timeout_sec", value=3.0),
            Parameter("transition_timeout_sec", value=3.0),
            Parameter("indoor_timeout_sec", value=3.0),
            Parameter(
                "transition_stop_duration_sec", value=self.stop_confirmation_sec
            ),
            Parameter("localization_timeout_sec", value=0.50),
            Parameter("localization_health_timeout_sec", value=0.50),
            Parameter("observation_receive_timeout_sec", value=0.50),
            Parameter("observation_sensor_timeout_sec", value=0.50),
            Parameter("odom_timeout_sec", value=0.50),
            Parameter("feedback_period_sec", value=0.05),
            Parameter("mode_heartbeat_period_sec", value=0.05),
        ]

    def return_goal(self) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self.expected_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.return_goal_x
        goal.pose.position.y = self.return_goal_y
        goal.pose.position.z = self.return_goal_z
        goal.pose.orientation.z = math.sin(self.return_goal_yaw * 0.5)
        goal.pose.orientation.w = math.cos(self.return_goal_yaw * 0.5)
        return goal

    def _route_callback(self, message: PathMessage) -> None:
        error = ""
        if message.header.frame_id != self.expected_frame:
            error = f"route frame is {message.header.frame_id!r}"
        elif len(message.poses) != len(self.route_document["points"]):
            error = "route pose count differs from validated document"
        else:
            for axis in ("x", "y", "z"):
                actual = float(getattr(message.poses[0].pose.position, axis))
                expected = float(self.route_document["points"][0][axis])
                if abs(actual - expected) > 1.0e-6:
                    error = f"route start {axis} differs from validated document"
                    break
        with self.lock:
            self.route_error = error
            self.route_loaded = not error
        self._publish_route_state("FAULT" if error else "READY", error or "mock route ready")

    def _route_enable_callback(self, request, response):
        with self.lock:
            if request.data and (not self.route_loaded or self.route_error):
                response.success = False
                response.message = self.route_error or "mock route is not loaded"
                return response
            if request.data:
                self.route_enable_count += 1
                self.route_enabled_at = time.monotonic()
                self.route_completed = False
            else:
                self.route_disable_count += 1
                self.route_enabled_at = None
        response.success = True
        response.message = "mock route enabled" if request.data else "mock route disabled"
        self._publish_route_state(
            "TRACKING" if request.data else "DISABLED", response.message
        )
        return response

    def _p2p_enable_callback(self, request, response):
        with self.lock:
            self.p2p_enabled = bool(request.data)
            if request.data:
                self.p2p_enable_count += 1
            else:
                self.p2p_disable_count += 1
        response.success = True
        response.message = "mock P2P enabled" if request.data else "mock P2P disabled"
        return response

    def _p2p_goal_callback(self, _request) -> GoalResponse:
        with self.lock:
            enabled = self.p2p_enabled
        return GoalResponse.ACCEPT if enabled else GoalResponse.REJECT

    def _p2p_cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_p2p(self, goal_handle):
        with self.lock:
            self.p2p_goal = goal_handle.request.target_pose
        deadline = time.monotonic() + self.p2p_complete_delay_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                result = PToPMoveBase.Result()
                result.status = 2
                result.result = "mock P2P canceled"
                goal_handle.canceled()
                return result
            time.sleep(0.01)
        result = PToPMoveBase.Result()
        result.status = 1
        result.result = "mock P2P reached route start"
        goal_handle.succeed()
        return result

    def _tick(self) -> None:
        tracking = String()
        tracking.data = "TRACKING"
        healthy = String()
        healthy.data = "HEALTHY"
        observation = PointCloud2()
        observation.header.frame_id = self.expected_frame
        observation.header.stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = observation.header.stamp
        self.localization_status_publisher.publish(tracking)
        self.localization_health_publisher.publish(healthy)
        self.observation_publisher.publish(observation)
        self.odom_publisher.publish(odom)

        complete = False
        with self.lock:
            if (
                self.route_enabled_at is not None
                and not self.route_completed
                and time.monotonic() - self.route_enabled_at
                >= self.route_complete_delay_sec
            ):
                self.route_completed = True
                complete = True
        if complete:
            progress = Float64()
            progress.data = 1.0
            self.route_progress_publisher.publish(progress)
            self._publish_route_state("COMPLETED", "mock route reached endpoint")

    def _publish_route_state(self, state: str, detail: str) -> None:
        message = String()
        message.data = f"{state}: {detail}"
        self.route_status_publisher.publish(message)

    def _mission_status_callback(self, message: String) -> None:
        try:
            phase = str(json.loads(message.data)["phase"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self.lock:
            if not self.phases or self.phases[-1] != phase:
                self.phases.append(phase)

    def _mode_callback(self, message: String) -> None:
        mode = message.data.strip().lower()
        with self.lock:
            if not self.modes or self.modes[-1] != mode:
                self.modes.append(mode)

    def verify(self, wrapped_result) -> None:
        if wrapped_result is None:
            raise RuntimeError("mission returned no wrapped result")
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"mission action status is {wrapped_result.status}")
        if wrapped_result.result.status != OutdoorIndoorMission.Result.SUCCESS:
            raise RuntimeError(
                f"mission result is {wrapped_result.result.status}: "
                f"{wrapped_result.result.message}"
            )
        with self.lock:
            phases = list(self.phases)
            modes = list(self.modes)
            p2p_goal = self.p2p_goal
            counts = (
                self.route_enable_count,
                self.route_disable_count,
                self.p2p_enable_count,
                self.p2p_disable_count,
            )
        require_subsequence(
            phases,
            [
                "VALIDATING",
                "OUTDOOR_TRACKING",
                "TRANSITION_STOP",
                "INDOOR_NAVIGATION",
                "COMPLETED",
            ],
            "mission phases",
        )
        require_subsequence(
            modes, ["outdoor", "none", "indoor", "none"], "control modes"
        )
        if p2p_goal is None:
            raise RuntimeError("mock P2P server did not receive a goal")
        expected = self.return_goal().pose
        actual = p2p_goal.pose
        for axis in ("x", "y", "z"):
            difference = abs(
                getattr(actual.position, axis) - getattr(expected.position, axis)
            )
            if difference > 1.0e-6:
                raise RuntimeError(f"P2P return goal {axis} does not match route start")
        if abs(actual.orientation.z - expected.orientation.z) > 1.0e-6 or abs(
            actual.orientation.w - expected.orientation.w
        ) > 1.0e-6:
            raise RuntimeError("P2P return orientation does not match configured start yaw")
        if counts[0] != 1 or counts[1] < 1 or counts[2] != 1 or counts[3] < 2:
            raise RuntimeError(f"unexpected controller gate counts: {counts}")
        graph_topics = {name for name, _ in self.get_topic_names_and_types()}
        unexpected = sorted(FORBIDDEN_MOTION_TOPICS & graph_topics)
        if unexpected:
            raise RuntimeError(f"no-motion graph contains motion topics: {unexpected}")

        end = self.route_document["points"][-1]
        print(f"ROUTE_ID={self.route_id}")
        print(f"UNIFIED_MAP_SHA256={self.map_sha256}")
        print(
            "ROUTE_END_XYZ=%.9f %.9f %.9f"
            % (float(end["x"]), float(end["y"]), float(end["z"]))
        )
        print(
            "P2P_RETURN_XYZ=%.9f %.9f %.9f"
            % (self.return_goal_x, self.return_goal_y, self.return_goal_z)
        )
        print("PHASES=" + ",".join(phases))
        print("CONTROL_MODES=" + ",".join(modes))
        print("ZERO_ODOMETRY_ONLY=PASS")
        print("FORBIDDEN_MOTION_TOPICS_ABSENT=PASS")
        print("RESULT=MY_ROUTE_A_NO_MOTION_DRY_RUN_PASS")


def main() -> int:
    rclpy.init()
    plant: Optional[NoMotionPlant] = None
    mission: Optional[OutdoorIndoorMissionServer] = None
    executor: Optional[MultiThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None
    try:
        plant = NoMotionPlant()
        mission = OutdoorIndoorMissionServer(
            parameter_overrides=plant.mission_parameter_overrides()
        )
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(plant)
        executor.add_node(mission)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if not plant.mission_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("mission Action server did not become ready")
        goal = OutdoorIndoorMission.Goal()
        goal.mission_id = plant.mission_id
        goal.route_id = plant.route_id
        goal.indoor_goal = plant.return_goal()
        client_goal = wait_future(
            plant.mission_client.send_goal_async(goal),
            3.0,
            "mission goal acceptance",
        )
        if client_goal is None or not client_goal.accepted:
            raise RuntimeError("mission Action rejected the synthetic test goal")
        wrapped_result = wait_future(
            client_goal.get_result_async(),
            plant.test_timeout_sec,
            "complete outdoor-to-indoor mission",
        )
        time.sleep(0.10)
        plant.verify(wrapped_result)
        return 0
    finally:
        if mission is not None:
            mission.action_server.destroy()
        if plant is not None:
            plant.p2p_action_server.destroy()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if mission is not None:
            mission.destroy_node()
        if plant is not None:
            plant.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
