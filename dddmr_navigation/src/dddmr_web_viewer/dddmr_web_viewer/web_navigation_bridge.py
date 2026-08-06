"""Bridge browser requests to DDDMR using preview-before-execute semantics."""

import copy
import json
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from dddmr_sys_core.action import GetPlan, PToPMoveBase
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, String

from .policy import PreviewAuthorization, position_is_finite, quaternion_is_valid


class WebNavigationBridge(Node):
    """Relay visualization data and guard browser-triggered navigation actions."""

    def __init__(self, parameter_overrides=None) -> None:
        """Configure relays, guarded commands, and DDDMR action clients."""
        super().__init__(
            "dddmr_web_navigation_bridge",
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter("map_topic", "/map1/mapcloud")
        self.declare_parameter("ground_topic", "/map1/mapground")
        self.declare_parameter("preview_action_name", "/get_plan")
        self.declare_parameter("navigation_action_name", "/p2p_move_base")
        self.declare_parameter("initial_pose_topic", "/initial_3d_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("allow_navigation_execution", False)
        self.declare_parameter("preview_max_age_sec", 30.0)

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._ground_topic = str(self.get_parameter("ground_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._allow_execution = bool(
            self.get_parameter("allow_navigation_execution").value
        )
        preview_max_age_sec = float(
            self.get_parameter("preview_max_age_sec").value
        )
        if preview_max_age_sec <= 0.0:
            raise ValueError("preview_max_age_sec must be positive")

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = QoSReliabilityPolicy.RELIABLE
        transient_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._status_pub = self.create_publisher(
            String, "/dddmr_web/status", transient_qos
        )
        self._map_pub = self.create_publisher(
            PointCloud2, "/dddmr_web/mapcloud", 2
        )
        self._ground_pub = self.create_publisher(
            PointCloud2, "/dddmr_web/mapground", 2
        )
        self._preview_path_pub = self.create_publisher(
            Path, "/dddmr_web/preview_path", 2
        )
        initial_pose_topic = str(
            self.get_parameter("initial_pose_topic").value
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, initial_pose_topic, 2
        )

        self._map_sub = self.create_subscription(
            PointCloud2,
            self._map_topic,
            self._on_map,
            transient_qos,
        )
        self._ground_sub = self.create_subscription(
            PointCloud2,
            self._ground_topic,
            self._on_ground,
            transient_qos,
        )
        self._preview_goal_sub = self.create_subscription(
            PoseStamped,
            "/dddmr_web/preview_goal",
            self._on_preview_goal,
            2,
        )
        self._execute_sub = self.create_subscription(
            Empty,
            "/dddmr_web/execute_preview",
            self._on_execute_preview,
            2,
        )
        self._cancel_sub = self.create_subscription(
            Empty,
            "/dddmr_web/cancel_navigation",
            self._on_cancel_navigation,
            2,
        )
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/dddmr_web/initial_pose",
            self._on_initial_pose,
            2,
        )
        self._request_status_sub = self.create_subscription(
            Empty,
            "/dddmr_web/request_status",
            self._on_status_request,
            2,
        )

        preview_action_name = str(
            self.get_parameter("preview_action_name").value
        )
        navigation_action_name = str(
            self.get_parameter("navigation_action_name").value
        )
        self._preview_client = ActionClient(
            self, GetPlan, preview_action_name
        )
        self._navigation_client = ActionClient(
            self, PToPMoveBase, navigation_action_name
        )

        self._cached_map: Optional[PointCloud2] = None
        self._cached_ground: Optional[PointCloud2] = None
        self._requested_preview_goal: Optional[PoseStamped] = None
        self._pending_preview_goal: Optional[PoseStamped] = None
        self._preview_generation = 0
        self._preview_authorization = PreviewAuthorization(
            max_age_sec=preview_max_age_sec
        )
        self._execution_send_pending = False
        self._active_navigation_goal = None
        self._cancel_when_accepted = False

        self._publish_status(
            "bridge_ready",
            True,
            "Web bridge is ready; navigation execution is disabled by default."
            if not self._allow_execution
            else "Web bridge is ready; guarded navigation execution is enabled.",
        )
        execution_state = "ENABLED" if self._allow_execution else "disabled"
        self.get_logger().warn(f"Web navigation execution is {execution_state}")

    def _publish_status(
        self,
        event: str,
        ok: bool,
        message: str,
        **details: Any,
    ) -> None:
        payload: Dict[str, Any] = {
            "event": event,
            "ok": ok,
            "message": message,
            "execution_allowed": self._allow_execution,
        }
        payload.update(details)
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(output)

    def _on_map(self, message: PointCloud2) -> None:
        self._cached_map = message
        self._map_pub.publish(message)

    def _on_ground(self, message: PointCloud2) -> None:
        self._cached_ground = message
        self._ground_pub.publish(message)

    def _on_status_request(self, _message: Empty) -> None:
        self._publish_status(
            "bridge_ready",
            True,
            "Bridge status and cached visualization layers refreshed.",
            map_ready=self._cached_map is not None,
            ground_ready=self._cached_ground is not None,
        )
        if self._cached_map is not None:
            self._map_pub.publish(self._cached_map)
        if self._cached_ground is not None:
            self._ground_pub.publish(self._cached_ground)

    def _pose_is_valid(self, message: PoseStamped) -> bool:
        position = message.pose.position
        orientation = message.pose.orientation
        return (
            message.header.frame_id == self._map_frame
            and position_is_finite(position.x, position.y, position.z)
            and quaternion_is_valid(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )

    def _on_preview_goal(self, message: PoseStamped) -> None:
        self._preview_generation += 1
        generation = self._preview_generation
        self._requested_preview_goal = None
        self._pending_preview_goal = None
        self._preview_authorization.clear()
        self._preview_path_pub.publish(Path())

        if not self._pose_is_valid(message):
            self._publish_status(
                "preview_rejected",
                False,
                "Preview goal must be finite, use frame 'map', and contain a unit quaternion.",
            )
            return
        self._requested_preview_goal = copy.deepcopy(message)
        if not self._preview_client.server_is_ready():
            self._publish_status(
                "preview_unavailable",
                False,
                "Global planner action is not ready.",
            )
            return

        request = GetPlan.Goal()
        request.goal = copy.deepcopy(message)
        request.activate_threading = True
        request.project_goal_to_ground = True
        future = self._preview_client.send_goal_async(request)
        future.add_done_callback(
            lambda completed, current=generation: self._on_preview_goal_response(
                completed, current
            )
        )
        self._publish_status(
            "preview_planning",
            True,
            "Planning a read-only route preview.",
        )

    def _on_preview_goal_response(self, future: Any, generation: int) -> None:
        if generation != self._preview_generation:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - middleware exception type varies
            self._publish_status("preview_failed", False, str(error))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status(
                "preview_rejected", False, "Global planner rejected the preview goal."
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, current=generation: self._on_preview_result(
                completed, current
            )
        )

    def _on_preview_result(self, future: Any, generation: int) -> None:
        if generation != self._preview_generation:
            return
        try:
            wrapped_result = future.result()
            path = wrapped_result.result.path
        except Exception as error:  # pragma: no cover - middleware exception type varies
            self._publish_status("preview_failed", False, str(error))
            return
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or len(path.poses) < 2
        ):
            self._publish_status(
                "preview_failed", False, "No traversable path was found for this goal."
            )
            return

        if self._requested_preview_goal is None:
            self._publish_status(
                "preview_failed", False, "The requested preview goal is no longer available."
            )
            return
        self._pending_preview_goal = copy.deepcopy(self._requested_preview_goal)
        self._preview_authorization.record(self.get_clock().now().nanoseconds)
        self._preview_path_pub.publish(path)
        self._publish_status(
            "preview_ready",
            True,
            "Route preview is ready. Execution still requires explicit confirmation.",
            path_points=len(path.poses),
        )

    def _on_execute_preview(self, _message: Empty) -> None:
        if not self._allow_execution:
            self._publish_status(
                "execution_blocked",
                False,
                "Navigation execution is disabled by the launch safety gate.",
            )
            return
        if self._execution_send_pending or self._active_navigation_goal is not None:
            self._publish_status(
                "execution_blocked",
                False,
                "A web-started navigation goal is already active.",
            )
            return
        authorized, reason = self._preview_authorization.authorize(
            self.get_clock().now().nanoseconds
        )
        if not authorized or self._pending_preview_goal is None:
            self._publish_status("execution_blocked", False, reason)
            return
        if not self._navigation_client.server_is_ready():
            self._publish_status(
                "execution_unavailable",
                False,
                "P2P navigation action is not ready; preview must be planned again.",
            )
            return

        request = PToPMoveBase.Goal()
        request.target_pose = copy.deepcopy(self._pending_preview_goal)
        request.project_goal_to_ground = True
        self._execution_send_pending = True
        self._cancel_when_accepted = False
        future = self._navigation_client.send_goal_async(request)
        future.add_done_callback(self._on_navigation_goal_response)
        self._publish_status(
            "execution_requested",
            True,
            "Confirmed navigation goal sent to the P2P action server.",
        )

    def _on_navigation_goal_response(self, future: Any) -> None:
        self._execution_send_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - middleware exception type varies
            self._publish_status("execution_failed", False, str(error))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status(
                "execution_rejected", False, "P2P navigation rejected the goal."
            )
            return

        self._active_navigation_goal = goal_handle
        self._publish_status(
            "execution_accepted", True, "P2P navigation accepted the confirmed goal."
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)
        if self._cancel_when_accepted:
            self._cancel_active_goal()

    def _on_navigation_result(self, future: Any) -> None:
        self._active_navigation_goal = None
        try:
            wrapped_result = future.result()
        except Exception as error:  # pragma: no cover - middleware exception type varies
            self._publish_status("execution_failed", False, str(error))
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(
                "execution_succeeded", True, "Navigation goal completed."
            )
        elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
            self._publish_status(
                "execution_canceled", True, "Navigation goal was canceled."
            )
        else:
            self._publish_status(
                "execution_failed",
                False,
                "Navigation ended without success.",
                action_status=int(wrapped_result.status),
            )

    def _on_cancel_navigation(self, _message: Empty) -> None:
        if self._execution_send_pending:
            self._cancel_when_accepted = True
            self._publish_status(
                "cancel_pending",
                True,
                "Cancellation will be sent as soon as the goal is accepted.",
            )
            return
        if self._active_navigation_goal is None:
            self._publish_status(
                "cancel_ignored", False, "No web-started navigation goal is active."
            )
            return
        self._cancel_active_goal()

    def _cancel_active_goal(self) -> None:
        if self._active_navigation_goal is None:
            return
        future = self._active_navigation_goal.cancel_goal_async()
        future.add_done_callback(self._on_cancel_response)
        self._publish_status(
            "cancel_requested", True, "Cancellation requested for the web-started goal."
        )

    def _on_cancel_response(self, future: Any) -> None:
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as error:  # pragma: no cover - middleware exception type varies
            self._publish_status("cancel_failed", False, str(error))
            return
        self._publish_status(
            "cancel_accepted" if accepted else "cancel_rejected",
            accepted,
            "Navigation cancellation was accepted."
            if accepted
            else "Navigation cancellation was rejected.",
        )

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        if not self._pose_is_valid(pose):
            self._publish_status(
                "initial_pose_rejected",
                False,
                "Initial pose must be finite, use frame 'map', and contain a unit quaternion.",
            )
            return
        output = copy.deepcopy(message)
        output.header.stamp = self.get_clock().now().to_msg()
        self._initial_pose_pub.publish(output)
        self._publish_status(
            "initial_pose_published",
            True,
            "Initial 3D pose was forwarded to the localizer.",
        )


def main(args: Optional[list] = None) -> None:
    """Run the guarded web navigation bridge."""
    rclpy.init(args=args)
    node = WebNavigationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
