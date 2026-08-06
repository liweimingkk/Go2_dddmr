"""ROS integration test for preview-before-execute bridge behavior."""

import json
from threading import Thread
import time

import pytest


rclpy = pytest.importorskip("rclpy")

from dddmr_sys_core.action import GetPlan, PToPMoveBase  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Empty, String  # noqa: E402

from dddmr_web_viewer.web_navigation_bridge import WebNavigationBridge  # noqa: E402


def wait_for(predicate, timeout_sec=5.0):
    """Wait for an asynchronous ROS assertion to become true."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_successful_preview_is_required_and_may_execute_only_once():
    """Exercise the planner and navigator actions without robot command topics."""
    rclpy.init()
    fake_server_node = rclpy.create_node("dddmr_web_fake_actions")
    client_node = rclpy.create_node("dddmr_web_test_client")
    navigation_goals = []

    def plan(goal_handle):
        result = GetPlan.Result()
        result.path = Path()
        result.path.header.frame_id = "map"
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.orientation.w = 1.0
        result.path.poses = [start, goal_handle.request.goal]
        goal_handle.succeed()
        return result

    def navigate(goal_handle):
        navigation_goals.append(goal_handle.request.target_pose)
        result = PToPMoveBase.Result()
        result.status = 1
        result.result = "test success"
        goal_handle.succeed()
        return result

    planner_server = ActionServer(
        fake_server_node,
        GetPlan,
        "/get_plan",
        execute_callback=plan,
    )
    navigation_server = ActionServer(
        fake_server_node,
        PToPMoveBase,
        "/p2p_move_base",
        execute_callback=navigate,
    )
    bridge = WebNavigationBridge(
        parameter_overrides=[
            Parameter("allow_navigation_execution", value=True),
            Parameter("preview_max_age_sec", value=10.0),
        ]
    )

    statuses = []
    preview_paths = []
    client_node.create_subscription(
        String,
        "/dddmr_web/status",
        lambda message: statuses.append(json.loads(message.data)),
        10,
    )
    client_node.create_subscription(
        Path,
        "/dddmr_web/preview_path",
        preview_paths.append,
        10,
    )
    preview_pub = client_node.create_publisher(
        PoseStamped, "/dddmr_web/preview_goal", 10
    )
    execute_pub = client_node.create_publisher(
        Empty, "/dddmr_web/execute_preview", 10
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake_server_node)
    executor.add_node(bridge)
    executor.add_node(client_node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        assert wait_for(
            lambda: preview_pub.get_subscription_count() == 1
            and execute_pub.get_subscription_count() == 1
        )
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 2.0
        goal.pose.position.y = -1.0
        goal.pose.position.z = 0.3
        goal.pose.orientation.w = 1.0
        preview_pub.publish(goal)

        assert wait_for(
            lambda: any(status["event"] == "preview_ready" for status in statuses)
        )
        assert preview_paths[-1].poses[-1].pose.position.x == 2.0
        assert navigation_goals == []

        execute_pub.publish(Empty())
        assert wait_for(
            lambda: any(
                status["event"] == "execution_succeeded" for status in statuses
            )
        )
        assert len(navigation_goals) == 1
        assert navigation_goals[0].pose.position.x == 2.0

        execute_pub.publish(Empty())
        assert wait_for(
            lambda: any(
                status["event"] == "execution_blocked"
                and "already executed" in status["message"]
                for status in statuses
            )
        )
        assert len(navigation_goals) == 1
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        planner_server.destroy()
        navigation_server.destroy()
        bridge.destroy_node()
        client_node.destroy_node()
        fake_server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
