#!/usr/bin/env python3
"""Exercise initial-pose loading and two sequential P2P Action goals."""

import json
import math
import pathlib
import tempfile
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


TEMPORARY = tempfile.TemporaryDirectory()
ROOT = pathlib.Path(TEMPORARY.name)
INITIAL_PATH = ROOT / "fixed_start.json"
MISSION_PATH = ROOT / "route_a.json"


def write_documents():
    covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        covariance[index] = 0.1
    INITIAL_PATH.write_text(
        json.dumps(
            {
                "version": 1,
                "frame_id": "map",
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.32},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0,
                    },
                },
                "covariance": covariance,
            }
        ),
        encoding="utf-8",
    )
    MISSION_PATH.write_text(
        json.dumps(
            {
                "version": 1,
                "mission_id": "route_a",
                "initial_pose_file": INITIAL_PATH.name,
                "waypoints": [
                    {
                        "id": "wp_001",
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.0,
                        "yaw": 0.5,
                        "dwell_sec": 0.05,
                    },
                    {
                        "id": "wp_002",
                        "x": 2.0,
                        "y": 3.0,
                        "z": 0.1,
                        "yaw": -0.4,
                        "dwell_sec": 0.0,
                    },
                ],
            }
        ),
        encoding="utf-8",
    )


@pytest.mark.launch_test
def generate_test_description():
    write_documents()
    mission = launch_ros.actions.Node(
        package="dddmr_route_navigation",
        executable="p2p_mission_executor.py",
        name="p2p_mission_executor",
        parameters=[
            {
                "mission_file": str(MISSION_PATH),
                "allowed_root": str(ROOT),
                "arrival_stable_sec": 0.10,
                "waypoint_timeout_sec": 5.0,
                "initial_pose_timeout_sec": 5.0,
                "initial_pose_map_settle_sec": 0.10,
                "initial_pose_retry_sec": 0.30,
                "planner_ready_timeout_sec": 5.0,
                "prearm_stable_sec": 0.10,
                "input_timeout_sec": 0.40,
                "auto_arm": False,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [mission, launch_testing.actions.ReadyToTest()]
        ),
        {"mission": mission},
    )


class TestP2PMissionExecutor(unittest.TestCase):
    def test_executes_action_goals_in_order(self, proc_info, mission):
        proc_info.assertWaitForStartup(process=mission, timeout=10)
        rclpy.init()
        node = Node("p2p_mission_executor_test")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        states = []
        initial_messages = []
        goals = []
        enabled = {"value": False}

        node.create_subscription(
            String,
            "/p2p_multi_point/state",
            lambda message: states.append(message.data),
            transient_qos,
        )
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/initial_3d_pose",
            initial_messages.append,
            10,
        )
        status_pub = node.create_publisher(
            String, "/localization_status", transient_qos
        )
        health_pub = node.create_publisher(
            String, "/localization_health", transient_qos
        )
        pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, "/mcl_pose", 10
        )
        safe_pub = node.create_publisher(
            Twist, "/dddmr_go2/safe_cmd_vel", 10
        )
        map_pub = node.create_publisher(
            PointCloud2, "/map1/mapground", transient_qos
        )
        planner_pub = node.create_publisher(
            PointCloud2, "/weighted_ground", transient_qos
        )

        def enable_callback(request, response):
            enabled["value"] = bool(request.data)
            response.success = True
            response.message = "mock P2P state changed"
            return response

        node.create_service(
            SetBool, "/p2p_move_base/set_enabled", enable_callback
        )

        def goal_callback(_request):
            return (
                GoalResponse.ACCEPT
                if enabled["value"]
                else GoalResponse.REJECT
            )

        def cancel_callback(_goal_handle):
            return CancelResponse.ACCEPT

        def execute_callback(goal_handle):
            goals.append(goal_handle.request.target_pose)
            feedback = PToPMoveBase.Feedback()
            feedback.current_decision = "d_align_goal_heading"
            goal_handle.publish_feedback(feedback)
            result = PToPMoveBase.Result()
            result.status = 1
            result.result = "mock waypoint reached"
            goal_handle.succeed()
            return result

        action_server = ActionServer(
            node,
            PToPMoveBase,
            "/p2p_move_base",
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=cancel_callback,
        )
        arm_client = node.create_client(Trigger, "/p2p_multi_point/arm")
        spin_thread.start()

        def publish_localization(status, x, y, z, yaw):
            status_message = String()
            status_message.data = status
            status_pub.publish(status_message)
            health_message = String()
            health_message.data = "HEALTHY"
            health_pub.publish(health_message)
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            pose.pose.pose.position.z = z
            pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
            pose_pub.publish(pose)
            safe_pub.publish(Twist())

        try:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not initial_messages:
                ground = PointCloud2()
                ground.header.frame_id = "map"
                ground.width = 1
                ground.height = 1
                map_pub.publish(ground)
                time.sleep(0.03)
            self.assertTrue(initial_messages)

            deadline = time.monotonic() + 0.20
            while time.monotonic() < deadline:
                publish_localization("LOCALIZING", 0.0, 0.0, 0.32, 0.0)
                time.sleep(0.02)
            planner = PointCloud2()
            planner.header.frame_id = "map"
            planner.width = 1
            planner.height = 1
            planner_pub.publish(planner)
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and (
                not states or states[-1] != "READY"
            ):
                publish_localization("TRACKING", 0.0, 0.0, 0.32, 0.0)
                planner_pub.publish(planner)
                time.sleep(0.02)
            self.assertEqual(states[-1], "READY", states)
            self.assertFalse(goals)

            self.assertTrue(arm_client.wait_for_service(timeout_sec=2.0))
            arm_future = arm_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not arm_future.done():
                publish_localization("TRACKING", 0.0, 0.0, 0.32, 0.0)
                time.sleep(0.02)
            self.assertTrue(arm_future.done())
            self.assertTrue(arm_future.result().success)

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and (
                not states or states[-1] != "COMPLETE"
            ):
                if len(goals) == 0:
                    x, y, z, yaw = 0.0, 0.0, 0.32, 0.0
                else:
                    target = goals[-1].pose
                    x = target.position.x
                    y = target.position.y
                    z = target.position.z + 0.32
                    yaw = math.atan2(
                        2.0
                        * (
                            target.orientation.w * target.orientation.z
                            + target.orientation.x * target.orientation.y
                        ),
                        1.0
                        - 2.0
                        * (
                            target.orientation.y**2
                            + target.orientation.z**2
                        ),
                    )
                publish_localization("TRACKING", x, y, z, yaw)
                time.sleep(0.02)

            self.assertEqual(states[-1], "COMPLETE", states)
            self.assertEqual(len(goals), 2)
            self.assertAlmostEqual(goals[0].pose.position.x, 1.0)
            self.assertAlmostEqual(goals[1].pose.position.x, 2.0)
            self.assertIn("ALIGNING", states)
            self.assertIn("SETTLING", states)
            self.assertIn("DWELLING", states)
        finally:
            action_server.destroy()
            executor.shutdown(timeout_sec=2.0)
            spin_thread.join(timeout=2.0)
            node.destroy_node()
            rclpy.try_shutdown()
