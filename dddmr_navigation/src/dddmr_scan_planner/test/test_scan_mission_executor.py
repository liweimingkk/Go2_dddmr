"""Exercise initial-pose loading and a sequential two-point SCAN mission."""

import json
import math
import pathlib
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


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
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
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
    executor = launch_ros.actions.Node(
        package="dddmr_scan_planner",
        executable="scan_mission_executor.py",
        name="scan_mission_executor",
        parameters=[
            {
                "mission_file": str(MISSION_PATH),
                "allowed_root": str(ROOT),
                "arrival_stable_sec": 0.10,
                "planning_timeout_sec": 2.0,
                "waypoint_timeout_sec": 5.0,
                "initial_pose_timeout_sec": 5.0,
                "initial_pose_map_settle_sec": 0.10,
                "initial_pose_retry_sec": 0.30,
                "initial_pose_xy_tolerance": 0.50,
                "initial_pose_z_tolerance": 0.25,
                "initial_pose_yaw_tolerance": 0.35,
                "input_timeout_sec": 1.0,
                "guard_failure_grace_sec": 0.20,
                "auto_arm": True,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([executor, launch_testing.actions.ReadyToTest()]),
        {"executor": executor},
    )


class TestScanMissionExecutor(unittest.TestCase):
    def test_loads_pose_then_completes_waypoints_in_order(
        self, proc_info, executor
    ):
        proc_info.assertWaitForStartup(process=executor, timeout=10)
        rclpy.init()
        node = Node("scan_mission_executor_test")
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        initial_messages = []
        goals = []
        states = []
        enabled_states = []
        initial_subscription = node.create_subscription(
            PoseWithCovarianceStamped,
            "/initial_3d_pose",
            initial_messages.append,
            10,
        )
        goal_subscription = node.create_subscription(
            PoseStamped, "/goal_pose_3d", goals.append, 10
        )
        state_subscription = node.create_subscription(
            String, "/scan_multi_point/state", lambda message: states.append(message.data),
            transient_qos,
        )
        enabled_subscription = node.create_subscription(
            Bool,
            "/scan_multi_point/enabled",
            lambda message: enabled_states.append(message.data),
            transient_qos,
        )
        map_pub = node.create_publisher(
            PointCloud2, "/map1/mapground", transient_qos
        )
        status_pub = node.create_publisher(
            String, "/localization_status", transient_qos
        )
        health_pub = node.create_publisher(
            String, "/localization_health", transient_qos
        )
        body_pub = node.create_publisher(
            Odometry, "/scan_planner/body_pose", qos_profile_sensor_data
        )
        mcl_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, "/mcl_pose", 10
        )
        command_pub = node.create_publisher(
            Twist, "/scan_planner/raw_cmd_vel", 10
        )
        route_pub = node.create_publisher(
            Bool, "/scan_planner/route_ready", transient_qos
        )
        guard_pub = node.create_publisher(
            String, "/scan_planner/command_guard_status", 10
        )

        def publish_localization(
            status, x=1.0, y=2.0, z=0.32, yaw=0.5
        ):
            status_message = String()
            status_message.data = status
            status_pub.publish(status_message)
            health_message = String()
            health_message.data = "HEALTHY"
            health_pub.publish(health_message)
            body = Odometry()
            body.header.stamp = node.get_clock().now().to_msg()
            body.header.frame_id = "map"
            body.child_frame_id = "base_link"
            body.pose.pose.position.x = x
            body.pose.pose.position.y = y
            body.pose.pose.position.z = z
            body.pose.pose.orientation.z = math.sin(yaw / 2.0)
            body.pose.pose.orientation.w = math.cos(yaw / 2.0)
            body_pub.publish(body)
            mcl_pose = PoseWithCovarianceStamped()
            mcl_pose.header.stamp = node.get_clock().now().to_msg()
            mcl_pose.header.frame_id = "map"
            mcl_pose.pose.pose = body.pose.pose
            mcl_pose_pub.publish(mcl_pose)

        def publish_route_cycle():
            not_ready = Bool()
            not_ready.data = False
            route_pub.publish(not_ready)
            for _ in range(5):
                publish_localization("TRACKING")
                rclpy.spin_once(node, timeout_sec=0.02)
            ready = Bool()
            ready.data = True
            route_pub.publish(ready)

        def publish_arrival(x, y, z, yaw):
            publish_localization("TRACKING", x, y, z, yaw)
            command_pub.publish(Twist())
            guard = String()
            guard.data = "ok"
            guard_pub.publish(guard)
            rclpy.spin_once(node, timeout_sec=0.02)

        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not initial_messages:
                ground = PointCloud2()
                ground.header.frame_id = "map"
                ground.width = 1
                ground.height = 1
                map_pub.publish(ground)
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(initial_messages, "executor did not publish fixed initial pose")

            # A healthy but unrelated auto-global result must not arm the
            # mission. With no matching /mcl_pose, the fixed seed is retried.
            transition_deadline = time.monotonic() + 0.15
            while time.monotonic() < transition_deadline:
                publish_localization(
                    "LOCALIZING", 12.92, 7.10, 0.21, 1.20
                )
                rclpy.spin_once(node, timeout_sec=0.02)
            wrong_pose_deadline = time.monotonic() + 0.70
            while time.monotonic() < wrong_pose_deadline:
                publish_localization("TRACKING", 12.92, 7.10, 0.21, 1.20)
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertFalse(
                goals,
                f"wrong global-localization pose armed mission; states={states}",
            )
            self.assertGreaterEqual(
                len(initial_messages),
                2,
                "executor did not retry an unconfirmed fixed initial pose",
            )

            # Prove the saved pose is observed after a post-seed transition.
            transition_deadline = time.monotonic() + 0.20
            while time.monotonic() < transition_deadline:
                publish_localization("LOCALIZING", 0.0, 0.0, 0.32, 0.0)
                rclpy.spin_once(node, timeout_sec=0.02)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not goals:
                publish_localization("TRACKING", 0.0, 0.0, 0.32, 0.0)
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(goals, f"executor did not submit waypoint; states={states}")
            self.assertAlmostEqual(goals[-1].pose.position.x, 1.0)
            self.assertAlmostEqual(goals[-1].pose.position.y, 2.0)
            self.assertAlmostEqual(
                goals[-1].pose.orientation.z, math.sin(0.25), places=6
            )

            publish_route_cycle()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and len(goals) < 2:
                publish_arrival(1.0, 2.0, 0.32, 0.5)
            self.assertGreaterEqual(
                len(goals), 2, f"executor did not submit waypoint 2; states={states}"
            )
            self.assertAlmostEqual(goals[-1].pose.position.x, 2.0)
            self.assertAlmostEqual(goals[-1].pose.position.y, 3.0)
            self.assertAlmostEqual(goals[-1].pose.position.z, 0.1)
            self.assertAlmostEqual(
                goals[-1].pose.orientation.z, math.sin(-0.2), places=6
            )

            publish_route_cycle()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and "COMPLETE" not in states:
                publish_arrival(2.0, 3.0, 0.42, -0.4)
            self.assertIn("COMPLETE", states)
            disabled_deadline = time.monotonic() + 1.0
            while (
                time.monotonic() < disabled_deadline
                and (not enabled_states or enabled_states[-1])
            ):
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(enabled_states)
            self.assertFalse(enabled_states[-1])
        finally:
            for subscription in (
                initial_subscription,
                goal_subscription,
                state_subscription,
                enabled_subscription,
            ):
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
