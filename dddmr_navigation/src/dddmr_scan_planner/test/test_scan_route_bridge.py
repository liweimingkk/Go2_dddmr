"""Prove SCAN goals wait for the global planner readiness signal."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from dddmr_sys_core.action import GetPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    bridge = launch_ros.actions.Node(
        package="dddmr_scan_planner",
        executable="scan_route_bridge_node",
        name="scan_route_bridge",
        parameters=[
            {
                "global_plan_action": "/test_get_plan",
                "planner_ready_topic": "/test_weighted_ground",
                "map_frame": "map",
                "body_pose_timeout": 0.50,
                "min_path_poses": 2,
                "start_exclusion_xy": 0.15,
                "min_path_point_separation": 0.02,
            }
        ],
        remappings=[
            ("goal_pose_3d", "/test_scan_goal"),
            ("body_pose", "/test_scan_body"),
            ("initial_path", "/test_scan_path"),
            ("route_ready", "/test_scan_route_ready"),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([bridge, launch_testing.actions.ReadyToTest()]),
        {"bridge": bridge},
    )


class TestScanRouteBridge(unittest.TestCase):
    def test_holds_goal_until_planner_weighted_ground(self, proc_info, bridge):
        proc_info.assertWaitForStartup(process=bridge, timeout=10)
        rclpy.init()
        node = Node("scan_route_bridge_test")
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        requests = []
        paths = []
        route_states = []

        def execute_plan(goal_handle):
            requests.append(goal_handle.request)
            result = GetPlan.Result()
            result.path.header.frame_id = "map"
            for x in (0.0, 0.5, 1.0):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.orientation.w = 1.0
                result.path.poses.append(pose)
            goal_handle.succeed()
            return result

        action_server = ActionServer(
            node, GetPlan, "/test_get_plan", execute_callback=execute_plan
        )
        body_pub = node.create_publisher(
            Odometry, "/test_scan_body", qos_profile_sensor_data
        )
        goal_pub = node.create_publisher(PoseStamped, "/test_scan_goal", 10)
        planner_pub = node.create_publisher(
            PointCloud2, "/test_weighted_ground", transient_qos
        )
        path_sub = node.create_subscription(
            Path, "/test_scan_path", paths.append, transient_qos
        )
        route_sub = node.create_subscription(
            Bool,
            "/test_scan_route_ready",
            lambda message: route_states.append(message.data),
            transient_qos,
        )

        def publish_body():
            body = Odometry()
            body.header.stamp = node.get_clock().now().to_msg()
            body.header.frame_id = "map"
            body.child_frame_id = "base_link"
            body.pose.pose.orientation.w = 1.0
            body_pub.publish(body)

        try:
            discovery_deadline = time.monotonic() + 0.50
            while time.monotonic() < discovery_deadline:
                publish_body()
                rclpy.spin_once(node, timeout_sec=0.02)

            goal = PoseStamped()
            goal.header.stamp = node.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = 1.0
            goal.pose.orientation.w = 1.0
            goal_pub.publish(goal)

            hold_deadline = time.monotonic() + 0.60
            while time.monotonic() < hold_deadline:
                publish_body()
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertFalse(
                requests,
                "route request escaped before planner weighted ground",
            )
            self.assertFalse(paths)

            weighted_ground = PointCloud2()
            weighted_ground.header.frame_id = "map"
            weighted_ground.width = 1
            weighted_ground.height = 1
            planner_pub.publish(weighted_ground)

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not paths:
                publish_body()
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(len(requests), 1)
            self.assertTrue(paths, "queued goal was not planned after readiness")
            self.assertEqual(len(paths[-1].poses), 2)
            self.assertAlmostEqual(paths[-1].poses[-1].pose.position.x, 1.0)
            ready_deadline = time.monotonic() + 1.0
            while (
                time.monotonic() < ready_deadline
                and (not route_states or not route_states[-1])
            ):
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(route_states)
            self.assertTrue(route_states[-1])
        finally:
            node.destroy_subscription(path_sub)
            node.destroy_subscription(route_sub)
            action_server.destroy()
            node.destroy_node()
            rclpy.shutdown()
