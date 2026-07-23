"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from scan_planner_msgs.msg import DataDisp


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestPlannerStartup(unittest.TestCase):
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)

    def test_heartbeat_continues_while_fsm_is_idle(self):
        rclpy.init()
        node = Node("scan_planner_heartbeat_test")
        heartbeats = []
        subscription = node.create_subscription(
            DataDisp,
            "planning/data_display",
            lambda message: heartbeats.append(message.header.stamp),
            10,
        )
        try:
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and len(heartbeats) < 5:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreaterEqual(len(heartbeats), 5)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
