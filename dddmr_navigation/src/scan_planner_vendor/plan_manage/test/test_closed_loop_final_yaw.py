"""Verify that the closed-loop controller performs terminal yaw alignment."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        parameters=[
            {
                "time_forward": 0.1,
                "heading_error_threshold": 0.8,
                "kp_pos": 0.8,
                "kp_yaw": 1.5,
                "max_vx": 0.5,
                "max_vy": 0.0,
                "max_vyaw": 0.5,
                "finish_dist": 0.15,
                "finish_yaw_tolerance": 0.1,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()]),
        {"controller": controller},
    )


class TestClosedLoopFinalYaw(unittest.TestCase):
    @staticmethod
    def make_trajectory(node: Node) -> Bspline:
        trajectory = Bspline()
        trajectory.order = 3
        trajectory.traj_id = 42
        trajectory.start_time = node.get_clock().now().to_msg()
        trajectory.knots = [
            -0.3,
            -0.2,
            -0.1,
            0.0,
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            0.6,
        ]
        trajectory.pos_pts = [
            Point(x=1.0, y=2.0, z=0.3) for _ in range(6)
        ]
        trajectory.yaw_pts = [math.pi / 2.0]
        trajectory.yaw_dt = 0.0
        return trajectory

    @staticmethod
    def make_odom(node: Node, yaw: float) -> Odometry:
        odom = Odometry()
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 0.3
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return odom

    @staticmethod
    def spin_until(
        node: Node,
        odom_publisher,
        yaw: float,
        predicate,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            odom_publisher.publish(TestClosedLoopFinalYaw.make_odom(node, yaw))
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def test_aligns_then_reports_complete(self, proc_info, controller):
        proc_info.assertWaitForStartup(process=controller, timeout=10)

        rclpy.init()
        node = Node("closed_loop_final_yaw_test")
        odom_publisher = node.create_publisher(
            Odometry, "body_pose", qos_profile_sensor_data
        )
        trajectory_publisher = node.create_publisher(
            Bspline, "planning/bspline", 10
        )
        commands = []
        frozen_states = []
        command_subscription = node.create_subscription(
            Twist, "cmd_vel", commands.append, 20
        )
        frozen_subscription = node.create_subscription(
            Bool,
            "planning/go2_execution_frozen",
            lambda message: frozen_states.append(message.data),
            20,
        )

        try:
            connected = self.spin_until(
                node,
                odom_publisher,
                0.0,
                lambda: trajectory_publisher.get_subscription_count() > 0,
                3.0,
            )
            self.assertTrue(connected, "controller did not subscribe to the trajectory")

            trajectory_publisher.publish(self.make_trajectory(node))
            commands.clear()
            frozen_states.clear()
            aligning = self.spin_until(
                node,
                odom_publisher,
                0.0,
                lambda: (
                    bool(commands)
                    and bool(frozen_states)
                    and abs(commands[-1].linear.x) < 1e-6
                    and abs(commands[-1].linear.y) < 1e-6
                    and commands[-1].angular.z > 0.2
                    and frozen_states[-1]
                ),
                3.0,
            )
            self.assertTrue(aligning, "controller did not enter final yaw alignment")

            commands.clear()
            frozen_states.clear()
            complete = self.spin_until(
                node,
                odom_publisher,
                math.pi / 2.0,
                lambda: (
                    bool(commands)
                    and bool(frozen_states)
                    and abs(commands[-1].linear.x) < 1e-6
                    and abs(commands[-1].linear.y) < 1e-6
                    and abs(commands[-1].angular.z) < 1e-6
                    and not frozen_states[-1]
                ),
                2.0,
            )
            self.assertTrue(
                complete,
                "controller did not stop after position and yaw entered tolerance",
            )
        finally:
            node.destroy_subscription(command_subscription)
            node.destroy_subscription(frozen_subscription)
            node.destroy_node()
            rclpy.shutdown()
