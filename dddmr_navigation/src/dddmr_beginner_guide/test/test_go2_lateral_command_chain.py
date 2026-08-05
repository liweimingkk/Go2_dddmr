#!/usr/bin/env python3

import pathlib
import sys
import unittest

from geometry_msgs.msg import Twist


SCRIPTS_DIRECTORY = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

from go2_nav_cmd_gate import Go2NavCmdGate  # noqa: E402
from go2_sport_cmd_vel_adapter import Go2SportCmdVelAdapter  # noqa: E402
from go2_sport_cmd_vel_dry_run import Go2SportCmdVelDryRun  # noqa: E402


def command(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Twist:
    message = Twist()
    message.linear.x = x
    message.linear.y = y
    message.angular.z = yaw
    return message


def configured_gate(max_y: float) -> Go2NavCmdGate:
    gate = object.__new__(Go2NavCmdGate)
    gate.max_x = 0.50
    gate.max_y = max_y
    gate.max_yaw = 0.50
    gate.zero_epsilon = 0.001
    return gate


def configured_adapter(adapter_type, max_y: float):
    adapter = object.__new__(adapter_type)
    adapter.axis_mode = "standard"
    adapter.x_sign = 1.0
    adapter.y_sign = 1.0
    adapter.yaw_sign = 1.0
    adapter.max_x = 0.50
    adapter.max_y = max_y
    adapter.max_yaw = 0.50
    adapter.linear_deadband = 0.01
    adapter.angular_deadband = 0.02
    return adapter


class Go2LateralCommandChainTest(unittest.TestCase):
    def test_positive_and_negative_y_keep_their_sign_through_all_clamps(self):
        gate = configured_gate(0.20)
        adapters = (
            configured_adapter(Go2SportCmdVelDryRun, 0.20),
            configured_adapter(Go2SportCmdVelAdapter, 0.20),
        )

        for requested, expected in ((0.30, 0.20), (-0.30, -0.20)):
            with self.subTest(requested=requested):
                gated = gate.clamp_twist(command(y=requested))
                self.assertAlmostEqual(gated.linear.y, expected)
                for adapter in adapters:
                    mapped = adapter.map_axis_and_clamp(gated)
                    self.assertEqual(mapped, (0.0, expected, 0.0))

    def test_standard_axis_mode_does_not_swap_forward_and_lateral(self):
        adapters = (
            configured_adapter(Go2SportCmdVelDryRun, 0.20),
            configured_adapter(Go2SportCmdVelAdapter, 0.20),
        )

        for adapter in adapters:
            with self.subTest(adapter=type(adapter).__name__):
                self.assertEqual(
                    adapter.map_axis_and_clamp(command(x=0.15, y=-0.10)),
                    (0.15, -0.10, 0.0),
                )

    def test_zero_limit_and_nonfinite_input_fail_closed(self):
        locked_gate = configured_gate(0.0)
        locked_adapters = (
            configured_adapter(Go2SportCmdVelDryRun, 0.0),
            configured_adapter(Go2SportCmdVelAdapter, 0.0),
        )

        locked = locked_gate.clamp_twist(command(y=0.20))
        self.assertEqual(locked.linear.y, 0.0)
        for adapter in locked_adapters:
            self.assertEqual(
                adapter.map_axis_and_clamp(command(y=0.20))[1],
                0.0,
            )

        open_gate = configured_gate(0.20)
        invalid = open_gate.clamp_twist(command(y=float("nan")))
        self.assertEqual(invalid.linear.y, 0.0)
        for adapter_type in (Go2SportCmdVelDryRun, Go2SportCmdVelAdapter):
            adapter = configured_adapter(adapter_type, 0.20)
            self.assertEqual(
                adapter.map_axis_and_clamp(command(y=float("nan")))[1],
                0.0,
            )


if __name__ == "__main__":
    unittest.main()
