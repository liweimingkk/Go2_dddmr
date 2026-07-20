#!/usr/bin/env python3

import argparse
import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("go2_wasd_teleop.py")
SPEC = importlib.util.spec_from_file_location("go2_wasd_teleop", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Go2WasdTeleopTest(unittest.TestCase):
    def test_wasd_mapping_uses_standard_sport_axes(self):
        expected = {
            "w": (0.10, 0.0, 0.0),
            "s": (-0.10, 0.0, 0.0),
            "a": (0.0, 0.0, 0.25),
            "d": (0.0, 0.0, -0.25),
        }
        for key, values in expected.items():
            with self.subTest(key=key):
                action = MODULE.action_for_key(key, 0.10, 0.25)
                self.assertIsNotNone(action)
                self.assertEqual(action.kind, "move")
                self.assertEqual(
                    (action.command.x, action.command.y, action.command.yaw),
                    values,
                )

    def test_stop_and_quit_keys(self):
        for key in (" ", "x", "X"):
            self.assertEqual(
                MODULE.action_for_key(key, 0.10, 0.25).kind,
                "stop",
            )
        for key in ("q", "Q", "\x03", "\x1b"):
            self.assertEqual(
                MODULE.action_for_key(key, 0.10, 0.25).kind,
                "quit",
            )

    def test_motion_command_uses_unitree_move_payload(self):
        command = MODULE.MotionCommand(0.1, 0.0, -0.25)
        self.assertEqual(command.parameter(), '{"x":0.1,"y":0.0,"z":-0.25}')

    def test_active_motion_expires(self):
        active = MODULE.ActiveMotion()
        command = MODULE.MotionCommand(0.1, 0.0, 0.0)
        active.activate(command, now=10.0, timeout=0.35)
        self.assertEqual(active.current(10.349), command)
        self.assertIsNone(active.current(10.35))
        self.assertIsNone(active.current(11.0))

    def test_preview_does_not_require_live_confirmation(self):
        args = MODULE.build_parser().parse_args([])
        MODULE.validate_options(args, {})
        self.assertFalse(args.live)

    def test_live_requires_confirmation_and_cyclonedds(self):
        args = MODULE.build_parser().parse_args(["--live"])
        with self.assertRaisesRegex(ValueError, MODULE.LIVE_CONFIRM_ENV):
            MODULE.validate_options(args, {})
        with self.assertRaisesRegex(ValueError, "rmw_cyclonedds_cpp"):
            MODULE.validate_options(
                args,
                {MODULE.LIVE_CONFIRM_ENV: MODULE.LIVE_CONFIRM_PHRASE},
            )
        MODULE.validate_options(
            args,
            {
                MODULE.LIVE_CONFIRM_ENV: MODULE.LIVE_CONFIRM_PHRASE,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            },
        )

    def test_hard_speed_caps(self):
        defaults = vars(MODULE.build_parser().parse_args([]))
        too_fast = argparse.Namespace(**defaults)
        too_fast.linear_speed = MODULE.MAX_LINEAR_SPEED + 0.01
        with self.assertRaisesRegex(ValueError, "linear speed"):
            MODULE.validate_options(too_fast, {})


if __name__ == "__main__":
    unittest.main()
