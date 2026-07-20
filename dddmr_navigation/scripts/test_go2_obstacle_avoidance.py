#!/usr/bin/env python3
"""No-ROS unit tests for go2_obstacle_avoidance.py."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import math
import pathlib
import sys
import unittest


SCRIPT_PATH = pathlib.Path(__file__).with_name("go2_obstacle_avoidance.py")
SPEC = importlib.util.spec_from_file_location("go2_obstacle_avoidance", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class PayloadParsingTest(unittest.TestCase):
    def test_parse_enable_payload(self) -> None:
        self.assertFalse(MODULE.parse_enable_payload('{"enable":false}'))
        self.assertTrue(MODULE.parse_enable_payload('{"enable":true}'))

    def test_parse_enable_payload_rejects_non_boolean(self) -> None:
        with self.assertRaises(ValueError):
            MODULE.parse_enable_payload('{"enable":0}')
        with self.assertRaises(ValueError):
            MODULE.parse_enable_payload("not-json")

    def test_parse_multistate_payload(self) -> None:
        payload = '{"volume":8,"obstaclesAvoidSwitch":false}'
        self.assertFalse(MODULE.parse_multistate_payload(payload))


class StationaryEvaluationTest(unittest.TestCase):
    def test_stationary_samples_pass(self) -> None:
        samples = [
            MODULE.MotionSample(0.0, 0.0, 0.01),
            MODULE.MotionSample(0.01, -0.01, -0.02),
            MODULE.MotionSample(-0.01, 0.0, 0.0),
        ]
        self.assertTrue(MODULE.evaluate_stationary(samples).stationary)

    def test_sustained_slow_motion_fails(self) -> None:
        samples = [MODULE.MotionSample(0.05, 0.0, 0.0) for _ in range(5)]
        self.assertFalse(MODULE.evaluate_stationary(samples).stationary)

    def test_large_outlier_fails(self) -> None:
        samples = [MODULE.MotionSample(0.0, 0.0, 0.0) for _ in range(4)]
        samples.append(MODULE.MotionSample(0.2, 0.0, 0.0))
        self.assertFalse(MODULE.evaluate_stationary(samples).stationary)

    def test_invalid_samples_fail_closed(self) -> None:
        with self.assertRaises(ValueError):
            MODULE.evaluate_stationary([])
        with self.assertRaises(ValueError):
            MODULE.evaluate_stationary(
                [MODULE.MotionSample(math.nan, 0.0, 0.0)]
            )


class ArgumentTest(unittest.TestCase):
    def test_default_is_status_only(self) -> None:
        args = MODULE.build_parser().parse_args([])
        self.assertFalse(args.disable)
        self.assertFalse(args.enable)

    def test_actions_are_mutually_exclusive(self) -> None:
        parser = MODULE.build_parser()
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(["--disable", "--enable"])

    def test_option_bounds(self) -> None:
        args = MODULE.build_parser().parse_args(["--retries", "0"])
        with self.assertRaises(ValueError):
            MODULE.validate_options(args)


if __name__ == "__main__":
    unittest.main()
