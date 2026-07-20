#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from go2_nav_gate_policy import (  # noqa: E402
    RECOVERY_ROTATION_DECISION,
    localization_block_reason,
    recovery_rotation_gate,
)


class LocalizationGatePolicyTest(unittest.TestCase):
    def test_optional_localization_never_blocks(self):
        self.assertIsNone(localization_block_reason(False, None, None, 0.5))

    def test_missing_and_stale_status_block(self):
        self.assertEqual(
            localization_block_reason(True, None, None, 0.5),
            "localization_no_status",
        )
        self.assertEqual(
            localization_block_reason(True, "TRACKING", 0.6, 0.5),
            "localization_status_stale",
        )

    def test_only_fresh_tracking_allows_motion(self):
        self.assertIsNone(
            localization_block_reason(True, "TRACKING", 0.1, 0.5)
        )
        self.assertEqual(
            localization_block_reason(True, "LOCALIZING", 0.1, 0.5),
            "localization_localizing",
        )
        self.assertEqual(
            localization_block_reason(True, "LOST", 0.1, 0.5),
            "localization_lost",
        )


class RecoveryRotationGatePolicyTest(unittest.TestCase):
    def gate(self, **overrides):
        values = {
            "decision": RECOVERY_ROTATION_DECISION,
            "decision_age_sec": 0.05,
            "decision_timeout_sec": 0.30,
            "command_time_ns": 200,
            "decision_transition_time_ns": 100,
            "x": 0.0,
            "y": 0.0,
            "yaw": -0.40,
            "zero_epsilon": 0.001,
            "max_yaw": 0.25,
        }
        values.update(overrides)
        return recovery_rotation_gate(**values)

    def test_fresh_new_pure_yaw_is_allowed_and_bounded(self):
        result = self.gate()
        self.assertTrue(result.allowed)
        self.assertEqual(result.reason, "pure_yaw")
        self.assertAlmostEqual(result.yaw, -0.25)

        positive = self.gate(yaw=0.10)
        self.assertTrue(positive.allowed)
        self.assertAlmostEqual(positive.yaw, 0.10)

    def test_only_rotate_recovery_state_is_allowed(self):
        self.assertEqual(
            self.gate(decision="d_recovery_position_control_waitdone").reason,
            "wrong_state",
        )
        self.assertEqual(self.gate(decision="d_controlling").reason, "wrong_state")

    def test_old_command_is_blocked(self):
        for command_time in (None, 99, 100):
            with self.subTest(command_time=command_time):
                result = self.gate(command_time_ns=command_time)
                self.assertFalse(result.allowed)
                self.assertEqual(result.reason, "old_command")

    def test_translation_and_zero_yaw_are_blocked(self):
        for translation in ({"x": 0.01}, {"y": -0.01}):
            with self.subTest(translation=translation):
                result = self.gate(**translation)
                self.assertFalse(result.allowed)
                self.assertEqual(result.reason, "translation_not_allowed")

        result = self.gate(yaw=0.0)
        self.assertFalse(result.allowed)
        self.assertEqual(result.reason, "zero_yaw")

    def test_missing_stale_and_nonfinite_decisions_are_blocked(self):
        for age in (None, 0.31, float("nan"), -0.01):
            with self.subTest(age=age):
                result = self.gate(decision_age_sec=age)
                self.assertFalse(result.allowed)
                self.assertEqual(result.reason, "stale_decision")

    def test_nonfinite_command_and_invalid_limits_are_blocked(self):
        self.assertEqual(self.gate(yaw=float("nan")).reason, "nonfinite_command")
        self.assertEqual(self.gate(max_yaw=0.0).reason, "invalid_limits")


if __name__ == "__main__":
    unittest.main()
