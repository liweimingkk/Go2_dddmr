#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from go2_nav_gate_policy import (  # noqa: E402
    RECOVERY_ROTATION_DECISION,
    STAIR_STATE_ALIGN,
    STAIR_STATE_APPROACH,
    STAIR_STATE_COMMITTED,
    STAIR_STATE_FAULT_LATCH,
    TERRAIN_DROP,
    TERRAIN_FLAT,
    TERRAIN_RAMP,
    MotionProgressMonitor,
    TerrainFaultLatch,
    TerrainGateLimits,
    TerrainGateStatus,
    localization_block_reason,
    recovery_rotation_gate,
    terrain_motion_gate,
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


class TerrainMotionGatePolicyTest(unittest.TestCase):
    def limits(self):
        return TerrainGateLimits(
            timeout_sec=0.30,
            min_confidence=0.90,
            min_support_ratio=0.80,
            max_heading_error_rad=0.14,
            max_lateral_error_m=0.10,
            align_max_forward_mps=0.0,
            align_max_yaw_rps=0.25,
            committed_max_yaw_rps=0.0,
            zero_epsilon=0.001,
            committed_max_forward_mps=0.25,
            ramp_max_up_slope_rad=0.20,
            ramp_max_down_slope_rad=0.15,
            ramp_max_cross_slope_rad=0.08,
            ramp_up_max_x_mps=0.25,
            ramp_down_max_x_mps=0.20,
            ramp_max_yaw_rps=0.10,
            max_body_roll_rad=0.20,
            max_body_pitch_rad=0.30,
            output_max_x_mps=0.50,
            output_max_y_mps=0.30,
            output_max_yaw_rps=0.50,
        )

    def status(self, **overrides):
        values = {
            "terrain_class": TERRAIN_FLAT,
            "traversal_state": 0,
            "allow_forward": True,
            "allow_reverse": True,
            "drop_detected": False,
            "dynamic_obstacle": False,
            "gait_unchanged": True,
            "confidence": 0.95,
            "support_ratio": 0.90,
            "heading_error_rad": 0.02,
            "lateral_error_m": 0.01,
            "longitudinal_slope_rad": 0.0,
            "cross_slope_rad": 0.0,
            "body_roll_rad": 0.0,
            "body_pitch_rad": 0.0,
        }
        values.update(overrides)
        return TerrainGateStatus(**values)

    def gate(self, **overrides):
        values = {
            "required": True,
            "status": self.status(),
            "status_age_sec": 0.05,
            "limits": self.limits(),
            "x": 0.30,
            "y": 0.0,
            "yaw": 0.0,
        }
        values.update(overrides)
        return terrain_motion_gate(**values)

    def test_optional_gate_preserves_command(self):
        result = self.gate(required=False, status=None, status_age_sec=None)
        self.assertTrue(result.allowed)
        self.assertAlmostEqual(result.x, 0.30)

    def test_missing_stale_and_low_quality_status_block(self):
        self.assertEqual(
            self.gate(status=None).reason,
            "terrain_no_status",
        )
        self.assertEqual(
            self.gate(status_age_sec=0.31).reason,
            "terrain_status_stale",
        )
        self.assertEqual(
            self.gate(status=self.status(confidence=0.5)).reason,
            "terrain_low_confidence",
        )
        self.assertEqual(
            self.gate(status=self.status(support_ratio=0.5)).reason,
            "terrain_low_support",
        )

    def test_drop_dynamic_obstacle_and_gait_change_block(self):
        self.assertEqual(
            self.gate(status=self.status(terrain_class=TERRAIN_DROP)).reason,
            "terrain_drop",
        )
        self.assertEqual(
            self.gate(status=self.status(dynamic_obstacle=True)).reason,
            "terrain_dynamic_obstacle",
        )
        self.assertEqual(
            self.gate(status=self.status(gait_unchanged=False)).reason,
            "terrain_gait_changed",
        )

    def test_ramp_passes_when_quality_is_valid(self):
        result = self.gate(
            status=self.status(
                terrain_class=TERRAIN_RAMP,
                longitudinal_slope_rad=0.10,
            )
        )
        self.assertTrue(result.allowed)
        self.assertEqual(result.reason, "ramp_up")
        self.assertAlmostEqual(result.x, 0.25)

    def test_normal_and_ramp_respect_direction_authorization(self):
        forward = self.gate(status=self.status(allow_forward=False), x=0.30)
        self.assertFalse(forward.allowed)
        self.assertEqual(forward.reason, "terrain_forward_not_allowed")

        reverse = self.gate(status=self.status(allow_reverse=False), x=-0.20)
        self.assertFalse(reverse.allowed)
        self.assertEqual(reverse.reason, "terrain_reverse_not_allowed")

        ramp_reverse = self.gate(
            status=self.status(
                terrain_class=TERRAIN_RAMP,
                longitudinal_slope_rad=-0.10,
                allow_reverse=False,
            ),
            x=-0.20,
        )
        self.assertFalse(ramp_reverse.allowed)
        self.assertEqual(ramp_reverse.reason, "terrain_reverse_not_allowed")

    def test_ramp_direction_cross_slope_and_authorization_are_bounded(self):
        status = self.status(
            terrain_class=TERRAIN_RAMP,
            longitudinal_slope_rad=-0.10,
        )
        result = self.gate(status=status, x=0.30, yaw=0.40)
        self.assertTrue(result.allowed)
        self.assertEqual(result.reason, "ramp_down")
        self.assertAlmostEqual(result.x, 0.20)
        self.assertAlmostEqual(result.yaw, 0.10)

        too_steep = self.status(
            terrain_class=TERRAIN_RAMP,
            longitudinal_slope_rad=0.21,
        )
        self.assertEqual(self.gate(status=too_steep).reason, "ramp_up_slope")
        cross = self.status(
            terrain_class=TERRAIN_RAMP,
            cross_slope_rad=0.09,
        )
        self.assertEqual(self.gate(status=cross).reason, "ramp_cross_slope")

        limits = self.limits()._replace(ramp_up_max_x_mps=0.0)
        self.assertEqual(
            self.gate(
                status=self.status(
                    terrain_class=TERRAIN_RAMP,
                    longitudinal_slope_rad=0.10,
                ),
                limits=limits,
            ).reason,
            "ramp_speed_not_authorized",
        )

    def test_body_attitude_is_fail_closed(self):
        self.assertEqual(
            self.gate(status=self.status(body_roll_rad=0.21)).reason,
            "terrain_body_roll",
        )
        self.assertEqual(
            self.gate(status=self.status(body_pitch_rad=0.31)).reason,
            "terrain_body_pitch",
        )

    def test_stair_approach_preserves_scored_command_or_rejects(self):
        result = self.gate(
            status=self.status(traversal_state=STAIR_STATE_APPROACH),
            x=0.30,
            y=0.20,
            yaw=0.40,
        )
        self.assertTrue(result.allowed)
        self.assertEqual(result.reason, "stair_approach")
        self.assertAlmostEqual(result.x, 0.30)
        self.assertAlmostEqual(result.y, 0.20)
        self.assertAlmostEqual(result.yaw, 0.40)

        limits = self.limits()._replace(output_max_y_mps=0.0)
        rejected = self.gate(
            status=self.status(traversal_state=STAIR_STATE_APPROACH),
            limits=limits,
            y=0.20,
        )
        self.assertFalse(rejected.allowed)
        self.assertEqual(rejected.reason, "stair_approach_unscored_command")

    def test_stair_align_requires_an_already_scored_bounded_command(self):
        rejected = self.gate(
            status=self.status(traversal_state=STAIR_STATE_ALIGN),
            x=0.30,
            y=0.20,
            yaw=0.50,
        )
        self.assertFalse(rejected.allowed)
        self.assertEqual(rejected.reason, "stair_align_unscored_command")

        result = self.gate(
            status=self.status(traversal_state=STAIR_STATE_ALIGN),
            x=0.0,
            y=0.0,
            yaw=0.20,
        )
        self.assertTrue(result.allowed)
        self.assertEqual(result.x, 0.0)
        self.assertEqual(result.y, 0.0)
        self.assertAlmostEqual(result.yaw, 0.20)

    def test_stair_committed_is_forward_only_without_yaw(self):
        committed = self.status(traversal_state=STAIR_STATE_COMMITTED)
        result = self.gate(status=committed, x=0.20, y=0.0, yaw=0.0)
        self.assertTrue(result.allowed)
        self.assertAlmostEqual(result.x, 0.20)
        self.assertEqual(result.y, 0.0)
        self.assertEqual(result.yaw, 0.0)

        unscored = self.gate(status=committed, x=0.30, y=0.20, yaw=0.40)
        self.assertFalse(unscored.allowed)
        self.assertEqual(unscored.reason, "stair_committed_unscored_command")

        reverse = self.gate(status=committed, x=-0.10)
        self.assertFalse(reverse.allowed)
        self.assertEqual(reverse.reason, "stair_forward_only")

        unauthorized = self.gate(
            status=committed,
            limits=self.limits()._replace(committed_max_forward_mps=0.0),
        )
        self.assertFalse(unauthorized.allowed)
        self.assertEqual(unauthorized.reason, "stair_speed_not_authorized")

    def test_fault_latch_blocks(self):
        result = self.gate(
            status=self.status(traversal_state=STAIR_STATE_FAULT_LATCH)
        )
        self.assertFalse(result.allowed)
        self.assertEqual(result.reason, "stair_fault_latched")


class MotionProgressPolicyTest(unittest.TestCase):
    def test_sustained_mismatch_blocks_but_normal_progress_resets(self):
        monitor = MotionProgressMonitor(0.10, 0.05, 0.5, 0.30)
        self.assertIsNone(monitor.update(1.0, 0.30, 0.0))
        self.assertIsNone(monitor.update(1.29, 0.30, 0.0))
        self.assertEqual(
            monitor.update(1.31, 0.30, 0.0), "motion_progress_mismatch"
        )
        self.assertIsNone(monitor.update(1.32, 0.30, 0.20))
        self.assertIsNone(monitor.update(2.0, 0.30, 0.0))

    def test_nonfinite_odom_and_reverse_direction_are_rejected(self):
        monitor = MotionProgressMonitor(0.10, 0.05, 0.5, 0.30)
        self.assertEqual(monitor.update(1.0, 0.30, float("nan")), "odom_nonfinite")
        self.assertIsNone(monitor.update(2.0, -0.30, 0.20))
        self.assertEqual(
            monitor.update(2.31, -0.30, 0.20), "motion_progress_mismatch"
        )


class TerrainFaultLatchPolicyTest(unittest.TestCase):
    def test_startup_wait_does_not_latch_then_motion_fault_does(self):
        latch = TerrainFaultLatch()
        latch.observe(False, "terrain_no_status", motion_requested=True)
        self.assertFalse(latch.latched)
        latch.observe(True, "terrain_pass", motion_requested=False)
        latch.observe(False, "terrain_status_stale", motion_requested=True)
        self.assertTrue(latch.latched)
        self.assertEqual(latch.reason, "terrain_status_stale")
        self.assertFalse(latch.reset(stopped=False, inputs_healthy=True))
        self.assertTrue(latch.reset(stopped=True, inputs_healthy=True))

    def test_gait_and_drop_always_latch_after_observation(self):
        for reason in ("terrain_gait_changed", "terrain_drop"):
            with self.subTest(reason=reason):
                latch = TerrainFaultLatch()
                latch.observe(False, reason, motion_requested=False)
                self.assertTrue(latch.latched)

    def test_direction_denial_latches_after_gate_was_healthy(self):
        latch = TerrainFaultLatch()
        latch.observe(True, "terrain_pass", motion_requested=True)
        latch.observe(
            False, "terrain_reverse_not_allowed", motion_requested=True
        )
        self.assertTrue(latch.latched)
        self.assertEqual(latch.reason, "terrain_reverse_not_allowed")


if __name__ == "__main__":
    unittest.main()
