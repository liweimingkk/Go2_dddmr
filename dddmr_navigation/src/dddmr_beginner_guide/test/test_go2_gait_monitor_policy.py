#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from go2_gait_monitor_policy import GaitStateContract  # noqa: E402


class GaitStateContractTest(unittest.TestCase):
    def test_first_sample_latches_unspecified_baseline(self):
        contract = GaitStateContract(timeout_sec=0.30)
        result = contract.observe(mode=0, gait_type=1, now_sec=10.0)
        self.assertTrue(result.unchanged)
        self.assertEqual(result.baseline_mode, 0)
        self.assertEqual(result.baseline_gait_type, 1)

    def test_configured_baseline_fails_on_wrong_first_sample(self):
        contract = GaitStateContract(
            expected_mode=1, expected_gait_type=2, timeout_sec=0.30
        )
        result = contract.observe(mode=0, gait_type=2, now_sec=10.0)
        self.assertFalse(result.unchanged)
        self.assertEqual(result.reason, "mode_changed")

    def test_mode_change_latches_even_if_value_returns(self):
        contract = GaitStateContract(timeout_sec=0.30)
        contract.observe(mode=0, gait_type=1, now_sec=10.0)
        changed = contract.observe(mode=1, gait_type=1, now_sec=10.1)
        returned = contract.observe(mode=0, gait_type=1, now_sec=10.2)
        self.assertEqual(changed.reason, "mode_changed")
        self.assertEqual(returned.reason, "mode_changed")
        self.assertFalse(returned.unchanged)

    def test_gait_change_latches_even_if_value_returns(self):
        contract = GaitStateContract(timeout_sec=0.30)
        contract.observe(mode=0, gait_type=1, now_sec=10.0)
        changed = contract.observe(mode=0, gait_type=2, now_sec=10.1)
        returned = contract.observe(mode=0, gait_type=1, now_sec=10.2)
        self.assertEqual(changed.reason, "gait_changed")
        self.assertEqual(returned.reason, "gait_changed")

    def test_missing_and_stale_status_fail_closed(self):
        contract = GaitStateContract(timeout_sec=0.30)
        self.assertEqual(contract.evaluate(10.0).reason, "no_status")
        contract.observe(mode=0, gait_type=1, now_sec=10.0)
        self.assertEqual(contract.evaluate(10.31).reason, "stale_status")
        self.assertTrue(contract.observe(mode=0, gait_type=1, now_sec=10.32).unchanged)

    def test_invalid_status_and_time_regression_latch(self):
        invalid = GaitStateContract(timeout_sec=0.30)
        self.assertEqual(
            invalid.observe(mode=-1, gait_type=0, now_sec=10.0).reason,
            "invalid_status",
        )

        regressed = GaitStateContract(timeout_sec=0.30)
        regressed.observe(mode=0, gait_type=1, now_sec=10.0)
        self.assertEqual(
            regressed.observe(mode=0, gait_type=1, now_sec=9.0).reason,
            "time_regression",
        )

    def test_invalid_configuration_rejected(self):
        for kwargs in (
            {"expected_mode": -2},
            {"expected_gait_type": -2},
            {"timeout_sec": 0.0},
            {"timeout_sec": float("nan")},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    GaitStateContract(**kwargs)


if __name__ == "__main__":
    unittest.main()
