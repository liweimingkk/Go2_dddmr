#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "mission_command_mux_policy.py"
SPEC = importlib.util.spec_from_file_location("mission_command_mux_policy", SCRIPT)
mux_policy = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(mux_policy)


class MissionCommandMuxPolicyTest(unittest.TestCase):
    def select(self, **overrides):
        values = {
            "mode": "outdoor",
            "mode_age_sec": 0.05,
            "command": (0.2, 0.0, 0.1),
            "command_age_sec": 0.05,
            "decision": "d_controlling",
            "decision_age_sec": 0.05,
            "mode_timeout_sec": 0.30,
            "command_timeout_sec": 0.20,
            "decision_timeout_sec": 0.30,
            "allowed_motion_decisions": {"d_controlling"},
            "max_x": 0.35,
            "max_y": 0.0,
            "max_yaw": 0.50,
            "zero_epsilon": 0.001,
        }
        values.update(overrides)
        return mux_policy.select_command(**values)

    def test_passes_only_selected_fresh_authorized_source(self):
        command, decision, reason = self.select()
        self.assertEqual(command, (0.2, 0.0, 0.1))
        self.assertEqual(decision, "d_controlling")
        self.assertEqual(reason, "pass")

    def test_none_stale_and_disallowed_inputs_fail_closed(self):
        for overrides, expected_reason in (
            ({"mode": "none"}, "mode_none"),
            ({"mode_age_sec": 0.31}, "mode_stale"),
            ({"command_age_sec": 0.21}, "command_stale"),
            ({"decision_age_sec": 0.31}, "decision_stale"),
            ({"decision": "d_route_disabled"}, "decision_disallowed"),
        ):
            command, decision, reason = self.select(**overrides)
            self.assertEqual(command, (0.0, 0.0, 0.0))
            self.assertEqual(decision, "d_mission_stopped")
            self.assertEqual(reason, expected_reason)

    def test_zero_command_does_not_require_motion_decision(self):
        command, decision, reason = self.select(
            command=(0.0, 0.0, 0.0), decision="d_route_completed"
        )
        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(decision, "d_route_completed")
        self.assertEqual(reason, "pass")

    def test_clamps_authorized_command_and_blocks_nonfinite_values(self):
        command, _, reason = self.select(command=(1.0, 0.5, -1.0))
        self.assertEqual(command, (0.35, 0.0, -0.5))
        self.assertEqual(reason, "pass")
        command, decision, reason = self.select(command=(float("nan"), 0.0, 0.0))
        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(decision, "d_mission_stopped")
        self.assertEqual(reason, "command_invalid")


if __name__ == "__main__":
    unittest.main()
