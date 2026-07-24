#!/usr/bin/env python3
"""Unit tests for SCAN mission JSON and arrival-window behavior."""

import importlib.util
import json
import math
import pathlib
import sys
import tempfile
import unittest


if len(sys.argv) > 1 and pathlib.Path(sys.argv[1]).is_file():
    module_path = pathlib.Path(sys.argv.pop(1))
else:
    module_path = pathlib.Path(__file__).parents[1] / "scripts" / "scan_mission_io.py"
spec = importlib.util.spec_from_file_location("scan_mission_io", module_path)
mission_io = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mission_io
assert spec.loader is not None
spec.loader.exec_module(mission_io)


class ScanMissionIoTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)
        self.initial_path = self.root / "fixed_start.json"
        self.mission_path = self.root / "route_a.json"
        initial = mission_io.initial_pose_document(
            "map",
            (1.0, 2.0, 0.32),
            mission_io.quaternion_from_yaw(0.5),
            [0.1 if index in (0, 7, 14, 21, 28, 35) else 0.0 for index in range(36)],
        )
        mission_io.atomic_write_json(self.initial_path, initial)

    def tearDown(self):
        self.temporary.cleanup()

    def write_mission(self, waypoints=None):
        if waypoints is None:
            waypoints = [
                mission_io.Waypoint("wp_001", 2.0, 3.0, 0.0, 0.2, 1.5),
                mission_io.Waypoint("wp_002", 4.0, 5.0, 0.1, -0.3, 0.0),
            ]
        document = mission_io.mission_document(
            "route_a", "fixed_start.json", waypoints
        )
        mission_io.atomic_write_json(self.mission_path, document)

    def test_round_trip_valid_bundle(self):
        self.write_mission()
        mission = mission_io.load_mission(
            self.mission_path, allowed_root=self.root
        )
        self.assertEqual(mission.mission_id, "route_a")
        self.assertEqual(len(mission.waypoints), 2)
        self.assertEqual(mission.initial_pose_path, self.initial_path)
        self.assertEqual(mission.waypoints[0].dwell_sec, 1.5)

    def test_requires_explicit_dwell(self):
        self.write_mission()
        document = json.loads(self.mission_path.read_text(encoding="utf-8"))
        del document["waypoints"][0]["dwell_sec"]
        mission_io.atomic_write_json(self.mission_path, document)
        with self.assertRaisesRegex(
            mission_io.MissionValidationError, "dwell_sec"
        ):
            mission_io.load_mission(self.mission_path)

    def test_rejects_duplicate_ids_and_empty_mission(self):
        duplicate = [
            mission_io.Waypoint("same", 0.0, 0.0, 0.0, 0.0, 0.0),
            mission_io.Waypoint("same", 1.0, 0.0, 0.0, 0.0, 0.0),
        ]
        self.write_mission(duplicate)
        with self.assertRaisesRegex(
            mission_io.MissionValidationError, "duplicate waypoint"
        ):
            mission_io.load_mission(self.mission_path)

        self.write_mission([])
        with self.assertRaisesRegex(
            mission_io.MissionValidationError, "at least one"
        ):
            mission_io.load_mission(self.mission_path)

    def test_rejects_path_escape_and_nonfinite_values(self):
        self.write_mission()
        document = json.loads(self.mission_path.read_text(encoding="utf-8"))
        document["initial_pose_file"] = "../outside_pose.json"
        mission_io.atomic_write_json(self.mission_path, document)
        with self.assertRaisesRegex(
            mission_io.MissionValidationError, "must stay under"
        ):
            mission_io.load_mission(self.mission_path, allowed_root=self.root)

        self.write_mission()
        document = json.loads(self.mission_path.read_text(encoding="utf-8"))
        document["waypoints"][0]["x"] = float("nan")
        mission_io.atomic_write_json(self.mission_path, document)
        with self.assertRaisesRegex(
            mission_io.MissionValidationError, "finite number"
        ):
            mission_io.load_mission(self.mission_path)

    def test_yaw_normalizes_across_pi_boundary(self):
        error = mission_io.normalize_angle(
            (-math.pi + 0.05) - (math.pi - 0.05)
        )
        self.assertAlmostEqual(error, 0.10)

    def test_arrival_requires_continuous_pose_and_zero_command(self):
        window = mission_io.ArrivalWindow(0.5)
        self.assertFalse(window.update(1.0, True, True))
        self.assertFalse(window.update(1.4, True, True))
        self.assertFalse(window.update(1.41, True, False))
        self.assertFalse(window.update(2.0, True, True))
        self.assertTrue(window.update(2.5, True, True))


if __name__ == "__main__":
    unittest.main()
