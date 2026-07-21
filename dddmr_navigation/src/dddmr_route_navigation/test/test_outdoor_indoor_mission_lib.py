#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "outdoor_indoor_mission_lib.py"
SPEC = importlib.util.spec_from_file_location("outdoor_indoor_mission_lib", SCRIPT)
mission_lib = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(mission_lib)


class OutdoorIndoorMissionLibTest(unittest.TestCase):
    def make_map_and_route(self, root: Path):
        map_directory = root / "map"
        route_directory = root / "routes"
        map_directory.mkdir()
        route_directory.mkdir()
        for name in mission_lib.MAP_FINGERPRINT_FILES:
            (map_directory / name).write_bytes((name + "\n").encode("ascii"))
        fingerprint = mission_lib.map_fingerprint(map_directory)
        point = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        document = {
            "schema": mission_lib.ROUTE_SCHEMA,
            "route_id": "front-door",
            "frame_id": "map",
            "map": {"sha256": fingerprint},
            "points": [dict(point), dict(point, x=0.5), dict(point, x=1.0)],
        }
        route_path = route_directory / "front-door.json"
        route_path.write_text(json.dumps(document), encoding="utf-8")
        return map_directory, route_directory, route_path

    def test_loads_route_only_when_unified_map_fingerprint_matches(self):
        with tempfile.TemporaryDirectory() as directory:
            map_directory, route_directory, route_path = self.make_map_and_route(
                Path(directory)
            )
            loaded_path, document, fingerprint = mission_lib.load_validated_route(
                route_directory, "front-door", map_directory
            )
            self.assertEqual(loaded_path, route_path)
            self.assertEqual(document["route_id"], "front-door")
            self.assertEqual(fingerprint, document["map"]["sha256"])

            (map_directory / "map.pcd").write_text("changed\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "fingerprint mismatch"):
                mission_lib.load_validated_route(
                    route_directory, "front-door", map_directory
                )

    def test_rejects_path_traversal_and_non_planar_goal(self):
        with tempfile.TemporaryDirectory() as directory:
            _, route_directory, _ = self.make_map_and_route(Path(directory))
            with self.assertRaisesRegex(ValueError, "route_id"):
                mission_lib.route_file_for_id(route_directory, "../outside")
        with self.assertRaisesRegex(ValueError, "roll/pitch"):
            mission_lib.validate_planar_goal(
                "map", (1.0, 2.0, 0.0), (0.1, 0.0, 0.0, 0.994987437)
            )

    def test_continuous_stop_requires_fresh_low_speed_odometry(self):
        detector = mission_lib.ContinuousStopDetector(1.0, 0.5, 0.03, 0.05)
        self.assertFalse(detector.update(10.0, 10.0, 0.02, 0.04))
        self.assertFalse(detector.update(10.6, 10.0, 0.02, 0.04))
        self.assertFalse(detector.update(11.0, 11.0, 0.02, 0.04))
        self.assertTrue(detector.update(12.0, 12.0, 0.02, 0.04))
        self.assertFalse(detector.update(12.1, 12.1, 0.04, 0.01))


if __name__ == "__main__":
    unittest.main()
