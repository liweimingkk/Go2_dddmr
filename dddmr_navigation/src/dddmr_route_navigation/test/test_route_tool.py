#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "route_tool.py"
SPEC = importlib.util.spec_from_file_location("route_tool", SCRIPT)
route_tool = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(route_tool)


PCD = """# .PCD v0.7
VERSION 0.7
FIELDS x y z intensity roll pitch yaw time
SIZE 4 4 4 4 4 4 4 8
TYPE F F F F F F F F
COUNT 1 1 1 1 1 1 1 1
WIDTH 5
HEIGHT 1
POINTS 5
DATA ascii
0.000 0.000 0.000 0 0.0 0.0 0.0 1.0
0.005 0.000 0.000 1 0.0 0.0 0.1 2.0
0.500 0.000 0.100 2 0.0 0.1 0.2 3.0
1.000 0.000 0.200 3 0.0 0.2 0.3 4.0
1.500 0.000 0.300 4 0.0 0.3 0.4 5.0
"""


class RouteToolTest(unittest.TestCase):
    def test_pose_graph_conversion_filters_stationary_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            pcd = Path(directory) / "poses.pcd"
            pcd.write_text(PCD, encoding="utf-8")
            points = route_tool.filter_duplicate_points(
                route_tool.points_from_pose_graph(pcd), 0.02
            )
            self.assertEqual(len(points), 4)
            self.assertAlmostEqual(points[0]["x"], 0.0)
            self.assertAlmostEqual(route_tool.yaw_from_quaternion(points[0]), 0.1)
            route_tool.validate_points(points)
            self.assertGreater(route_tool.route_length(points), 1.5)

    def test_rejects_binary_pcd(self):
        with tempfile.TemporaryDirectory() as directory:
            pcd = Path(directory) / "poses.pcd"
            pcd.write_text(
                "FIELDS x y z roll pitch yaw\nDATA binary\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "DATA ascii"):
                route_tool.points_from_pose_graph(pcd)

    def test_document_round_trip_is_atomic_and_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.json"
            points = route_tool.filter_duplicate_points(
                [
                    {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                    {"x": 0.5, "y": 0.0, "z": 0.1, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                    {"x": 1.0, "y": 0.0, "z": 0.2, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                ],
                0.02,
            )
            document = route_tool.build_document(
                points, "test-route", "map", {"type": "test"}, None, 2.0
            )
            route_tool.write_document(path, document)
            loaded = route_tool.load_document(path)
            self.assertEqual(loaded["schema"], route_tool.SCHEMA)
            self.assertEqual(len(loaded["points"]), 3)
            self.assertEqual(list(Path(directory).glob("*.tmp")), [])

    def test_rejects_unnormalized_quaternion_and_route_gap(self):
        points = [
            {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 2.0},
            {"x": 0.5, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            {"x": 1.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        ]
        with self.assertRaisesRegex(ValueError, "not normalized"):
            route_tool.validate_points(points)
        points[0]["qw"] = 1.0
        points[2]["x"] = 4.0
        with self.assertRaisesRegex(ValueError, "maximum"):
            route_tool.validate_points(points)


if __name__ == "__main__":
    unittest.main()
