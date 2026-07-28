#!/usr/bin/env python3

import pathlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as element_tree

import yaml


LAUNCH_DIRECTORY = pathlib.Path(__file__).resolve().parents[1] / "launch"
NAVIGATION_LAUNCH = LAUNCH_DIRECTORY / "go2_xt16_navigation.launch"
P2P_LAUNCH_NAME = "go2_xt16_p2p_move_base.launch.py"

sys.path.insert(0, str(LAUNCH_DIRECTORY))

from go2_xt16_p2p_runtime_parameters import (  # noqa: E402
    build_exact_runtime_parameters,
    write_exact_runtime_parameters,
)


class Go2Xt16P2PRuntimeParametersTest(unittest.TestCase):
    def test_navigation_launch_routes_p2p_through_exact_override_launcher(self):
        root = element_tree.parse(NAVIGATION_LAUNCH).getroot()
        includes = [
            include
            for include in root.findall("./include")
            if P2P_LAUNCH_NAME in include.attrib.get("file", "")
        ]

        self.assertEqual(len(includes), 1)
        include = includes[0]
        self.assertEqual(include.attrib.get("if"), "$(var start_move_base)")
        forwarded_arguments = {
            argument.attrib["name"]: argument.attrib.get("value")
            for argument in include.findall("./arg")
        }
        self.assertEqual(
            forwarded_arguments,
            {
                "config_file": "$(var config_file)",
                "p2p_goals_enabled": "$(var p2p_goals_enabled)",
                "local_lidar_expected_sensor_time_sec": (
                    "$(var local_lidar_expected_sensor_time_sec)"
                ),
                "omni_min_vel_y": "$(var omni_min_vel_y)",
                "omni_max_vel_y": "$(var omni_max_vel_y)",
            },
        )
        self.assertIsNone(
            root.find("./node[@exec='p2p_move_base_node']"),
            "the Foxy-incompatible anonymous inline-parameter node must be removed",
        )

    def test_builds_exact_absolute_node_overrides(self):
        parameters = build_exact_runtime_parameters(
            "0.35",
            "-0.0",
            "0.0",
            "false",
        )

        self.assertEqual(
            set(parameters),
            {
                "/perception_3d_local",
                "/trajectory_generators",
                "/p2p_move_base",
            },
        )
        self.assertEqual(
            parameters["/perception_3d_local"]["ros__parameters"][
                "lidar.expected_sensor_time"
            ],
            0.35,
        )
        planner = parameters["/trajectory_generators"]["ros__parameters"]
        self.assertEqual(planner["omni_drive_simple.min_vel_y"], -0.0)
        self.assertEqual(planner["omni_drive_simple.max_vel_y"], 0.0)
        self.assertFalse(
            parameters["/p2p_move_base"]["ros__parameters"]["goals_enabled"]
        )
        self.assertNotIn("/**", parameters)

    def test_written_yaml_preserves_exact_rules_and_double_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_exact_runtime_parameters(
                0.35,
                -0.2,
                0.2,
                True,
                directory=directory,
            )
            parsed = yaml.safe_load(
                pathlib.Path(path).read_text(encoding="utf-8")
            )

            self.assertIn("/perception_3d_local", parsed)
            self.assertIn("/trajectory_generators", parsed)
            self.assertIsInstance(
                parsed["/perception_3d_local"]["ros__parameters"][
                    "lidar.expected_sensor_time"
                ],
                float,
            )
            self.assertEqual(
                parsed["/trajectory_generators"]["ros__parameters"][
                    "omni_drive_simple.min_vel_y"
                ],
                -0.2,
            )

    def test_rejects_unsafe_or_inconsistent_values(self):
        invalid_cases = (
            (0.0, -0.1, 0.1, True),
            (0.351, -0.1, 0.1, True),
            (0.2, 0.2, -0.2, True),
            (0.2, float("nan"), 0.2, True),
            (0.2, -0.2, 0.2, "maybe"),
        )
        for values in invalid_cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                build_exact_runtime_parameters(*values)


if __name__ == "__main__":
    unittest.main()
