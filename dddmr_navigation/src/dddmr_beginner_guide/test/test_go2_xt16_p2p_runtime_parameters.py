#!/usr/bin/env python3

import pathlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as element_tree

import yaml


LAUNCH_DIRECTORY = pathlib.Path(__file__).resolve().parents[1] / "launch"
NAVIGATION_LAUNCH = LAUNCH_DIRECTORY / "go2_xt16_navigation.launch"
RELOCALIZATION_CONFIG = (
    pathlib.Path(__file__).resolve().parents[1]
    / "config"
    / "go2_xt16_relocalization.yaml"
)
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

    def test_p2p_pose_freshness_budget_exceeds_stationary_mcl_cadence(self):
        root = element_tree.parse(NAVIGATION_LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("./arg")
        }
        p2p_input_timeout = float(
            arguments["p2p_mission_input_timeout_sec"]
        )

        relocalization = yaml.safe_load(
            RELOCALIZATION_CONFIG.read_text(encoding="utf-8")
        )
        measure_interval = float(
            relocalization["mcl_3dl"]["ros__parameters"][
                "localization_measure_interval_sec"
            ]
        )
        self.assertGreaterEqual(
            p2p_input_timeout,
            measure_interval + 0.50,
            "P2P freshness must include stationary MCL compute jitter",
        )
        self.assertLessEqual(
            p2p_input_timeout,
            1.50,
            "P2P freshness must remain a short fail-closed window",
        )

        executor = root.find(
            "./node[@exec='p2p_mission_executor.py']"
        )
        self.assertIsNotNone(executor)
        parameters = {
            parameter.attrib["name"]: parameter.attrib
            for parameter in executor.findall("./param")
        }
        self.assertEqual(
            parameters["input_timeout_sec"].get("value"),
            "$(var p2p_mission_input_timeout_sec)",
        )
        self.assertEqual(parameters["input_timeout_sec"].get("type"), "float")

        command_gate = relocalization["go2_nav_cmd_gate"]["ros__parameters"]
        self.assertTrue(command_gate["require_localization_pose"])
        self.assertEqual(command_gate["localization_pose_topic"], "/mcl_pose")
        self.assertEqual(
            float(command_gate["localization_pose_timeout_sec"]),
            p2p_input_timeout,
            "the command gate and mission executor must share the MCL pose budget",
        )
        sensor_timeout = float(
            relocalization["mcl_3dl"]["ros__parameters"][
                "localization_sensor_timeout_sec"
            ]
        )
        self.assertGreater(
            sensor_timeout,
            p2p_input_timeout,
            "motion must stop on stale pose before MCL enters LOST",
        )
        self.assertLessEqual(
            sensor_timeout,
            2.0,
            "a sustained feature outage must still mark localization LOST",
        )

    def test_localization_health_recovery_window_stays_bounded(self):
        root = element_tree.parse(NAVIGATION_LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("./arg")
        }
        recovery_window = float(
            arguments["p2p_localization_health_grace_sec"]
        )
        self.assertGreaterEqual(
            recovery_window,
            5.0,
            "the observed stopped MCL recovery took about 4.8 seconds",
        )
        self.assertLessEqual(
            recovery_window,
            6.0,
            "persistent localization degradation must still cancel promptly",
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
        self.assertEqual(planner["omni_drive_simple.linear_y_sample"], 1.0)
        self.assertFalse(
            parameters["/p2p_move_base"]["ros__parameters"]["goals_enabled"]
        )
        self.assertNotIn("/**", parameters)

    def test_written_yaml_preserves_exact_rules_and_double_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_exact_runtime_parameters(
                0.35,
                0.0,
                0.0,
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
                0.0,
            )
            self.assertEqual(
                parsed["/trajectory_generators"]["ros__parameters"][
                    "omni_drive_simple.linear_y_sample"
                ],
                1.0,
            )

    def test_rejects_unsafe_or_inconsistent_values(self):
        invalid_cases = (
            (0.0, -0.1, 0.1, True),
            (0.351, -0.1, 0.1, True),
            (0.2, -0.2, 0.2, True),
            (0.2, 0.2, -0.2, True),
            (0.2, float("nan"), 0.2, True),
            (0.2, -0.2, 0.2, "maybe"),
        )
        for values in invalid_cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                build_exact_runtime_parameters(*values)


if __name__ == "__main__":
    unittest.main()
