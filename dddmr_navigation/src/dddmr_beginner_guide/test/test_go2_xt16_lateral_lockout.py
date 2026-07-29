#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as element_tree

import yaml


PACKAGE_DIRECTORY = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = pathlib.Path(__file__).resolve().parents[3]
NAVIGATION_CONFIG = PACKAGE_DIRECTORY / "config" / "go2_xt16_navigation.yaml"
RELOCALIZATION_CONFIG = (
    PACKAGE_DIRECTORY / "config" / "go2_xt16_relocalization.yaml"
)
NAVIGATION_LAUNCH = PACKAGE_DIRECTORY / "launch" / "go2_xt16_navigation.launch"
NAVIGATION_WRAPPER = WORKSPACE / "scripts" / "run_go2_xt16_navigation_test.sh"


class Go2Xt16LateralLockoutTest(unittest.TestCase):
    def test_planner_has_only_one_zero_lateral_sample(self):
        config = yaml.safe_load(NAVIGATION_CONFIG.read_text(encoding="utf-8"))
        planner = config["trajectory_generators"]["ros__parameters"][
            "omni_drive_simple"
        ]

        self.assertEqual(planner["min_vel_y"], 0.0)
        self.assertEqual(planner["max_vel_y"], 0.0)
        self.assertEqual(planner["linear_y_sample"], 1.0)

    def test_all_direct_launch_command_limits_default_to_zero(self):
        root = element_tree.parse(NAVIGATION_LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("./arg")
        }

        for name in (
            "omni_min_vel_y",
            "omni_max_vel_y",
            "go2_nav_cmd_gate_max_y",
            "sport_dry_run_max_y",
            "go2_sport_max_y",
        ):
            with self.subTest(name=name):
                self.assertEqual(arguments[name], "0.0")

    def test_supervised_wrapper_rejects_nonzero_lateral_limit(self):
        script = NAVIGATION_WRAPPER.read_text(encoding="utf-8")

        self.assertIn('MAX_Y_VALUE="${MAX_Y:-0.0}"', script)
        self.assertIn(
            'die "Lateral motion is disabled: MAX_Y must be exactly 0."',
            script,
        )
        self.assertNotIn('MAX_Y_VALUE="0.20"', script)

    def test_tracking_thresholds_keep_margin_to_lost(self):
        config = yaml.safe_load(
            RELOCALIZATION_CONFIG.read_text(encoding="utf-8")
        )["mcl_3dl"]["ros__parameters"]

        self.assertEqual(config["localization_tracking_max_residual"], 0.20)
        self.assertLess(
            config["localization_tracking_max_residual"],
            config["localization_lost_max_residual"],
        )
        self.assertEqual(
            config["localization_tracking_max_ground_normal_error"],
            0.14,
        )
        self.assertLess(
            config["localization_tracking_max_ground_normal_error"],
            config["localization_lost_max_ground_normal_error"],
        )


if __name__ == "__main__":
    unittest.main()
