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
FEATURE_ASSOCIATION = (
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "src"
    / "featureAssociation.cpp"
)
IMAGE_PROJECTION = (
    WORKSPACE
    / "src"
    / "dddmr_lego_loam"
    / "lego_loam_bor"
    / "src"
    / "imageProjection.cpp"
)
MULTILAYER_LIDAR = (
    WORKSPACE
    / "src"
    / "dddmr_perception_3d"
    / "plugins"
    / "multilayer_spinning_lidar.cpp"
)
MCL_IMPLEMENTATION = (
    WORKSPACE / "src" / "dddmr_mcl_3dl" / "src" / "mcl_3dl.cpp"
)


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

    def test_local_observation_gate_retries_without_relaxing_freshness(self):
        script = NAVIGATION_WRAPPER.read_text(encoding="utf-8")

        self.assertIn(
            'OBSERVATION_GATE_ATTEMPTS_VALUE="${GO2_NAV_OBSERVATION_GATE_ATTEMPTS:-3}"',
            script,
        )
        self.assertIn(
            "attempt <= OBSERVATION_GATE_ATTEMPTS_VALUE",
            script,
        )
        self.assertIn(
            "'BEGIN { exit !(value <= 0.35) }'",
            script,
        )
        self.assertIn(
            "--max-header-age-sec "
            "'${LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE}'",
            script,
        )
        self.assertIn(
            "'BEGIN { exit !(value <= 0.05) }'",
            script,
        )
        self.assertIn(
            "'BEGIN { exit !(value <= 0.40) }'",
            script,
        )
        self.assertIn("--reliability reliable", script)

    def test_startup_gates_validate_actual_mcl_input_and_retry_odom(self):
        script = NAVIGATION_WRAPPER.read_text(encoding="utf-8")

        self.assertIn(
            'MCL_FEATURE_TOPIC="/laser_cloud_less_sharp"',
            script,
        )
        self.assertIn(
            'ODOM_SYNC_GATE_ATTEMPTS_VALUE="${GO2_NAV_ODOM_SYNC_GATE_ATTEMPTS:-3}"',
            script,
        )
        self.assertIn(
            "attempt <= ODOM_SYNC_GATE_ATTEMPTS_VALUE",
            script,
        )
        self.assertIn(
            "trap cleanup_runtime_on_exit EXIT",
            script,
        )

    def test_live_mcl_feature_transport_cannot_reliably_backpressure(self):
        feature_source = FEATURE_ASSOCIATION.read_text(encoding="utf-8")
        mcl_source = MCL_IMPLEMENTATION.read_text(encoding="utf-8")

        for topic in (
            "laser_cloud_sharp",
            "laser_cloud_less_sharp",
            "laser_cloud_flat",
            "laser_cloud_less_flat",
        ):
            with self.subTest(topic=topic):
                self.assertIn(
                    f'("{topic}", rclcpp::SensorDataQoS())',
                    feature_source,
                )
                self.assertIn(
                    f'"{topic}", rmw_qos_profile_sensor_data',
                    mcl_source,
                )

    def test_large_local_obstacle_cloud_uses_end_to_end_reliable_depth_one(self):
        projection_source = IMAGE_PROJECTION.read_text(encoding="utf-8")
        publisher_start = projection_source.index(
            "_pub_segmented_cloud_pure = this->create_publisher"
        )
        publisher_end = projection_source.index(");", publisher_start)
        publisher = projection_source[publisher_start:publisher_end]

        self.assertIn('"segmented_cloud_pure"', publisher)
        self.assertIn("rclcpp::KeepLast(1)", publisher)
        self.assertIn("durability_volatile()", publisher)
        self.assertIn("reliable()", publisher)
        self.assertNotIn("SensorDataQoS", publisher)

        lidar_source = MULTILAYER_LIDAR.read_text(encoding="utf-8")
        subscription_start = lidar_source.index(
            "rclcpp::QoS sensor_qos(rclcpp::KeepLast(1))"
        )
        subscription_end = lidar_source.index(
            "sensor_sub_ = node_->create_subscription", subscription_start
        )
        subscription_qos = lidar_source[subscription_start:subscription_end]
        self.assertIn("sensor_qos.durability_volatile()", subscription_qos)
        self.assertIn("if(is_local_planner_)", subscription_qos)
        self.assertIn("sensor_qos.reliable()", subscription_qos)
        self.assertIn("sensor_qos.best_effort()", subscription_qos)

        observation_qos_start = lidar_source.index(
            "rclcpp::QoS current_observation_qos(rclcpp::KeepLast(2))"
        )
        observation_qos_end = lidar_source.index(
            "pub_current_observation_ = node_->create_publisher",
            observation_qos_start,
        )
        observation_qos = lidar_source[
            observation_qos_start:observation_qos_end
        ]
        self.assertIn("if(is_local_planner_)", observation_qos)
        self.assertIn("current_observation_qos.reliable()", observation_qos)
        self.assertIn("current_observation_qos.best_effort()", observation_qos)

    def test_tracking_thresholds_keep_margin_to_lost(self):
        config = yaml.safe_load(
            RELOCALIZATION_CONFIG.read_text(encoding="utf-8")
        )["mcl_3dl"]["ros__parameters"]

        self.assertEqual(config["localization_tracking_max_residual"], 0.22)
        self.assertLess(
            config["localization_tracking_max_residual"],
            config["localization_lost_max_residual"],
        )
        self.assertGreaterEqual(
            config["localization_lost_max_residual"]
            - config["localization_tracking_max_residual"],
            0.03,
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
