#!/usr/bin/env python3

from __future__ import annotations

import copy
from contextlib import redirect_stdout
import io
import math
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml


DDDMR_ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(DDDMR_ROOT / "scripts"))

from validate_go2_xt16_terrain_site_profile import (  # noqa: E402
    DEFAULT_PROFILE,
    ProfileValidationError,
    main,
    validate_profile,
    validate_runtime_config,
)


MAP_HASH = "a" * 64
RUNTIME_CONFIG = DDDMR_ROOT / "src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"


def valid_profile() -> dict:
    # Synthetic geometry used only to exercise the pure validator.  These are
    # not site coordinates and this fixture is never loaded by a ROS node.
    return {
        "schema_version": 1,
        "profile": {
            "enabled": True,
            "site_id": "indoor-test-site-01",
            "capability_profile_version": "site-v1",
            "software_commit": "1234abc",
        },
        "map": {
            "frame_id": "map",
            "sha256": MAP_HASH,
            "complete_ground_voxel_size_m": 0.20,
        },
        "terrain_roi": {
            "enabled": True,
            "source_map_sha256": MAP_HASH,
            "frame_id": "map",
            "voxel_size_m": 0.05,
            "min_xyz": [-1.0, -1.0, -0.5],
            "max_xyz": [5.0, 2.0, 2.0],
        },
        "terrain": {
            "enabled": True,
            "fail_closed": True,
            "source_map_sha256": MAP_HASH,
            "model": {
                "normal_radius_m": 0.30,
                "min_neighbors": 8,
                "max_plane_residual_m": 0.04,
                "edge_sample_spacing_m": 0.05,
                "support_radius_m": 0.10,
                "min_support_ratio": 0.85,
                "min_confidence": 0.90,
                "max_unknown_ratio": 0.0,
                "max_age_s": 0.20,
            },
            "traversal": {
                "max_roughness_m": 0.02,
                "max_normal_change_deg": 10.0,
                "max_body_roll_deg": 10.0,
                "max_body_pitch_deg": 15.0,
                "ramp": {
                    "enabled": True,
                    "max_up_slope_deg": 10.0,
                    "max_down_slope_deg": 8.0,
                    "max_cross_slope_deg": 4.0,
                    "max_up_speed_mps": 0.30,
                    "max_down_speed_mps": 0.25,
                    "max_yaw_rps": 0.10,
                },
                "generic": {"max_step_up_m": 0.0, "max_step_down_m": 0.0},
            },
        },
        "ramp_capability": {
            "verified": True,
            "source_map_sha256": MAP_HASH,
            "max_up_slope_deg": 12.0,
            "max_down_slope_deg": 10.0,
            "max_cross_slope_deg": 5.0,
            "max_roughness_m": 0.03,
            "max_normal_change_deg": 12.0,
            "max_body_roll_deg": 12.0,
            "max_body_pitch_deg": 18.0,
            "minimum_observed_support_ratio": 0.80,
            "v_exec_min_mps": 0.20,
            "max_up_speed_mps": 0.35,
            "max_down_speed_mps": 0.30,
            "max_yaw_rps": 0.15,
            "evidence_ids": ["ramp-manual-normal-gait-run-001"],
        },
        "stair": {
            "enabled": True,
            "require_manual_corridor": True,
            "require_online_confirmation": True,
            "min_confidence": 0.90,
            "max_riser_height_m": 0.22,
            "min_tread_depth_m": 0.28,
            "max_tread_depth_m": 0.32,
            "max_riser_deviation_m": 0.02,
            "max_tread_deviation_m": 0.03,
            "step_count_semantics": "riser_count",
            "highest_tread_semantics": "level_with_upper_landing",
            "max_height_closure_error_m": 0.02,
            "max_heading_error_deg": 8.0,
            "max_lateral_error_m": 0.10,
            "max_body_roll_deg": 10.0,
            "max_body_pitch_deg": 20.0,
            "v_exec_min_mps": 0.20,
            "max_up_speed_mps": 0.30,
            "max_down_speed_mps": 0.25,
            "max_committed_yaw_rps": 0.0,
            "max_step_index_delta": 1,
            "allow_in_place_rotation": False,
            "allow_replanning_while_committed": False,
            "normal_gait_constraint": {
                "enabled": True,
                "policy": "latch_normal_navigation_gait_at_test_start",
                "allow_gait_change_requests": False,
                "require_state_monitor": True,
                "require_sport_request_audit": True,
                "expected_gait_type": None,
            },
            "riser_semantics": {
                "enabled": True,
                "max_snapshot_age_s": 0.20,
                "max_node_match_distance_m": 0.08,
                "riser_plane_tolerance_m": 0.04,
                "riser_lateral_tolerance_m": 0.02,
                "riser_vertical_tolerance_m": 0.04,
                "surface_plane_tolerance_m": 0.04,
                "leg_envelope_min_xyz": [-0.40, -0.30, -0.20],
                "leg_envelope_max_xyz": [0.40, 0.30, 0.0],
                "max_support_xy_distance_m": 0.15,
                "min_body_clearance_m": 0.15,
                "max_body_clearance_m": 0.35,
            },
            "corridors": [
                {
                    "enabled": True,
                    "id": 17,
                    "source_map_sha256": MAP_HASH,
                    "capability_profile_version": "site-v1",
                    "up_axis_xyz": [1.0, 0.0, 0.0],
                    "first_riser_center_xyz": [0.5, 0.0, 0.10],
                    "corridor_polygon_xy": [
                        [0.5, -0.5],
                        [3.5, -0.5],
                        [3.5, 0.5],
                        [0.5, 0.5],
                    ],
                    "lower_landing": {
                        "center_xyz": [0.25, 0.0, 0.0],
                        "polygon_xy": [
                            [0.0, -0.5],
                            [0.5, -0.5],
                            [0.5, 0.5],
                            [0.0, 0.5],
                        ],
                    },
                    "upper_landing": {
                        "center_xyz": [3.75, 0.0, 1.0],
                        "polygon_xy": [
                            [3.5, -0.5],
                            [4.0, -0.5],
                            [4.0, 0.5],
                            [3.5, 0.5],
                        ],
                    },
                    "width_m": 1.0,
                    "riser_height_m": 0.20,
                    "tread_depth_m": 0.30,
                    "step_count": 5,
                    "confidence": 0.95,
                    "maximum_surface_roughness_m": 0.03,
                    "minimum_observed_support_ratio": 0.80,
                    "geometry_evidence_ids": ["stair-survey-001"],
                    "allow_up": True,
                    "allow_down": True,
                    "normal_gait_up_evidence_ids": ["manual-stair-up-001"],
                    "normal_gait_down_evidence_ids": ["manual-stair-down-001"],
                }
            ],
        },
        "runtime_activation": {
            "terrain_layers_enabled": True,
            "global_edge_policy_enabled": True,
            "terrain_trajectory_generator_enabled": True,
            "local_support_critic_enabled": True,
            "goal_surface_match_enabled": True,
            "stair_supervisor_enabled": True,
            "stair_riser_collision_semantics_enabled": True,
            "stair_riser_marking_enabled": True,
            "terrain_command_gate_enabled": True,
            "gait_monitor_enabled": True,
        },
    }


def _params(runtime: dict, node_name: str) -> dict:
    return runtime[node_name]["ros__parameters"]


def _flatten(vertices: list[list[float]]) -> list[float]:
    return [coordinate for vertex in vertices for coordinate in vertex]


def valid_runtime_config(profile: dict, target: str) -> dict:
    runtime = yaml.safe_load(RUNTIME_CONFIG.read_text(encoding="utf-8"))
    stair_target = target.startswith("stair")
    terrain_profile = profile["terrain"]
    model = terrain_profile["model"]
    traversal = terrain_profile["traversal"]
    ramp = traversal["ramp"]
    stair = profile["stair"]
    map_hash = profile["map"]["sha256"]

    map_params = _params(runtime, "map1")
    map_params["source_map_sha256"] = map_hash
    map_params["terrain_roi_enabled"] = True
    map_params["terrain_roi_voxel_size"] = profile["terrain_roi"]["voxel_size_m"]
    for index, axis in enumerate("xyz"):
        map_params[f"terrain_roi_min_{axis}"] = profile["terrain_roi"]["min_xyz"][index]
        map_params[f"terrain_roi_max_{axis}"] = profile["terrain_roi"]["max_xyz"][index]

    corridors = [entry for entry in stair["corridors"] if entry["enabled"]]
    for node_name in ("perception_3d_local", "perception_3d_global"):
        params = _params(runtime, node_name)
        terrain = params["terrain"]
        terrain["enabled"] = True
        terrain["map_hash"] = map_hash
        terrain["max_age_sec"] = model["max_age_s"]
        terrain["status_min_confidence"] = model["min_confidence"]
        terrain["status_min_support_ratio"] = model["min_support_ratio"]
        terrain["model"] = {
            "normal_radius_m": model["normal_radius_m"],
            "min_normal_neighbors": model["min_neighbors"],
            "max_plane_residual_m": model["max_plane_residual_m"],
        }
        if stair_target:
            terrain["stair_ids"] = [entry["id"] for entry in corridors]
            terrain["stairs"] = {}
            for corridor in corridors:
                lower = corridor["lower_landing"]
                upper = corridor["upper_landing"]
                terrain["stairs"][str(corridor["id"])] = {
                    "map_hash": map_hash,
                    "up_axis": corridor["up_axis_xyz"],
                    "lower_landing_center": lower["center_xyz"],
                    "upper_landing_center": upper["center_xyz"],
                    "first_riser_center": corridor["first_riser_center_xyz"],
                    "corridor_polygon_xy": _flatten(corridor["corridor_polygon_xy"]),
                    "lower_landing_polygon_xy": _flatten(lower["polygon_xy"]),
                    "upper_landing_polygon_xy": _flatten(upper["polygon_xy"]),
                    "width_m": corridor["width_m"],
                    "riser_height_m": corridor["riser_height_m"],
                    "tread_depth_m": corridor["tread_depth_m"],
                    "step_count": corridor["step_count"],
                    "confidence": corridor["confidence"],
                    "allow_up": corridor["allow_up"],
                    "allow_down": corridor["allow_down"],
                }
        else:
            terrain["stair_ids"] = []

    global_terrain = _params(runtime, "global_planner")["terrain"]
    global_terrain.update(
        {
            "enabled": True,
            "fail_closed": True,
            "map_hash": map_hash,
            "support_sample_spacing_m": model["edge_sample_spacing_m"],
            "support_search_radius_m": model["support_radius_m"],
            "continuous_height_residual_m": model["max_plane_residual_m"],
            "max_up_slope_rad": math.radians(ramp["max_up_slope_deg"]),
            "max_down_slope_rad": math.radians(ramp["max_down_slope_deg"]),
            "max_cross_slope_rad": math.radians(ramp["max_cross_slope_deg"]),
            "max_roughness_m": traversal["max_roughness_m"],
            "max_normal_change_rad": math.radians(
                traversal["max_normal_change_deg"]
            ),
            "max_step_up_m": traversal["generic"]["max_step_up_m"],
            "max_step_down_m": traversal["generic"]["max_step_down_m"],
            "min_support_ratio": model["min_support_ratio"],
            "max_unknown_ratio": model["max_unknown_ratio"],
            "min_confidence": model["min_confidence"],
            "max_support_sample_spacing_m": model["edge_sample_spacing_m"],
            "stair_enabled": stair_target,
            "allow_stair_up": stair_target and any(value["allow_up"] for value in corridors),
            "allow_stair_down": stair_target and any(
                value["allow_down"] for value in corridors
            ),
        }
    )
    if stair_target:
        global_terrain.update(
            {
                "max_stair_riser_height_m": stair["max_riser_height_m"],
                "min_stair_tread_depth_m": stair["min_tread_depth_m"],
                "max_stair_tread_depth_m": stair["max_tread_depth_m"],
                "max_stair_riser_deviation_m": stair["max_riser_deviation_m"],
                "max_stair_tread_deviation_m": stair["max_tread_deviation_m"],
                "max_stair_heading_error_rad": math.radians(
                    stair["max_heading_error_deg"]
                ),
            }
        )

    generator = _params(runtime, "trajectory_generators")[
        "differential_drive_terrain"
    ]
    generator.update(
        {
            "terrain_enabled": True,
            "terrain_fail_closed": True,
            "terrain_min_support_ratio": model["min_support_ratio"],
            "terrain_min_confidence": model["min_confidence"],
            "terrain_max_normal_change_rad": math.radians(
                traversal["max_normal_change_deg"]
            ),
            "terrain_stairs_enabled": stair_target,
            "max_vel_x": (
                stair["max_up_speed_mps"]
                if target == "stair-up"
                else stair["max_down_speed_mps"]
                if target == "stair-down"
                else min(ramp["max_up_speed_mps"], ramp["max_down_speed_mps"])
            ),
            "max_vel_theta": stair["max_committed_yaw_rps"]
            if stair_target
            else ramp["max_yaw_rps"],
            "terrain_max_stair_heading_error_rad": math.radians(
                stair["max_heading_error_deg"]
            )
            if stair_target
            else 0.0,
        }
    )

    support = _params(runtime, "mpc_critics")["terrain_support"]
    support.update(
        {
            "enabled": True,
            "fail_closed": True,
            "min_support_ratio": model["min_support_ratio"],
            "min_confidence": model["min_confidence"],
            "stairs_enabled": stair_target,
        }
    )
    _params(runtime, "local_planner")["goal_surface_match_required"] = True
    p2p = _params(runtime, "p2p_move_base")
    p2p["main_trajectory_generator"] = "differential_drive_terrain"
    p2p["terrain_supervisor_enabled"] = stair_target
    if stair_target:
        p2p["terrain_status_timeout_sec"] = model["max_age_s"]
        p2p["gait_monitor_timeout_sec"] = model["max_age_s"]
        p2p["stair_min_confidence"] = model["min_confidence"]
        p2p["stair_min_support_ratio"] = model["min_support_ratio"]
        p2p["stair_max_heading_error_rad"] = math.radians(
            stair["max_heading_error_deg"]
        )
        p2p["stair_max_lateral_error_m"] = stair["max_lateral_error_m"]
        p2p["stair_committed_max_yaw_rps"] = stair["max_committed_yaw_rps"]

    semantics = stair["riser_semantics"]
    collision = _params(runtime, "mpc_critics")["terrain_collision"]
    collision_semantics = collision["stair_semantics"]
    collision_semantics["enabled"] = stair_target
    if stair_target:
        common = {
            "expected_map_hash": map_hash,
            "max_snapshot_age_sec": semantics["max_snapshot_age_s"],
            "minimum_stair_confidence": stair["min_confidence"],
            "max_node_match_distance_m": semantics["max_node_match_distance_m"],
            "riser_plane_tolerance_m": semantics["riser_plane_tolerance_m"],
            "riser_lateral_tolerance_m": semantics["riser_lateral_tolerance_m"],
            "riser_vertical_tolerance_m": semantics["riser_vertical_tolerance_m"],
        }
        collision_semantics.update(common)
        for index, axis in enumerate("xyz"):
            collision_semantics[f"leg_envelope_min_{axis}_m"] = semantics[
                "leg_envelope_min_xyz"
            ][index]
            collision_semantics[f"leg_envelope_max_{axis}_m"] = semantics[
                "leg_envelope_max_xyz"
            ][index]
        for key in (
            "max_support_xy_distance_m",
            "min_body_clearance_m",
            "max_body_clearance_m",
        ):
            collision_semantics[key] = semantics[key]
        for node_name in ("perception_3d_local", "perception_3d_global"):
            params = _params(runtime, node_name)
            marking = params["lidar"]["stair_riser_marking"]
            marking.update(common)
            marking["enabled"] = True
            marking["surface_plane_tolerance_m"] = semantics[
                "surface_plane_tolerance_m"
            ]
            params["lidar"]["marking_minimum_height"] = semantics[
                "leg_envelope_min_xyz"
            ][2]
    return runtime


class TerrainSiteProfileTest(unittest.TestCase):
    def test_published_template_is_safe_disabled_and_not_ready(self):
        template = yaml.safe_load(DEFAULT_PROFILE.read_text(encoding="utf-8"))
        report = validate_profile(template, "safe-disabled")
        self.assertEqual(report.enabled_corridor_ids, ())
        self.assertIsNone(template["map"]["sha256"])
        self.assertIsNone(template["terrain_roi"]["min_xyz"])
        self.assertIsNone(template["terrain_roi"]["max_xyz"])
        placeholder = template["stair"]["corridors"][0]
        self.assertIsNone(placeholder["up_axis_xyz"])
        self.assertIsNone(placeholder["first_riser_center_xyz"])
        self.assertEqual(placeholder["corridor_polygon_xy"], [])
        self.assertEqual(placeholder["lower_landing"]["polygon_xy"], [])
        self.assertEqual(placeholder["upper_landing"]["polygon_xy"], [])
        for target in ("ramp", "stair-up", "stair-down"):
            with self.subTest(target=target), self.assertRaises(ProfileValidationError):
                validate_profile(template, target)

    def test_complete_measured_fixture_is_ready_for_each_target(self):
        profile = valid_profile()
        self.assertEqual(validate_profile(profile, "ramp").target, "ramp")
        self.assertEqual(
            validate_profile(profile, "stair-up").enabled_corridor_ids, (17,)
        )
        self.assertEqual(
            validate_profile(profile, "stair-down").enabled_corridor_ids, (17,)
        )

    def test_map_hash_mismatch_fails_closed(self):
        for path in ("terrain", "terrain_roi", "ramp_capability"):
            with self.subTest(path=path):
                profile = valid_profile()
                profile[path]["source_map_sha256"] = "b" * 64
                with self.assertRaisesRegex(ProfileValidationError, "does not match"):
                    validate_profile(profile, "ramp")
        profile = valid_profile()
        profile["stair"]["corridors"][0]["source_map_sha256"] = "b" * 64
        with self.assertRaisesRegex(ProfileValidationError, "does not match"):
            validate_profile(profile, "stair-up")

    def test_roi_must_be_ordered_and_contain_stair_geometry(self):
        profile = valid_profile()
        profile["terrain_roi"]["max_xyz"][0] = -2.0
        with self.assertRaisesRegex(ProfileValidationError, "min_x"):
            validate_profile(profile, "ramp")
        profile = valid_profile()
        profile["terrain_roi"]["max_xyz"][0] = 3.0
        with self.assertRaisesRegex(ProfileValidationError, "outside"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["terrain_roi"]["voxel_size_m"] = 0.06
        with self.assertRaisesRegex(ProfileValidationError, "must not exceed 0.05"):
            validate_profile(profile, "ramp")
        profile = valid_profile()
        profile["map"]["complete_ground_voxel_size_m"] = 0.05
        with self.assertRaisesRegex(ProfileValidationError, "must be smaller"):
            validate_profile(profile, "ramp")

    def test_ramp_limits_must_stay_inside_verified_capability(self):
        profile = valid_profile()
        profile["terrain"]["traversal"]["ramp"]["max_up_slope_deg"] = 13.0
        with self.assertRaisesRegex(ProfileValidationError, "exceeds verified"):
            validate_profile(profile, "ramp")
        profile = valid_profile()
        profile["terrain"]["model"]["min_support_ratio"] = 0.70
        with self.assertRaisesRegex(ProfileValidationError, "verified minimum"):
            validate_profile(profile, "ramp")
        profile = valid_profile()
        profile["terrain"]["traversal"]["max_roughness_m"] = 0.04
        with self.assertRaisesRegex(ProfileValidationError, "max_roughness"):
            validate_profile(profile, "ramp")
        profile = valid_profile()
        profile["terrain"]["traversal"]["ramp"]["max_up_speed_mps"] = 0.10
        with self.assertRaisesRegex(ProfileValidationError, "v_exec_min"):
            validate_profile(profile, "ramp")

    def test_support_sampling_cannot_skip_gaps(self):
        profile = valid_profile()
        profile["terrain"]["model"]["edge_sample_spacing_m"] = 0.11
        with self.assertRaisesRegex(ProfileValidationError, "unsupported gaps"):
            validate_profile(profile, "ramp")

    def test_generic_step_bypass_is_closed_for_both_test_stages(self):
        for target in ("ramp", "stair-up"):
            with self.subTest(target=target):
                profile = valid_profile()
                profile["terrain"]["traversal"]["generic"]["max_step_up_m"] = 0.01
                with self.assertRaisesRegex(ProfileValidationError, "must remain 0.0"):
                    validate_profile(profile, target)

    def test_corridor_polygon_axis_and_height_are_checked(self):
        profile = valid_profile()
        profile["stair"]["corridors"][0]["corridor_polygon_xy"] = [
            [0.5, -0.5],
            [3.5, 0.5],
            [3.5, -0.5],
            [0.5, 0.5],
        ]
        with self.assertRaisesRegex(ProfileValidationError, "self-intersecting"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["corridors"][0]["up_axis_xyz"] = [-1.0, 0.0, 0.0]
        with self.assertRaisesRegex(ProfileValidationError, "disagrees"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["corridors"][0]["riser_height_m"] = 0.10
        profile["stair"]["corridors"][0]["first_riser_center_xyz"][2] = 0.05
        with self.assertRaisesRegex(ProfileValidationError, "inconsistent"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["max_height_closure_error_m"] = 0.05
        with self.assertRaisesRegex(ProfileValidationError, "runtime closure tolerance"):
            validate_profile(profile, "stair-up")

    def test_first_riser_anchor_must_match_runtime_geometry(self):
        profile = valid_profile()
        profile["stair"]["corridors"][0]["first_riser_center_xyz"] = None
        with self.assertRaisesRegex(ProfileValidationError, "must be a list"):
            validate_profile(profile, "stair-up")

        profile = valid_profile()
        profile["stair"]["corridors"][0]["first_riser_center_xyz"][2] = 0.15
        with self.assertRaisesRegex(ProfileValidationError, "half a riser"):
            validate_profile(profile, "stair-up")

        profile = valid_profile()
        profile["stair"]["corridors"][0]["first_riser_center_xyz"][0] = 4.5
        with self.assertRaisesRegex(ProfileValidationError, "outside"):
            validate_profile(profile, "stair-up")

    def test_step_count_is_riser_count_and_top_tread_matches_upper_landing(self):
        profile = valid_profile()
        profile["stair"]["step_count_semantics"] = "tread_count"
        with self.assertRaisesRegex(ProfileValidationError, "number of risers"):
            validate_profile(profile, "stair-up")

        profile = valid_profile()
        profile["stair"]["highest_tread_semantics"] = "one_riser_below_landing"
        with self.assertRaisesRegex(ProfileValidationError, "level with"):
            validate_profile(profile, "stair-up")

        profile = valid_profile()
        profile["stair"]["corridors"][0]["upper_landing"]["center_xyz"][2] = 1.019
        validate_profile(profile, "stair-up")
        profile["stair"]["corridors"][0]["upper_landing"]["center_xyz"][2] = 1.021
        with self.assertRaisesRegex(ProfileValidationError, "step_count"):
            validate_profile(profile, "stair-up")

        profile = valid_profile()
        corridor = profile["stair"]["corridors"][0]
        profile["stair"]["max_riser_height_m"] = 0.32
        profile["stair"]["max_height_closure_error_m"] = 0.03
        corridor["riser_height_m"] = 0.30
        corridor["step_count"] = 3
        corridor["first_riser_center_xyz"][2] = 0.15
        corridor["upper_landing"]["center_xyz"][2] = 0.929
        validate_profile(profile, "stair-up")
        corridor["upper_landing"]["center_xyz"][2] = 0.931
        with self.assertRaisesRegex(ProfileValidationError, "step_count"):
            validate_profile(profile, "stair-up")

    def test_riser_semantics_require_verified_fail_closed_geometry(self):
        profile = valid_profile()
        profile["stair"]["riser_semantics"]["enabled"] = False
        with self.assertRaisesRegex(ProfileValidationError, "riser_semantics.enabled"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["riser_semantics"]["leg_envelope_max_xyz"][2] = 0.01
        with self.assertRaisesRegex(ProfileValidationError, "body-frame z=0"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["riser_semantics"]["max_snapshot_age_s"] = 0.21
        with self.assertRaisesRegex(ProfileValidationError, "terrain.model.max_age_s"):
            validate_profile(profile, "stair-up")

    def test_stair_direction_requires_normal_gait_evidence(self):
        profile = valid_profile()
        profile["stair"]["corridors"][0]["normal_gait_up_evidence_ids"] = []
        with self.assertRaisesRegex(ProfileValidationError, "traceable evidence"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["corridors"][0]["allow_down"] = False
        profile["stair"]["corridors"][0]["normal_gait_down_evidence_ids"] = []
        with self.assertRaisesRegex(ProfileValidationError, "allow_down"):
            validate_profile(profile, "stair-down")

    def test_stair_riser_and_tread_limits_cover_measured_geometry(self):
        profile = valid_profile()
        profile["stair"]["max_riser_height_m"] = 0.19
        with self.assertRaisesRegex(ProfileValidationError, "max_riser_height"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["min_tread_depth_m"] = 0.31
        with self.assertRaisesRegex(ProfileValidationError, "tread limits"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["max_riser_deviation_m"] = 0.10
        with self.assertRaisesRegex(ProfileValidationError, "half"):
            validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["stair"]["max_down_speed_mps"] = 0.10
        with self.assertRaisesRegex(ProfileValidationError, "v_exec_min"):
            validate_profile(profile, "stair-down")

    def test_gait_switch_or_missing_monitor_fails_closed(self):
        for key in ("allow_gait_change_requests",):
            profile = valid_profile()
            profile["stair"]["normal_gait_constraint"][key] = True
            with self.assertRaisesRegex(ProfileValidationError, "must remain false"):
                validate_profile(profile, "stair-up")
        profile = valid_profile()
        profile["runtime_activation"]["gait_monitor_enabled"] = False
        with self.assertRaisesRegex(ProfileValidationError, "gait_monitor"):
            validate_profile(profile, "stair-up")

    def test_every_runtime_safety_switch_is_required(self):
        for key in (
            "terrain_layers_enabled",
            "global_edge_policy_enabled",
            "terrain_trajectory_generator_enabled",
            "local_support_critic_enabled",
            "goal_surface_match_enabled",
            "terrain_command_gate_enabled",
        ):
            with self.subTest(key=key):
                profile = valid_profile()
                profile["runtime_activation"][key] = False
                with self.assertRaisesRegex(ProfileValidationError, key):
                    validate_profile(profile, "ramp")

        for key in (
            "stair_supervisor_enabled",
            "stair_riser_collision_semantics_enabled",
            "stair_riser_marking_enabled",
            "gait_monitor_enabled",
        ):
            with self.subTest(key=key):
                profile = valid_profile()
                profile["runtime_activation"][key] = False
                with self.assertRaisesRegex(ProfileValidationError, key):
                    validate_profile(profile, "stair-up")

    def test_safe_disabled_rejects_hidden_activation(self):
        template = yaml.safe_load(DEFAULT_PROFILE.read_text(encoding="utf-8"))
        for section, key in (
            ("terrain", "enabled"),
            ("terrain_roi", "enabled"),
            ("stair", "enabled"),
        ):
            with self.subTest(section=section):
                modified = copy.deepcopy(template)
                modified[section][key] = True
                with self.assertRaises(ProfileValidationError):
                    validate_profile(modified, "safe-disabled")

    def test_shipped_runtime_config_is_explicitly_safe_disabled(self):
        template = yaml.safe_load(DEFAULT_PROFILE.read_text(encoding="utf-8"))
        runtime = yaml.safe_load(RUNTIME_CONFIG.read_text(encoding="utf-8"))
        validate_runtime_config(template, runtime, "safe-disabled")
        runtime["global_planner"]["ros__parameters"]["terrain"]["enabled"] = True
        with self.assertRaisesRegex(ProfileValidationError, "global_planner"):
            validate_runtime_config(template, runtime, "safe-disabled")

    def test_runtime_config_closes_every_target_to_the_site_profile(self):
        for target in ("ramp", "stair-up", "stair-down"):
            with self.subTest(target=target):
                profile = valid_profile()
                runtime = valid_runtime_config(profile, target)
                validate_runtime_config(profile, runtime, target)

    def test_runtime_map_global_generator_support_and_goal_mismatches_fail(self):
        cases = (
            ("map hash", lambda runtime: _params(runtime, "map1").update(
                {"source_map_sha256": "b" * 64}
            )),
            ("roi", lambda runtime: _params(runtime, "map1").update(
                {"terrain_roi_max_x": 99.0}
            )),
            ("global terrain", lambda runtime: _params(runtime, "global_planner")[
                "terrain"
            ].update({"max_step_up_m": 0.01})),
            ("generator", lambda runtime: _params(runtime, "trajectory_generators")[
                "differential_drive_terrain"
            ].update({"terrain_enabled": False})),
            ("support", lambda runtime: _params(runtime, "mpc_critics")[
                "terrain_support"
            ].update({"enabled": False})),
            ("goal", lambda runtime: _params(runtime, "local_planner").update(
                {"goal_surface_match_required": False}
            )),
            ("p2p main", lambda runtime: _params(runtime, "p2p_move_base").update(
                {"main_trajectory_generator": "differential_drive_simple"}
            )),
        )
        for label, mutate in cases:
            with self.subTest(label=label):
                profile = valid_profile()
                runtime = valid_runtime_config(profile, "ramp")
                mutate(runtime)
                with self.assertRaises(ProfileValidationError):
                    validate_runtime_config(profile, runtime, "ramp")

    def test_runtime_stair_supervisor_collision_and_marking_mismatches_fail(self):
        cases = (
            lambda runtime: _params(runtime, "p2p_move_base").update(
                {"terrain_supervisor_enabled": False}
            ),
            lambda runtime: _params(runtime, "mpc_critics")["terrain_collision"][
                "stair_semantics"
            ].update({"expected_map_hash": "b" * 64}),
            lambda runtime: _params(runtime, "perception_3d_local")["lidar"][
                "stair_riser_marking"
            ].update({"enabled": False}),
            lambda runtime: _params(runtime, "perception_3d_global")["terrain"].update(
                {"stair_ids": []}
            ),
        )
        for mutate in cases:
            profile = valid_profile()
            runtime = valid_runtime_config(profile, "stair-up")
            mutate(runtime)
            with self.assertRaises(ProfileValidationError):
                validate_runtime_config(profile, runtime, "stair-up")

    def test_cli_requires_runtime_config_for_non_disabled_target(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as profile_file:
            yaml.safe_dump(valid_profile(), profile_file)
            profile_file.flush()
            output = io.StringIO()
            with patch.object(
                sys,
                "argv",
                [
                    "validate_go2_xt16_terrain_site_profile.py",
                    "--profile",
                    profile_file.name,
                    "--target",
                    "ramp",
                ],
            ), redirect_stdout(output):
                self.assertEqual(main(), 1)
            self.assertIn("--runtime-config", output.getvalue())


if __name__ == "__main__":
    unittest.main()
