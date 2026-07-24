#!/usr/bin/env python3
"""Fail closed when the Go2 navigation config can silently admit wall paths."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"
)

# The narrow-route Go2 profile keeps a 5 cm centerline margin beyond the
# configured 0.21 m trajectory half-width (0.155 m standing body half-width
# plus lateral allowance). The local collision critic still checks the
# complete oriented cuboid, including its longer fore/aft extent.
NARROW_ROUTE_CENTERLINE_MARGIN_M = 0.05


def require_mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def require_positive(mapping: dict[str, Any], key: str, path: str) -> float:
    value = mapping.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{path}.{key} must be positive, got {value!r}")
    return float(value)


def require_number(mapping: dict[str, Any], key: str, path: str) -> float:
    value = mapping.get(key)
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
    ):
        raise ValueError(f"{path}.{key} must be finite, got {value!r}")
    return float(value)


def robot_xy_radius(config: dict[str, Any]) -> float:
    local_planner = require_mapping(config.get("local_planner"), "local_planner")
    params = require_mapping(local_planner.get("ros__parameters"), "local_planner.ros__parameters")
    cuboid = require_mapping(params.get("cuboid"), "local_planner.ros__parameters.cuboid")

    radii: list[float] = []
    for name, vertex in cuboid.items():
        if not isinstance(vertex, list) or len(vertex) < 2:
            raise ValueError(f"local_planner.ros__parameters.cuboid.{name} must contain x and y")
        radii.append(math.hypot(float(vertex[0]), float(vertex[1])))
    if not radii:
        raise ValueError("local_planner.ros__parameters.cuboid must not be empty")
    return max(radii)


def robot_half_width(config: dict[str, Any]) -> float:
    local_planner = require_mapping(config.get("local_planner"), "local_planner")
    params = require_mapping(local_planner.get("ros__parameters"), "local_planner.ros__parameters")
    cuboid = require_mapping(params.get("cuboid"), "local_planner.ros__parameters.cuboid")
    return max(abs(float(vertex[1])) for vertex in cuboid.values())


def perception_params(config: dict[str, Any], node: str) -> dict[str, Any]:
    section = require_mapping(config.get(node), node)
    return require_mapping(section.get("ros__parameters"), f"{node}.ros__parameters")


def validate(config: dict[str, Any]) -> dict[str, float]:
    body_radius = robot_xy_radius(config)
    half_width = robot_half_width(config)
    minimum_centerline_clearance = half_width + NARROW_ROUTE_CENTERLINE_MARGIN_M
    report: dict[str, float] = {
        "robot_xy_radius": body_radius,
        "robot_half_width": half_width,
        "minimum_centerline_clearance": minimum_centerline_clearance,
    }

    for node in ("perception_3d_local", "perception_3d_global"):
        params = perception_params(config, node)
        inscribed = require_positive(params, "inscribed_radius", node)
        inflation = require_positive(params, "inflation_radius", node)
        if inscribed + 1e-9 < minimum_centerline_clearance:
            raise ValueError(
                f"{node}.inscribed_radius={inscribed:.3f} is smaller than "
                f"the robot half-width plus "
                f"{NARROW_ROUTE_CENTERLINE_MARGIN_M:.2f} m margin "
                f"({minimum_centerline_clearance:.3f})"
            )
        if inflation < inscribed:
            raise ValueError(
                f"{node}.inflation_radius={inflation:.3f} is smaller than "
                f"inscribed_radius={inscribed:.3f}"
            )

        lidar = require_mapping(params.get("lidar"), f"{node}.lidar")
        require_positive(lidar, "xy_resolution", f"{node}.lidar")
        require_positive(lidar, "height_resolution", f"{node}.lidar")
        if "resolution" in lidar:
            raise ValueError(
                f"{node}.lidar.resolution is not consumed by MultiLayerSpinningLidar; "
                "use xy_resolution"
            )

        report[f"{node}.inscribed_radius"] = inscribed
        report[f"{node}.inflation_radius"] = inflation

    global_map = require_mapping(
        perception_params(config, "perception_3d_global").get("map"),
        "perception_3d_global.map",
    )
    min_points = global_map.get("static_obstacle_min_points")
    if not isinstance(min_points, int) or isinstance(min_points, bool) or min_points < 1:
        raise ValueError(
            "perception_3d_global.map.static_obstacle_min_points must be a positive integer"
        )
    report["static_obstacle_min_points"] = float(min_points)
    xy_radius = require_positive(
        global_map,
        "static_obstacle_xy_radius",
        "perception_3d_global.map",
    )
    global_inscribed = report["perception_3d_global.inscribed_radius"]
    if xy_radius < global_inscribed:
        raise ValueError(
            "perception_3d_global.map.static_obstacle_xy_radius must be at least "
            "the global inscribed_radius"
        )
    report["static_obstacle_xy_radius"] = xy_radius

    p2p = require_mapping(config.get("p2p_move_base"), "p2p_move_base")
    p2p_params = require_mapping(
        p2p.get("ros__parameters"), "p2p_move_base.ros__parameters"
    )
    if p2p_params.get("rotate_recovery_enabled") is not False:
        raise ValueError(
            "p2p_move_base.rotate_recovery_enabled must be false for the "
            "narrow-gate Go2 profile"
        )
    report["rotate_recovery_enabled"] = 0.0
    main_generator = p2p_params.get("main_trajectory_generator")
    if main_generator != "omni_drive_simple":
        raise ValueError(
            "p2p_move_base.main_trajectory_generator must be omni_drive_simple "
            "for collision-checked Go2 lateral avoidance"
        )

    trajectory_generators = require_mapping(
        config.get("trajectory_generators"), "trajectory_generators"
    )
    trajectory_params = require_mapping(
        trajectory_generators.get("ros__parameters"),
        "trajectory_generators.ros__parameters",
    )
    trajectory_plugins = trajectory_params.get("plugins")
    if (
        not isinstance(trajectory_plugins, list)
        or "omni_drive_simple" not in trajectory_plugins
    ):
        raise ValueError(
            "trajectory_generators.plugins must include omni_drive_simple"
        )
    omni = require_mapping(
        trajectory_params.get("omni_drive_simple"),
        "trajectory_generators.omni_drive_simple",
    )
    if omni.get("plugin") != (
        "trajectory_generators::OmniSimpleTrajectoryGeneratorTheory"
    ):
        raise ValueError(
            "trajectory_generators.omni_drive_simple.plugin must select the "
            "omnidirectional generator"
        )
    min_y = require_number(
        omni, "min_vel_y", "trajectory_generators.omni_drive_simple"
    )
    max_y = require_positive(
        omni, "max_vel_y", "trajectory_generators.omni_drive_simple"
    )
    min_trans = require_positive(
        omni, "min_vel_trans", "trajectory_generators.omni_drive_simple"
    )
    max_trans = require_positive(
        omni, "max_vel_trans", "trajectory_generators.omni_drive_simple"
    )
    max_x = require_positive(
        omni, "max_vel_x", "trajectory_generators.omni_drive_simple"
    )
    sim_time = require_positive(
        omni, "sim_time", "trajectory_generators.omni_drive_simple"
    )
    acc_y = require_positive(
        omni, "acc_lim_y", "trajectory_generators.omni_drive_simple"
    )
    lateral_samples = require_positive(
        omni, "linear_y_sample", "trajectory_generators.omni_drive_simple"
    )
    if max_y > 0.20 + 1e-9:
        raise ValueError(
            "trajectory_generators.omni_drive_simple.max_vel_y exceeds the "
            "0.20 m/s first-field-test cap"
        )
    if not math.isclose(min_y, -max_y, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(
            "trajectory_generators.omni_drive_simple lateral limits must be symmetric"
        )
    if min_trans > max_y + 1e-9:
        raise ValueError(
            "trajectory_generators.omni_drive_simple.min_vel_trans must admit "
            "a pure lateral candidate at max_vel_y"
        )
    if max_trans + 1e-9 < math.hypot(max_x, max_y):
        raise ValueError(
            "trajectory_generators.omni_drive_simple.max_vel_trans must admit "
            "the configured maximum diagonal command"
        )
    if lateral_samples < 3.0:
        raise ValueError(
            "trajectory_generators.omni_drive_simple.linear_y_sample must "
            "cover left, zero, and right"
        )
    report["omni.max_vel_y"] = max_y
    report["omni.min_vel_y"] = min_y
    report["omni.min_vel_trans"] = min_trans
    report["omni.acc_lim_y"] = acc_y

    critics = require_mapping(config.get("mpc_critics"), "mpc_critics")
    critic_params = require_mapping(
        critics.get("ros__parameters"), "mpc_critics.ros__parameters"
    )
    plugins = critic_params.get("plugins")
    if not isinstance(plugins, list) or "route_corridor" not in plugins:
        raise ValueError("mpc_critics.plugins must include route_corridor")
    corridor = require_mapping(
        critic_params.get("route_corridor"), "mpc_critics.route_corridor"
    )
    if corridor.get("plugin") != "mpc_critics::RouteCorridorModel":
        raise ValueError(
            "mpc_critics.route_corridor.plugin must be "
            "mpc_critics::RouteCorridorModel"
        )
    if corridor.get("trajectory_generator") != main_generator:
        raise ValueError(
            "mpc_critics.route_corridor must guard the configured main "
            "trajectory generator"
        )
    for critic_name in (
        "collision",
        "route_corridor",
        "stick_path",
        "pure_pursuit",
        "toward_global_plan",
    ):
        critic = require_mapping(
            critic_params.get(critic_name), f"mpc_critics.{critic_name}"
        )
        if critic.get("trajectory_generator") != main_generator:
            raise ValueError(
                f"mpc_critics.{critic_name} must score {main_generator}"
            )
    corridor_xy = require_positive(
        corridor, "max_xy_distance", "mpc_critics.route_corridor"
    )
    corridor_z = require_positive(
        corridor, "max_z_distance", "mpc_critics.route_corridor"
    )
    if corridor.get("adaptive_xy_enabled") is not True:
        raise ValueError(
            "mpc_critics.route_corridor.adaptive_xy_enabled must be true"
        )
    if corridor.get("adaptive_requires_lateral_motion") is not True:
        raise ValueError(
            "mpc_critics.route_corridor adaptive relaxation must require "
            "body-frame lateral motion"
        )
    adaptive_xy = require_positive(
        corridor, "adaptive_max_xy_distance", "mpc_critics.route_corridor"
    )
    adaptive_clearance = require_positive(
        corridor,
        "adaptive_min_obstacle_clearance",
        "mpc_critics.route_corridor",
    )
    adaptive_ground_distance = require_positive(
        corridor,
        "adaptive_max_ground_distance",
        "mpc_critics.route_corridor",
    )
    required_lateral_horizon = max_y * sim_time
    if adaptive_xy + 1e-9 < required_lateral_horizon:
        raise ValueError(
            "mpc_critics.route_corridor.adaptive_max_xy_distance must admit "
            f"the lateral planning horizon ({required_lateral_horizon:.3f} m)"
        )
    if adaptive_xy > 0.35 + 1e-9:
        raise ValueError(
            "mpc_critics.route_corridor.adaptive_max_xy_distance exceeds the "
            "0.35 m first-field-test cap"
        )
    required_clearance = body_radius + NARROW_ROUTE_CENTERLINE_MARGIN_M
    if adaptive_clearance + 1e-9 < required_clearance:
        raise ValueError(
            "mpc_critics.route_corridor.adaptive_min_obstacle_clearance is "
            f"smaller than the robot XY radius plus margin ({required_clearance:.3f} m)"
        )
    if adaptive_ground_distance > corridor_xy + 1e-9:
        raise ValueError(
            "mpc_critics.route_corridor.adaptive_max_ground_distance must not "
            "exceed the hard XY corridor"
        )
    if corridor_xy > 0.25 + 1e-9:
        raise ValueError(
            "mpc_critics.route_corridor.max_xy_distance must be no greater "
            "than 0.25 m for the gate profile"
        )
    if corridor_z > 0.60 + 1e-9:
        raise ValueError(
            "mpc_critics.route_corridor.max_z_distance must be no greater "
            "than 0.60 m for the gate profile"
        )
    report["route_corridor.max_xy_distance"] = corridor_xy
    report["route_corridor.max_z_distance"] = corridor_z
    report["route_corridor.adaptive_max_xy_distance"] = adaptive_xy
    report["route_corridor.adaptive_min_obstacle_clearance"] = adaptive_clearance
    report["route_corridor.adaptive_max_ground_distance"] = adaptive_ground_distance

    dwa = require_mapping(
        config.get("dynamic_window_aware_global_planner"),
        "dynamic_window_aware_global_planner",
    )
    dwa_params = require_mapping(
        dwa.get("ros__parameters"),
        "dynamic_window_aware_global_planner.ros__parameters",
    )
    join_xy = require_positive(
        dwa_params, "max_path_join_xy", "dynamic_window_aware_global_planner"
    )
    join_z = require_positive(
        dwa_params, "max_path_join_z", "dynamic_window_aware_global_planner"
    )
    if join_xy > corridor_xy + 1e-9:
        raise ValueError(
            "dynamic_window_aware_global_planner.max_path_join_xy must not "
            "exceed the hard route corridor"
        )
    report["dwa.max_path_join_xy"] = join_xy
    report["dwa.max_path_join_z"] = join_z

    global_planner = require_mapping(
        config.get("global_planner"), "global_planner"
    )
    global_planner_params = require_mapping(
        global_planner.get("ros__parameters"),
        "global_planner.ros__parameters",
    )
    goal_projection_xy = require_positive(
        global_planner_params,
        "max_goal_projection_xy",
        "global_planner",
    )
    start_projection_z = require_positive(
        global_planner_params,
        "max_start_projection_z",
        "global_planner",
    )
    if corridor_z + 1e-9 < start_projection_z:
        raise ValueError(
            "mpc_critics.route_corridor.max_z_distance must cover the normal "
            "base_link-to-ground start projection"
        )
    if goal_projection_xy > 0.35 + 1e-9:
        raise ValueError(
            "global_planner.max_goal_projection_xy exceeds the 0.35 m "
            "cargo-delivery bound"
        )
    report["global_planner.max_goal_projection_xy"] = goal_projection_xy
    report["global_planner.max_start_projection_z"] = start_projection_z
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()

    try:
        loaded = yaml.safe_load(args.config.read_text(encoding="utf-8"))
        config = require_mapping(loaded, "root")
        report = validate(config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"GO2_XT16_WALL_SAFETY_CONFIG_STATUS=FAIL: {exc}")
        return 1

    print(f"CONFIG={args.config}")
    for key, value in report.items():
        print(f"{key}={value:.3f}")
    print("GO2_XT16_WALL_SAFETY_CONFIG_STATUS=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
