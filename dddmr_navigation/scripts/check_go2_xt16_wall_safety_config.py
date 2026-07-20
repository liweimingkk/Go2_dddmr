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


def require_mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def require_positive(mapping: dict[str, Any], key: str, path: str) -> float:
    value = mapping.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{path}.{key} must be positive, got {value!r}")
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
    minimum_centerline_clearance = half_width + 0.10
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
                f"the robot half-width plus 0.10 m margin "
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
