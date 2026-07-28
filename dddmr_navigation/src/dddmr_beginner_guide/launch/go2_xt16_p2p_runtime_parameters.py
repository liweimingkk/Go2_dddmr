#!/usr/bin/env python3

import atexit
import math
import os
import tempfile
from typing import Dict, Optional

import yaml


MAX_LOCAL_LIDAR_FRESHNESS_SEC = 0.35


def _finite_float(raw_value, label: str) -> float:
    value = float(raw_value)
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def _strict_bool(raw_value, label: str) -> bool:
    if isinstance(raw_value, bool):
        return raw_value
    normalized = str(raw_value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"{label} must be true or false")


def build_exact_runtime_parameters(
    local_lidar_expected_sensor_time_sec,
    omni_min_vel_y,
    omni_max_vel_y,
    p2p_goals_enabled,
) -> Dict[str, dict]:
    freshness = _finite_float(
        local_lidar_expected_sensor_time_sec,
        "local_lidar_expected_sensor_time_sec",
    )
    minimum_y = _finite_float(omni_min_vel_y, "omni_min_vel_y")
    maximum_y = _finite_float(omni_max_vel_y, "omni_max_vel_y")
    goals_enabled = _strict_bool(p2p_goals_enabled, "p2p_goals_enabled")

    if freshness <= 0.0 or freshness > MAX_LOCAL_LIDAR_FRESHNESS_SEC:
        raise ValueError(
            "local_lidar_expected_sensor_time_sec must be within "
            f"(0, {MAX_LOCAL_LIDAR_FRESHNESS_SEC}]"
        )
    if minimum_y > maximum_y:
        raise ValueError("omni_min_vel_y must not exceed omni_max_vel_y")

    # Foxy converts anonymous-node inline launch parameters into a /** rule.
    # P2PMoveBase hosts several named rclcpp::Node instances in one process,
    # whose exact YAML sections take precedence over that wildcard. Generate
    # exact absolute node rules so the safety values reach their consumers.
    return {
        "/perception_3d_local": {
            "ros__parameters": {
                "lidar.expected_sensor_time": freshness,
            },
        },
        "/trajectory_generators": {
            "ros__parameters": {
                "omni_drive_simple.min_vel_y": minimum_y,
                "omni_drive_simple.max_vel_y": maximum_y,
            },
        },
        "/p2p_move_base": {
            "ros__parameters": {
                "goals_enabled": goals_enabled,
            },
        },
    }


def write_exact_runtime_parameters(
    local_lidar_expected_sensor_time_sec,
    omni_min_vel_y,
    omni_max_vel_y,
    p2p_goals_enabled,
    *,
    directory: Optional[str] = None,
) -> str:
    parameters = build_exact_runtime_parameters(
        local_lidar_expected_sensor_time_sec,
        omni_min_vel_y,
        omni_max_vel_y,
        p2p_goals_enabled,
    )
    file_descriptor, path = tempfile.mkstemp(
        prefix="go2_xt16_p2p_runtime_",
        suffix=".yaml",
        dir=directory,
    )
    os.fchmod(file_descriptor, 0o600)
    with os.fdopen(file_descriptor, "w", encoding="utf-8") as output:
        yaml.safe_dump(parameters, output, sort_keys=False)

    def remove_runtime_file() -> None:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass

    atexit.register(remove_runtime_file)
    return path
