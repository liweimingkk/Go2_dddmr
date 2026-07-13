#!/usr/bin/env python3
"""Validate a Go2 ramp/stair site profile without starting ROS or motion.

The validator is intentionally fail-closed.  A target is reported ready only
when its map identity, high-resolution ROI, terrain limits, measured capability
evidence, runtime safety switches, and (for stairs) manual geometry all agree.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any, Iterable, Sequence

import yaml


DEFAULT_PROFILE = (
    Path(__file__).resolve().parents[1]
    / "src/dddmr_beginner_guide/config/"
    "go2_xt16_ramp_stair_site_profile.template.yaml"
)
TARGETS = ("safe-disabled", "ramp", "stair-up", "stair-down")
SHA256_PATTERN = re.compile(r"^[0-9a-fA-F]{64}$")
COMMIT_PATTERN = re.compile(r"^[0-9a-fA-F]{7,40}$")
EPSILON = 1.0e-9


class ProfileValidationError(ValueError):
    """Raised when a profile cannot safely satisfy the requested target."""


@dataclass(frozen=True)
class ValidationReport:
    target: str
    site_id: str | None
    map_sha256: str | None
    enabled_corridor_ids: tuple[int, ...]


@dataclass(frozen=True)
class _CorridorReport:
    corridor_id: int
    riser_height_m: float
    tread_depth_m: float
    maximum_surface_roughness_m: float
    allow_up: bool
    allow_down: bool


def _fail(path: str, message: str) -> None:
    raise ProfileValidationError(f"{path}: {message}")


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        _fail(path, "must be a mapping")
    return value


def _list(value: Any, path: str) -> list[Any]:
    if not isinstance(value, list):
        _fail(path, "must be a list")
    return value


def _boolean(mapping: dict[str, Any], key: str, path: str) -> bool:
    value = mapping.get(key)
    if not isinstance(value, bool):
        _fail(f"{path}.{key}", f"must be true or false, got {value!r}")
    return value


def _number(
    mapping: dict[str, Any],
    key: str,
    path: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    strictly_positive: bool = False,
) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f"{path}.{key}", f"must be a finite number, got {value!r}")
    result = float(value)
    if not math.isfinite(result):
        _fail(f"{path}.{key}", "must be finite")
    if strictly_positive and result <= 0.0:
        _fail(f"{path}.{key}", "must be greater than zero")
    if minimum is not None and result < minimum:
        _fail(f"{path}.{key}", f"must be at least {minimum}")
    if maximum is not None and result > maximum:
        _fail(f"{path}.{key}", f"must not exceed {maximum}")
    return result


def _integer(
    mapping: dict[str, Any],
    key: str,
    path: str,
    *,
    minimum: int | None = None,
) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        _fail(f"{path}.{key}", f"must be an integer, got {value!r}")
    if minimum is not None and value < minimum:
        _fail(f"{path}.{key}", f"must be at least {minimum}")
    return value


def _nonempty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        _fail(path, "must be a non-empty string")
    lowered = value.strip().lower()
    if lowered in {"unset", "todo", "tbd", "placeholder", "example"}:
        _fail(path, "still contains a placeholder value")
    return value.strip()


def _sha256(value: Any, path: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        _fail(path, "must be the 64-hex SHA256 of the real source map")
    return value.lower()


def _optional_sha256(value: Any, path: str) -> str | None:
    if value is None:
        return None
    return _sha256(value, path)


def _evidence_ids(value: Any, path: str, *, required: bool) -> tuple[str, ...]:
    entries = _list(value, path)
    result: list[str] = []
    for index, entry in enumerate(entries):
        result.append(_nonempty_string(entry, f"{path}[{index}]"))
    if required and not result:
        _fail(path, "must contain at least one traceable evidence identifier")
    if len(set(result)) != len(result):
        _fail(path, "must not contain duplicate evidence identifiers")
    return tuple(result)


def _vector(value: Any, path: str, length: int) -> tuple[float, ...]:
    entries = _list(value, path)
    if len(entries) != length:
        _fail(path, f"must contain exactly {length} numbers")
    result: list[float] = []
    for index, entry in enumerate(entries):
        if isinstance(entry, bool) or not isinstance(entry, (int, float)):
            _fail(f"{path}[{index}]", "must be a finite number")
        number = float(entry)
        if not math.isfinite(number):
            _fail(f"{path}[{index}]", "must be finite")
        result.append(number)
    return tuple(result)


def _cross(a: Sequence[float], b: Sequence[float], c: Sequence[float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Sequence[float], b: Sequence[float], p: Sequence[float]) -> bool:
    return (
        abs(_cross(a, b, p)) <= EPSILON
        and min(a[0], b[0]) - EPSILON <= p[0] <= max(a[0], b[0]) + EPSILON
        and min(a[1], b[1]) - EPSILON <= p[1] <= max(a[1], b[1]) + EPSILON
    )


def _segments_intersect(
    a: Sequence[float],
    b: Sequence[float],
    c: Sequence[float],
    d: Sequence[float],
) -> bool:
    ab_c = _cross(a, b, c)
    ab_d = _cross(a, b, d)
    cd_a = _cross(c, d, a)
    cd_b = _cross(c, d, b)
    if ((ab_c > EPSILON and ab_d < -EPSILON) or (ab_c < -EPSILON and ab_d > EPSILON)) and (
        (cd_a > EPSILON and cd_b < -EPSILON) or (cd_a < -EPSILON and cd_b > EPSILON)
    ):
        return True
    return (
        (abs(ab_c) <= EPSILON and _on_segment(a, b, c))
        or (abs(ab_d) <= EPSILON and _on_segment(a, b, d))
        or (abs(cd_a) <= EPSILON and _on_segment(c, d, a))
        or (abs(cd_b) <= EPSILON and _on_segment(c, d, b))
    )


def _polygon(value: Any, path: str) -> tuple[tuple[float, float], ...]:
    vertices_raw = _list(value, path)
    if len(vertices_raw) < 3:
        _fail(path, "must contain at least three measured XY vertices")
    vertices = tuple(
        _vector(vertex, f"{path}[{index}]", 2)
        for index, vertex in enumerate(vertices_raw)
    )
    if len(set(vertices)) != len(vertices):
        _fail(path, "must not contain duplicate vertices or a repeated closing vertex")
    edge_count = len(vertices)
    for first in range(edge_count):
        first_next = (first + 1) % edge_count
        for second in range(first + 1, edge_count):
            second_next = (second + 1) % edge_count
            if first == second or first_next == second or second_next == first:
                continue
            if first == 0 and second_next == 0:
                continue
            if _segments_intersect(
                vertices[first],
                vertices[first_next],
                vertices[second],
                vertices[second_next],
            ):
                _fail(path, "must be a simple, non-self-intersecting polygon")
    twice_area = sum(
        vertices[index][0] * vertices[(index + 1) % len(vertices)][1]
        - vertices[(index + 1) % len(vertices)][0] * vertices[index][1]
        for index in range(len(vertices))
    )
    if abs(twice_area) <= EPSILON:
        _fail(path, "must have non-zero area")
    return vertices


def _point_in_polygon(
    point: Sequence[float], polygon: Sequence[Sequence[float]]
) -> bool:
    inside = False
    previous = len(polygon) - 1
    for index, vertex in enumerate(polygon):
        prior = polygon[previous]
        if _on_segment(prior, vertex, point):
            return True
        crosses = (prior[1] > point[1]) != (vertex[1] > point[1])
        if crosses:
            crossing_x = (
                (vertex[0] - prior[0]) * (point[1] - prior[1])
                / (vertex[1] - prior[1])
                + prior[0]
            )
            if point[0] < crossing_x:
                inside = not inside
        previous = index
    return inside


def _inside_roi(
    point: Sequence[float],
    minimum: Sequence[float],
    maximum: Sequence[float],
    path: str,
) -> None:
    if any(
        coordinate < minimum[index] - EPSILON
        or coordinate > maximum[index] + EPSILON
        for index, coordinate in enumerate(point)
    ):
        _fail(path, "lies outside the configured terrain ROI")


def _same_hash(actual: str, expected: str, path: str) -> None:
    if actual != expected:
        _fail(path, "does not match map.sha256")


def _validate_model(model: dict[str, Any], path: str) -> dict[str, float]:
    normal_radius = _number(model, "normal_radius_m", path, strictly_positive=True)
    min_neighbors = _integer(model, "min_neighbors", path, minimum=3)
    plane_residual = _number(
        model, "max_plane_residual_m", path, strictly_positive=True
    )
    sample_spacing = _number(
        model, "edge_sample_spacing_m", path, strictly_positive=True
    )
    support_radius = _number(model, "support_radius_m", path, strictly_positive=True)
    min_support = _number(
        model, "min_support_ratio", path, minimum=0.0, maximum=1.0
    )
    min_confidence = _number(
        model, "min_confidence", path, minimum=0.0, maximum=1.0
    )
    max_unknown = _number(
        model, "max_unknown_ratio", path, minimum=0.0, maximum=1.0
    )
    max_age = _number(model, "max_age_s", path, strictly_positive=True)
    if sample_spacing > support_radius + EPSILON:
        _fail(
            f"{path}.edge_sample_spacing_m",
            "must not exceed support_radius_m or unsupported gaps can be skipped",
        )
    return {
        "normal_radius_m": normal_radius,
        "min_neighbors": float(min_neighbors),
        "max_plane_residual_m": plane_residual,
        "edge_sample_spacing_m": sample_spacing,
        "support_radius_m": support_radius,
        "min_support_ratio": min_support,
        "min_confidence": min_confidence,
        "max_unknown_ratio": max_unknown,
        "max_age_s": max_age,
    }


def _require_generic_steps_disabled(terrain: dict[str, Any]) -> None:
    traversal = _mapping(terrain.get("traversal"), "terrain.traversal")
    generic = _mapping(traversal.get("generic"), "terrain.traversal.generic")
    for key in ("max_step_up_m", "max_step_down_m"):
        if abs(_number(generic, key, "terrain.traversal.generic")) > EPSILON:
            _fail(
                f"terrain.traversal.generic.{key}",
                "must remain 0.0 during the ramp/stair test stage",
            )


def _validate_roi(
    roi: dict[str, Any],
    map_section: dict[str, Any],
    map_hash: str,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    path = "terrain_roi"
    if not _boolean(roi, "enabled", path):
        _fail(f"{path}.enabled", "must be true for a terrain test target")
    roi_hash = _sha256(roi.get("source_map_sha256"), f"{path}.source_map_sha256")
    _same_hash(roi_hash, map_hash, f"{path}.source_map_sha256")
    map_frame = _nonempty_string(map_section.get("frame_id"), "map.frame_id")
    roi_frame = _nonempty_string(roi.get("frame_id"), f"{path}.frame_id")
    if roi_frame != map_frame:
        _fail(f"{path}.frame_id", "must match map.frame_id")
    coarse_voxel = _number(
        map_section,
        "complete_ground_voxel_size_m",
        "map",
        strictly_positive=True,
    )
    roi_voxel = _number(
        roi,
        "voxel_size_m",
        path,
        strictly_positive=True,
        maximum=0.05,
    )
    if roi_voxel >= coarse_voxel - EPSILON:
        _fail(
            f"{path}.voxel_size_m",
            "must be smaller than map.complete_ground_voxel_size_m",
        )
    minimum = _vector(roi.get("min_xyz"), f"{path}.min_xyz", 3)
    maximum = _vector(roi.get("max_xyz"), f"{path}.max_xyz", 3)
    for index, axis in enumerate("xyz"):
        if minimum[index] >= maximum[index]:
            _fail(
                path,
                f"min_{axis} must be less than max_{axis}",
            )
    return minimum, maximum


def _validate_ramp(
    terrain: dict[str, Any],
    ramp_capability: dict[str, Any],
    map_hash: str,
    model_values: dict[str, float],
) -> None:
    traversal = _mapping(terrain.get("traversal"), "terrain.traversal")
    ramp = _mapping(traversal.get("ramp"), "terrain.traversal.ramp")
    if not _boolean(ramp, "enabled", "terrain.traversal.ramp"):
        _fail("terrain.traversal.ramp.enabled", "must be true for ramp readiness")
    if not _boolean(ramp_capability, "verified", "ramp_capability"):
        _fail("ramp_capability.verified", "must be true for ramp readiness")
    capability_hash = _sha256(
        ramp_capability.get("source_map_sha256"),
        "ramp_capability.source_map_sha256",
    )
    _same_hash(capability_hash, map_hash, "ramp_capability.source_map_sha256")
    _evidence_ids(
        ramp_capability.get("evidence_ids"),
        "ramp_capability.evidence_ids",
        required=True,
    )
    planned_roughness = _number(
        traversal, "max_roughness_m", "terrain.traversal", strictly_positive=True
    )
    planned_normal_change = _number(
        traversal,
        "max_normal_change_deg",
        "terrain.traversal",
        strictly_positive=True,
        maximum=180.0,
    )
    planned_body_limits = {
        "max_body_roll_deg": _number(
            traversal,
            "max_body_roll_deg",
            "terrain.traversal",
            strictly_positive=True,
            maximum=90.0,
        ),
        "max_body_pitch_deg": _number(
            traversal,
            "max_body_pitch_deg",
            "terrain.traversal",
            strictly_positive=True,
            maximum=90.0,
        ),
    }
    capabilities = {
        "max_up_slope_deg": _number(
            ramp_capability,
            "max_up_slope_deg",
            "ramp_capability",
            strictly_positive=True,
            maximum=90.0,
        ),
        "max_down_slope_deg": _number(
            ramp_capability,
            "max_down_slope_deg",
            "ramp_capability",
            strictly_positive=True,
            maximum=90.0,
        ),
        "max_cross_slope_deg": _number(
            ramp_capability,
            "max_cross_slope_deg",
            "ramp_capability",
            strictly_positive=True,
            maximum=90.0,
        ),
        "max_roughness_m": _number(
            ramp_capability,
            "max_roughness_m",
            "ramp_capability",
            strictly_positive=True,
        ),
        "max_normal_change_deg": _number(
            ramp_capability,
            "max_normal_change_deg",
            "ramp_capability",
            strictly_positive=True,
            maximum=180.0,
        ),
        "max_body_roll_deg": _number(
            ramp_capability,
            "max_body_roll_deg",
            "ramp_capability",
            strictly_positive=True,
            maximum=90.0,
        ),
        "max_body_pitch_deg": _number(
            ramp_capability,
            "max_body_pitch_deg",
            "ramp_capability",
            strictly_positive=True,
            maximum=90.0,
        ),
    }
    for key, capability_limit in capabilities.items():
        if key == "max_roughness_m":
            planned_limit = planned_roughness
            planned_path = "terrain.traversal.max_roughness_m"
        elif key == "max_normal_change_deg":
            planned_limit = planned_normal_change
            planned_path = "terrain.traversal.max_normal_change_deg"
        elif key in planned_body_limits:
            planned_limit = planned_body_limits[key]
            planned_path = f"terrain.traversal.{key}"
        else:
            planned_limit = _number(
                ramp,
                key,
                "terrain.traversal.ramp",
                strictly_positive=True,
                maximum=90.0,
            )
            planned_path = f"terrain.traversal.ramp.{key}"
        if planned_limit > capability_limit + EPSILON:
            _fail(planned_path, f"exceeds verified ramp_capability.{key}")
    capability_support = _number(
        ramp_capability,
        "minimum_observed_support_ratio",
        "ramp_capability",
        minimum=0.0,
        maximum=1.0,
    )
    if model_values["min_support_ratio"] + EPSILON < capability_support:
        _fail(
            "terrain.model.min_support_ratio",
            "must be at least the verified minimum observed support ratio",
        )
    v_exec_min = _number(
        ramp_capability, "v_exec_min_mps", "ramp_capability",
        strictly_positive=True,
    )
    for direction in ("up", "down"):
        key = f"max_{direction}_speed_mps"
        verified_speed = _number(
            ramp_capability, key, "ramp_capability", strictly_positive=True
        )
        planned_speed = _number(
            ramp, key, "terrain.traversal.ramp", strictly_positive=True
        )
        if planned_speed + EPSILON < v_exec_min:
            _fail(
                f"terrain.traversal.ramp.{key}",
                "is below ramp_capability.v_exec_min_mps and may command no motion",
            )
        if planned_speed > verified_speed + EPSILON:
            _fail(
                f"terrain.traversal.ramp.{key}",
                f"exceeds verified ramp_capability.{key}",
            )
    planned_yaw = _number(
        ramp, "max_yaw_rps", "terrain.traversal.ramp", minimum=0.0
    )
    verified_yaw = _number(
        ramp_capability, "max_yaw_rps", "ramp_capability", minimum=0.0
    )
    if planned_yaw > verified_yaw + EPSILON:
        _fail(
            "terrain.traversal.ramp.max_yaw_rps",
            "exceeds verified ramp_capability.max_yaw_rps",
        )


def _validate_normal_gait(stair: dict[str, Any]) -> None:
    path = "stair.normal_gait_constraint"
    gait = _mapping(stair.get("normal_gait_constraint"), path)
    if not _boolean(gait, "enabled", path):
        _fail(f"{path}.enabled", "must be true for stair readiness")
    policy = _nonempty_string(gait.get("policy"), f"{path}.policy")
    if policy != "latch_normal_navigation_gait_at_test_start":
        _fail(
            f"{path}.policy",
            "must latch the existing normal navigation gait at test start",
        )
    if _boolean(gait, "allow_gait_change_requests", path):
        _fail(f"{path}.allow_gait_change_requests", "must remain false")
    for key in ("require_state_monitor", "require_sport_request_audit"):
        if not _boolean(gait, key, path):
            _fail(f"{path}.{key}", "must remain true")
    expected = gait.get("expected_gait_type")
    if expected is not None and (
        isinstance(expected, bool) or not isinstance(expected, int) or expected < 0
    ):
        _fail(
            f"{path}.expected_gait_type",
            "must be null or a non-negative observed gait type",
        )


def _validate_riser_semantics(
    stair: dict[str, Any], model_values: dict[str, float]
) -> dict[str, Any]:
    path = "stair.riser_semantics"
    semantics = _mapping(stair.get("riser_semantics"), path)
    if not _boolean(semantics, "enabled", path):
        _fail(f"{path}.enabled", "must be true for stair readiness")
    max_age = _number(
        semantics, "max_snapshot_age_s", path, strictly_positive=True
    )
    if max_age > model_values["max_age_s"] + EPSILON:
        _fail(
            f"{path}.max_snapshot_age_s",
            "must not exceed terrain.model.max_age_s",
        )
    _number(
        semantics, "max_node_match_distance_m", path, strictly_positive=True
    )
    _number(
        semantics, "riser_plane_tolerance_m", path, strictly_positive=True
    )
    _number(
        semantics, "riser_lateral_tolerance_m", path, minimum=0.0
    )
    _number(
        semantics, "riser_vertical_tolerance_m", path, strictly_positive=True
    )
    _number(
        semantics, "surface_plane_tolerance_m", path, strictly_positive=True
    )
    envelope_min = _vector(
        semantics.get("leg_envelope_min_xyz"),
        f"{path}.leg_envelope_min_xyz",
        3,
    )
    envelope_max = _vector(
        semantics.get("leg_envelope_max_xyz"),
        f"{path}.leg_envelope_max_xyz",
        3,
    )
    if any(
        envelope_max[index] <= envelope_min[index] + EPSILON
        for index in range(3)
    ):
        _fail(path, "leg pass-through envelope must have positive extent on every axis")
    if envelope_max[2] > EPSILON:
        _fail(
            f"{path}.leg_envelope_max_xyz",
            "z must not extend above body-frame z=0",
        )
    _number(
        semantics, "max_support_xy_distance_m", path, strictly_positive=True
    )
    min_clearance = _number(
        semantics, "min_body_clearance_m", path, minimum=0.0
    )
    max_clearance = _number(
        semantics, "max_body_clearance_m", path, strictly_positive=True
    )
    if max_clearance <= min_clearance + EPSILON:
        _fail(
            f"{path}.max_body_clearance_m",
            "must exceed min_body_clearance_m",
        )
    return semantics


def _validate_corridor(
    corridor: dict[str, Any],
    index: int,
    map_hash: str,
    capability_version: str,
    roi_minimum: Sequence[float],
    roi_maximum: Sequence[float],
    stair: dict[str, Any],
    model_values: dict[str, float],
) -> _CorridorReport:
    path = f"stair.corridors[{index}]"
    corridor_id = _integer(corridor, "id", path, minimum=0)
    corridor_hash = _sha256(
        corridor.get("source_map_sha256"), f"{path}.source_map_sha256"
    )
    _same_hash(corridor_hash, map_hash, f"{path}.source_map_sha256")
    corridor_version = _nonempty_string(
        corridor.get("capability_profile_version"),
        f"{path}.capability_profile_version",
    )
    if corridor_version != capability_version:
        _fail(
            f"{path}.capability_profile_version",
            "must match profile.capability_profile_version",
        )

    axis = _vector(corridor.get("up_axis_xyz"), f"{path}.up_axis_xyz", 3)
    horizontal_norm = math.hypot(axis[0], axis[1])
    if horizontal_norm <= EPSILON:
        _fail(f"{path}.up_axis_xyz", "must have a horizontal ascending direction")
    if abs(horizontal_norm - 1.0) > 0.05 or abs(axis[2]) > 0.05:
        _fail(
            f"{path}.up_axis_xyz",
            "must be a unit horizontal axis in the map frame (z approximately zero)",
        )

    corridor_polygon = _polygon(
        corridor.get("corridor_polygon_xy"), f"{path}.corridor_polygon_xy"
    )
    lower = _mapping(corridor.get("lower_landing"), f"{path}.lower_landing")
    upper = _mapping(corridor.get("upper_landing"), f"{path}.upper_landing")
    lower_center = _vector(
        lower.get("center_xyz"), f"{path}.lower_landing.center_xyz", 3
    )
    upper_center = _vector(
        upper.get("center_xyz"), f"{path}.upper_landing.center_xyz", 3
    )
    first_riser_center = _vector(
        corridor.get("first_riser_center_xyz"),
        f"{path}.first_riser_center_xyz",
        3,
    )
    lower_polygon = _polygon(
        lower.get("polygon_xy"), f"{path}.lower_landing.polygon_xy"
    )
    upper_polygon = _polygon(
        upper.get("polygon_xy"), f"{path}.upper_landing.polygon_xy"
    )
    if not _point_in_polygon(lower_center[:2], lower_polygon):
        _fail(f"{path}.lower_landing.center_xyz", "is outside its landing polygon")
    if not _point_in_polygon(upper_center[:2], upper_polygon):
        _fail(f"{path}.upper_landing.center_xyz", "is outside its landing polygon")
    if upper_center[2] <= lower_center[2] + EPSILON:
        _fail(path, "upper landing must be above lower landing")
    horizontal_delta = (
        upper_center[0] - lower_center[0],
        upper_center[1] - lower_center[1],
    )
    if horizontal_delta[0] * axis[0] + horizontal_delta[1] * axis[1] <= 0.0:
        _fail(path, "landing order disagrees with up_axis_xyz")

    for point_path, point in (
        (f"{path}.lower_landing.center_xyz", lower_center),
        (f"{path}.upper_landing.center_xyz", upper_center),
        (f"{path}.first_riser_center_xyz", first_riser_center),
    ):
        _inside_roi(point, roi_minimum, roi_maximum, point_path)
    for polygon_path, polygon, height in (
        (f"{path}.corridor_polygon_xy", corridor_polygon, lower_center[2]),
        (f"{path}.lower_landing.polygon_xy", lower_polygon, lower_center[2]),
        (f"{path}.upper_landing.polygon_xy", upper_polygon, upper_center[2]),
    ):
        for vertex_index, vertex in enumerate(polygon):
            _inside_roi(
                (vertex[0], vertex[1], height),
                roi_minimum,
                roi_maximum,
                f"{polygon_path}[{vertex_index}]",
            )

    _number(corridor, "width_m", path, strictly_positive=True)
    riser = _number(corridor, "riser_height_m", path, strictly_positive=True)
    tread = _number(corridor, "tread_depth_m", path, strictly_positive=True)
    steps = _integer(corridor, "step_count", path, minimum=1)
    first_riser_height_tolerance = max(0.02, 0.10 * riser)
    expected_first_riser_z = lower_center[2] + 0.5 * riser
    if abs(first_riser_center[2] - expected_first_riser_z) > (
        first_riser_height_tolerance + EPSILON
    ):
        _fail(
            f"{path}.first_riser_center_xyz",
            "z must match lower landing plus half a riser within the runtime tolerance",
        )
    if not _point_in_polygon(first_riser_center[:2], corridor_polygon):
        _fail(f"{path}.first_riser_center_xyz", "is outside the stair corridor")
    axis_xy = (axis[0] / horizontal_norm, axis[1] / horizontal_norm)
    first_from_lower = (
        (first_riser_center[0] - lower_center[0]) * axis_xy[0]
        + (first_riser_center[1] - lower_center[1]) * axis_xy[1]
    )
    upper_from_first = (
        (upper_center[0] - first_riser_center[0]) * axis_xy[0]
        + (upper_center[1] - first_riser_center[1]) * axis_xy[1]
    )
    if first_from_lower <= EPSILON or upper_from_first <= EPSILON:
        _fail(
            f"{path}.first_riser_center_xyz",
            "must lie between the landings along up_axis_xyz",
        )
    for step in range(steps):
        riser_xy = (
            first_riser_center[0] + step * tread * axis_xy[0],
            first_riser_center[1] + step * tread * axis_xy[1],
        )
        if not _point_in_polygon(riser_xy, corridor_polygon):
            _fail(path, "an expected riser center lies outside corridor_polygon_xy")
        remaining = (
            (upper_center[0] - riser_xy[0]) * axis_xy[0]
            + (upper_center[1] - riser_xy[1]) * axis_xy[1]
        )
        if remaining <= EPSILON:
            _fail(path, "an expected riser extends beyond the upper landing")
    confidence = _number(corridor, "confidence", path, minimum=0.0, maximum=1.0)
    min_confidence = _number(
        stair, "min_confidence", "stair", minimum=0.0, maximum=1.0
    )
    if confidence + EPSILON < min_confidence:
        _fail(f"{path}.confidence", "is below stair.min_confidence")
    roughness = _number(
        corridor, "maximum_surface_roughness_m", path, strictly_positive=True
    )
    support = _number(
        corridor,
        "minimum_observed_support_ratio",
        path,
        minimum=0.0,
        maximum=1.0,
    )
    if model_values["min_support_ratio"] + EPSILON < support:
        _fail(
            "terrain.model.min_support_ratio",
            f"is below {path}.minimum_observed_support_ratio",
        )
    closure_error = abs((upper_center[2] - lower_center[2]) - steps * riser)
    closure_limit = _number(
        stair, "max_height_closure_error_m", "stair", strictly_positive=True
    )
    runtime_closure_limit = max(0.02, 0.10 * riser)
    if closure_limit > runtime_closure_limit + EPSILON:
        _fail(
            "stair.max_height_closure_error_m",
            "exceeds the TerrainSnapshot runtime closure tolerance "
            f"({runtime_closure_limit:.6g} m for this riser)",
        )
    if closure_error > closure_limit + EPSILON:
        _fail(
            path,
            "landing height change is inconsistent with step_count * riser_height_m",
        )
    _evidence_ids(
        corridor.get("geometry_evidence_ids"),
        f"{path}.geometry_evidence_ids",
        required=True,
    )
    allow_up = _boolean(corridor, "allow_up", path)
    allow_down = _boolean(corridor, "allow_down", path)
    if not allow_up and not allow_down:
        _fail(path, "must explicitly allow at least one evidenced direction")
    _evidence_ids(
        corridor.get("normal_gait_up_evidence_ids"),
        f"{path}.normal_gait_up_evidence_ids",
        required=allow_up,
    )
    _evidence_ids(
        corridor.get("normal_gait_down_evidence_ids"),
        f"{path}.normal_gait_down_evidence_ids",
        required=allow_down,
    )
    return _CorridorReport(
        corridor_id=corridor_id,
        riser_height_m=riser,
        tread_depth_m=tread,
        maximum_surface_roughness_m=roughness,
        allow_up=allow_up,
        allow_down=allow_down,
    )


def _require_activation(
    activation: dict[str, Any], target: str, stair: dict[str, Any]
) -> None:
    core_keys = (
        "terrain_layers_enabled",
        "global_edge_policy_enabled",
        "terrain_trajectory_generator_enabled",
        "local_support_critic_enabled",
        "goal_surface_match_enabled",
        "terrain_command_gate_enabled",
    )
    for key in core_keys:
        if not _boolean(activation, key, "runtime_activation"):
            _fail(f"runtime_activation.{key}", f"must be true for {target} readiness")
    if target.startswith("stair"):
        for key in (
            "stair_supervisor_enabled",
            "stair_riser_collision_semantics_enabled",
            "stair_riser_marking_enabled",
            "gait_monitor_enabled",
        ):
            if not _boolean(activation, key, "runtime_activation"):
                _fail(
                    f"runtime_activation.{key}",
                    f"must be true for {target} readiness",
                )
        _validate_normal_gait(stair)


def _validate_safe_disabled(config: dict[str, Any]) -> ValidationReport:
    profile = _mapping(config.get("profile"), "profile")
    terrain_roi = _mapping(config.get("terrain_roi"), "terrain_roi")
    terrain = _mapping(config.get("terrain"), "terrain")
    traversal = _mapping(terrain.get("traversal"), "terrain.traversal")
    ramp = _mapping(traversal.get("ramp"), "terrain.traversal.ramp")
    stair = _mapping(config.get("stair"), "stair")
    gait = _mapping(stair.get("normal_gait_constraint"), "stair.normal_gait_constraint")
    riser_semantics = _mapping(stair.get("riser_semantics"), "stair.riser_semantics")
    activation = _mapping(config.get("runtime_activation"), "runtime_activation")
    switches: Iterable[tuple[str, bool]] = (
        ("profile.enabled", _boolean(profile, "enabled", "profile")),
        ("terrain_roi.enabled", _boolean(terrain_roi, "enabled", "terrain_roi")),
        ("terrain.enabled", _boolean(terrain, "enabled", "terrain")),
        (
            "terrain.traversal.ramp.enabled",
            _boolean(ramp, "enabled", "terrain.traversal.ramp"),
        ),
        ("stair.enabled", _boolean(stair, "enabled", "stair")),
        (
            "stair.normal_gait_constraint.enabled",
            _boolean(gait, "enabled", "stair.normal_gait_constraint"),
        ),
        (
            "stair.riser_semantics.enabled",
            _boolean(riser_semantics, "enabled", "stair.riser_semantics"),
        ),
    )
    for path, enabled in switches:
        if enabled:
            _fail(path, "must remain false in the safe-disabled target")
    for key in (
        "terrain_layers_enabled",
        "global_edge_policy_enabled",
        "terrain_trajectory_generator_enabled",
        "local_support_critic_enabled",
        "goal_surface_match_enabled",
        "stair_supervisor_enabled",
        "stair_riser_collision_semantics_enabled",
        "stair_riser_marking_enabled",
        "terrain_command_gate_enabled",
        "gait_monitor_enabled",
    ):
        if _boolean(activation, key, "runtime_activation"):
            _fail(f"runtime_activation.{key}", "must remain false")
    if not _boolean(terrain, "fail_closed", "terrain"):
        _fail("terrain.fail_closed", "must remain true even while disabled")
    if _boolean(gait, "allow_gait_change_requests", "stair.normal_gait_constraint"):
        _fail(
            "stair.normal_gait_constraint.allow_gait_change_requests",
            "must remain false",
        )
    corridors = _list(stair.get("corridors"), "stair.corridors")
    for index, entry in enumerate(corridors):
        corridor = _mapping(entry, f"stair.corridors[{index}]")
        if _boolean(corridor, "enabled", f"stair.corridors[{index}]"):
            _fail(f"stair.corridors[{index}].enabled", "must remain false")
        for key in ("allow_up", "allow_down"):
            if _boolean(corridor, key, f"stair.corridors[{index}]"):
                _fail(f"stair.corridors[{index}].{key}", "must remain false")
    _validate_model(_mapping(terrain.get("model"), "terrain.model"), "terrain.model")
    map_section = _mapping(config.get("map"), "map")
    return ValidationReport(
        target="safe-disabled",
        site_id=profile.get("site_id") if isinstance(profile.get("site_id"), str) else None,
        map_sha256=_optional_sha256(map_section.get("sha256"), "map.sha256"),
        enabled_corridor_ids=(),
    )


def validate_profile(config: dict[str, Any], target: str) -> ValidationReport:
    """Validate *config* for one of TARGETS and return a compact report."""
    if target not in TARGETS:
        _fail("target", f"must be one of {', '.join(TARGETS)}")
    schema_version = config.get("schema_version")
    if isinstance(schema_version, bool) or schema_version != 1:
        _fail("schema_version", "must be integer 1")
    if target == "safe-disabled":
        return _validate_safe_disabled(config)

    profile = _mapping(config.get("profile"), "profile")
    if not _boolean(profile, "enabled", "profile"):
        _fail("profile.enabled", f"must be true for {target} readiness")
    site_id = _nonempty_string(profile.get("site_id"), "profile.site_id")
    capability_version = _nonempty_string(
        profile.get("capability_profile_version"),
        "profile.capability_profile_version",
    )
    software_commit = _nonempty_string(
        profile.get("software_commit"), "profile.software_commit"
    )
    if COMMIT_PATTERN.fullmatch(software_commit) is None:
        _fail("profile.software_commit", "must be a 7-40 hex Git commit")

    map_section = _mapping(config.get("map"), "map")
    map_hash = _sha256(map_section.get("sha256"), "map.sha256")
    roi = _mapping(config.get("terrain_roi"), "terrain_roi")
    roi_minimum, roi_maximum = _validate_roi(roi, map_section, map_hash)

    terrain = _mapping(config.get("terrain"), "terrain")
    if not _boolean(terrain, "enabled", "terrain"):
        _fail("terrain.enabled", f"must be true for {target} readiness")
    if not _boolean(terrain, "fail_closed", "terrain"):
        _fail("terrain.fail_closed", "must be true")
    terrain_hash = _sha256(
        terrain.get("source_map_sha256"), "terrain.source_map_sha256"
    )
    _same_hash(terrain_hash, map_hash, "terrain.source_map_sha256")
    model_values = _validate_model(
        _mapping(terrain.get("model"), "terrain.model"), "terrain.model"
    )
    if model_values["max_unknown_ratio"] > EPSILON:
        _fail(
            "terrain.model.max_unknown_ratio",
            "must remain 0.0 during the ramp/stair test stage",
        )
    _require_generic_steps_disabled(terrain)

    stair = _mapping(config.get("stair"), "stair")
    activation = _mapping(config.get("runtime_activation"), "runtime_activation")
    _require_activation(activation, target, stair)

    enabled_corridor_ids: list[int] = []
    if target == "ramp":
        _validate_ramp(
            terrain,
            _mapping(config.get("ramp_capability"), "ramp_capability"),
            map_hash,
            model_values,
        )
    else:
        if not _boolean(stair, "enabled", "stair"):
            _fail("stair.enabled", f"must be true for {target} readiness")
        step_semantics = _nonempty_string(
            stair.get("step_count_semantics"), "stair.step_count_semantics"
        )
        if step_semantics != "riser_count":
            _fail(
                "stair.step_count_semantics",
                "must state that step_count is the number of risers",
            )
        highest_tread_semantics = _nonempty_string(
            stair.get("highest_tread_semantics"),
            "stair.highest_tread_semantics",
        )
        if highest_tread_semantics != "level_with_upper_landing":
            _fail(
                "stair.highest_tread_semantics",
                "must state that the highest tread is level with the upper landing",
            )
        _validate_riser_semantics(stair, model_values)
        for key in ("require_manual_corridor", "require_online_confirmation"):
            if not _boolean(stair, key, "stair"):
                _fail(f"stair.{key}", "must remain true")
        if _boolean(stair, "allow_in_place_rotation", "stair"):
            _fail("stair.allow_in_place_rotation", "must remain false")
        if _boolean(stair, "allow_replanning_while_committed", "stair"):
            _fail("stair.allow_replanning_while_committed", "must remain false")
        if _integer(stair, "max_step_index_delta", "stair", minimum=1) != 1:
            _fail("stair.max_step_index_delta", "must equal 1")
        _number(stair, "max_heading_error_deg", "stair", strictly_positive=True)
        _number(stair, "max_lateral_error_m", "stair", strictly_positive=True)
        stair_body_roll = _number(
            stair, "max_body_roll_deg", "stair",
            strictly_positive=True, maximum=90.0,
        )
        stair_body_pitch = _number(
            stair, "max_body_pitch_deg", "stair",
            strictly_positive=True, maximum=90.0,
        )
        traversal = _mapping(terrain.get("traversal"), "terrain.traversal")
        for key, stair_limit in (
            ("max_body_roll_deg", stair_body_roll),
            ("max_body_pitch_deg", stair_body_pitch),
        ):
            planned_limit = _number(
                traversal, key, "terrain.traversal",
                strictly_positive=True, maximum=90.0,
            )
            if planned_limit > stair_limit + EPSILON:
                _fail(
                    f"terrain.traversal.{key}",
                    f"exceeds stair.{key}",
                )
        planned_roughness = _number(
            traversal,
            "max_roughness_m",
            "terrain.traversal",
            strictly_positive=True,
        )
        _number(
            traversal,
            "max_normal_change_deg",
            "terrain.traversal",
            strictly_positive=True,
            maximum=180.0,
        )
        max_riser = _number(
            stair,
            "max_riser_height_m",
            "stair",
            strictly_positive=True,
        )
        min_tread = _number(
            stair,
            "min_tread_depth_m",
            "stair",
            strictly_positive=True,
        )
        max_tread = _number(
            stair,
            "max_tread_depth_m",
            "stair",
            strictly_positive=True,
        )
        if min_tread > max_tread + EPSILON:
            _fail(
                "stair.min_tread_depth_m",
                "must not exceed stair.max_tread_depth_m",
            )
        max_riser_deviation = _number(
            stair,
            "max_riser_deviation_m",
            "stair",
            strictly_positive=True,
        )
        max_tread_deviation = _number(
            stair,
            "max_tread_deviation_m",
            "stair",
            strictly_positive=True,
        )
        stair_v_exec_min = _number(
            stair, "v_exec_min_mps", "stair", strictly_positive=True
        )
        stair_up_speed = _number(
            stair, "max_up_speed_mps", "stair", strictly_positive=True
        )
        stair_down_speed = _number(
            stair, "max_down_speed_mps", "stair", strictly_positive=True
        )
        _number(
            stair, "max_committed_yaw_rps", "stair", minimum=0.0
        )
        target_speed = stair_up_speed if target == "stair-up" else stair_down_speed
        if target_speed + EPSILON < stair_v_exec_min:
            _fail(
                f"stair.max_{'up' if target == 'stair-up' else 'down'}_speed_mps",
                "is below stair.v_exec_min_mps and may command no motion",
            )
        corridors = _list(stair.get("corridors"), "stair.corridors")
        seen_ids: set[int] = set()
        for index, entry in enumerate(corridors):
            corridor = _mapping(entry, f"stair.corridors[{index}]")
            if not _boolean(corridor, "enabled", f"stair.corridors[{index}]"):
                continue
            corridor_report = _validate_corridor(
                corridor,
                index,
                map_hash,
                capability_version,
                roi_minimum,
                roi_maximum,
                stair,
                model_values,
            )
            if (
                planned_roughness
                > corridor_report.maximum_surface_roughness_m + EPSILON
            ):
                _fail(
                    "terrain.traversal.max_roughness_m",
                    f"exceeds stair.corridors[{index}].maximum_surface_roughness_m",
                )
            if corridor_report.riser_height_m > max_riser + EPSILON:
                _fail(
                    f"stair.corridors[{index}].riser_height_m",
                    "exceeds stair.max_riser_height_m",
                )
            if not (
                min_tread - EPSILON
                <= corridor_report.tread_depth_m
                <= max_tread + EPSILON
            ):
                _fail(
                    f"stair.corridors[{index}].tread_depth_m",
                    "is outside the configured stair tread limits",
                )
            if max_riser_deviation >= 0.5 * corridor_report.riser_height_m:
                _fail(
                    "stair.max_riser_deviation_m",
                    "must be less than half of every enabled riser height",
                )
            if max_tread_deviation >= 0.5 * corridor_report.tread_depth_m:
                _fail(
                    "stair.max_tread_deviation_m",
                    "must be less than half of every enabled tread depth",
                )
            if corridor_report.corridor_id in seen_ids:
                _fail(f"stair.corridors[{index}].id", "must be unique")
            seen_ids.add(corridor_report.corridor_id)
            direction_allowed = (
                corridor_report.allow_up
                if target == "stair-up"
                else corridor_report.allow_down
            )
            if direction_allowed:
                enabled_corridor_ids.append(corridor_report.corridor_id)
        if not enabled_corridor_ids:
            required_direction = "allow_up" if target == "stair-up" else "allow_down"
            _fail(
                "stair.corridors",
                f"has no enabled corridor with {required_direction}=true for {target}",
            )

    return ValidationReport(
        target=target,
        site_id=site_id,
        map_sha256=map_hash,
        enabled_corridor_ids=tuple(enabled_corridor_ids),
    )


def _runtime_params(runtime: dict[str, Any], node_name: str) -> dict[str, Any]:
    node = _mapping(runtime.get(node_name), f"runtime.{node_name}")
    return _mapping(
        node.get("ros__parameters"),
        f"runtime.{node_name}.ros__parameters",
    )


def _nested_value(mapping: dict[str, Any], keys: Sequence[str], path: str) -> Any:
    current: Any = mapping
    traversed: list[str] = []
    for key in keys:
        traversed.append(key)
        if not isinstance(current, dict):
            _fail(f"{path}.{'.'.join(traversed[:-1])}", "must be a mapping")
        lookup_key: Any = key
        if lookup_key not in current and key.isdigit() and int(key) in current:
            lookup_key = int(key)
        if lookup_key not in current:
            _fail(f"{path}.{'.'.join(traversed)}", "is missing")
        current = current[lookup_key]
    return current


def _runtime_expect(
    params: dict[str, Any],
    keys: Sequence[str],
    expected: Any,
    node_name: str,
) -> None:
    path = f"runtime.{node_name}.ros__parameters"
    actual = _nested_value(params, keys, path)
    value_path = f"{path}.{'.'.join(keys)}"
    if isinstance(expected, bool):
        if not isinstance(actual, bool) or actual is not expected:
            _fail(value_path, f"must equal {expected!r}, got {actual!r}")
        return
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if isinstance(actual, bool) or not isinstance(actual, (int, float)):
            _fail(value_path, f"must equal {expected!r}, got {actual!r}")
        actual_number = float(actual)
        expected_number = float(expected)
        if not math.isfinite(actual_number) or not math.isclose(
            actual_number, expected_number, rel_tol=1.0e-6, abs_tol=1.0e-8
        ):
            _fail(value_path, f"must equal {expected!r}, got {actual!r}")
        return
    if isinstance(expected, (list, tuple)):
        actual_list = _list(actual, value_path)
        if len(actual_list) != len(expected):
            _fail(value_path, f"must contain {len(expected)} values")
        for index, (actual_item, expected_item) in enumerate(
            zip(actual_list, expected)
        ):
            if isinstance(expected_item, (int, float)) and not isinstance(
                expected_item, bool
            ):
                if isinstance(actual_item, bool) or not isinstance(
                    actual_item, (int, float)
                ) or not math.isclose(
                    float(actual_item),
                    float(expected_item),
                    rel_tol=1.0e-6,
                    abs_tol=1.0e-8,
                ):
                    _fail(
                        f"{value_path}[{index}]",
                        f"must equal {expected_item!r}, got {actual_item!r}",
                    )
            elif actual_item != expected_item:
                _fail(
                    f"{value_path}[{index}]",
                    f"must equal {expected_item!r}, got {actual_item!r}",
                )
        return
    if actual != expected:
        _fail(value_path, f"must equal {expected!r}, got {actual!r}")


def _runtime_number(
    params: dict[str, Any],
    keys: Sequence[str],
    node_name: str,
    *,
    strictly_positive: bool = False,
    minimum: float | None = None,
) -> float:
    path = f"runtime.{node_name}.ros__parameters"
    actual = _nested_value(params, keys, path)
    value_path = f"{path}.{'.'.join(keys)}"
    if isinstance(actual, bool) or not isinstance(actual, (int, float)):
        _fail(value_path, "must be a finite number")
    result = float(actual)
    if not math.isfinite(result):
        _fail(value_path, "must be finite")
    if strictly_positive and result <= 0.0:
        _fail(value_path, "must be greater than zero")
    if minimum is not None and result < minimum:
        _fail(value_path, f"must be at least {minimum}")
    return result


def _flatten_polygon(value: Any, path: str) -> list[float]:
    polygon = _polygon(value, path)
    return [coordinate for vertex in polygon for coordinate in vertex]


def _enabled_site_corridors(profile: dict[str, Any]) -> list[dict[str, Any]]:
    stair = _mapping(profile.get("stair"), "stair")
    result: list[dict[str, Any]] = []
    for index, entry in enumerate(_list(stair.get("corridors"), "stair.corridors")):
        corridor = _mapping(entry, f"stair.corridors[{index}]")
        if _boolean(corridor, "enabled", f"stair.corridors[{index}]"):
            result.append(corridor)
    return result


def _validate_runtime_safe_disabled(runtime: dict[str, Any]) -> None:
    disabled_paths = (
        ("map1", ("terrain_roi_enabled",)),
        ("perception_3d_local", ("terrain", "enabled")),
        ("perception_3d_global", ("terrain", "enabled")),
        ("global_planner", ("terrain", "enabled")),
        ("global_planner", ("terrain", "stair_enabled")),
        ("global_planner", ("terrain", "allow_stair_up")),
        ("global_planner", ("terrain", "allow_stair_down")),
        (
            "trajectory_generators",
            ("differential_drive_terrain", "terrain_enabled"),
        ),
        (
            "trajectory_generators",
            ("differential_drive_terrain", "terrain_stairs_enabled"),
        ),
        ("mpc_critics", ("terrain_support", "enabled")),
        ("mpc_critics", ("terrain_support", "stairs_enabled")),
        (
            "mpc_critics",
            ("terrain_collision", "stair_semantics", "enabled"),
        ),
        ("local_planner", ("goal_surface_match_required",)),
        ("p2p_move_base", ("terrain_supervisor_enabled",)),
        (
            "perception_3d_local",
            ("lidar", "stair_riser_marking", "enabled"),
        ),
        (
            "perception_3d_global",
            ("lidar", "stair_riser_marking", "enabled"),
        ),
    )
    for node_name, keys in disabled_paths:
        _runtime_expect(_runtime_params(runtime, node_name), keys, False, node_name)

    fail_closed_paths = (
        ("perception_3d_local", ("terrain", "enabled"), ("terrain", "map_hash")),
        ("perception_3d_global", ("terrain", "enabled"), ("terrain", "map_hash")),
    )
    # An empty map hash is required while the shipped TerrainLayer instances
    # are disabled, preventing an accidental one-flag activation.
    for node_name, _, hash_keys in fail_closed_paths:
        _runtime_expect(_runtime_params(runtime, node_name), hash_keys, "", node_name)

    for node_name, keys in (
        ("global_planner", ("terrain", "fail_closed")),
        (
            "trajectory_generators",
            ("differential_drive_terrain", "terrain_fail_closed"),
        ),
        ("mpc_critics", ("terrain_support", "fail_closed")),
        (
            "mpc_critics",
            ("terrain_collision", "stair_semantics", "fail_closed"),
        ),
        (
            "perception_3d_local",
            ("lidar", "stair_riser_marking", "fail_closed"),
        ),
        (
            "perception_3d_global",
            ("lidar", "stair_riser_marking", "fail_closed"),
        ),
    ):
        _runtime_expect(_runtime_params(runtime, node_name), keys, True, node_name)

    p2p = _runtime_params(runtime, "p2p_move_base")
    main_generator = _nested_value(
        p2p,
        ("main_trajectory_generator",),
        "runtime.p2p_move_base.ros__parameters",
    )
    if main_generator == "differential_drive_terrain":
        _fail(
            "runtime.p2p_move_base.ros__parameters.main_trajectory_generator",
            "must not select the terrain generator in safe-disabled mode",
        )


def _validate_runtime_map_and_layers(
    profile: dict[str, Any], runtime: dict[str, Any], target: str
) -> None:
    map_profile = _mapping(profile.get("map"), "map")
    roi = _mapping(profile.get("terrain_roi"), "terrain_roi")
    terrain = _mapping(profile.get("terrain"), "terrain")
    model = _mapping(terrain.get("model"), "terrain.model")
    map_hash = _sha256(map_profile.get("sha256"), "map.sha256")

    map_params = _runtime_params(runtime, "map1")
    _runtime_expect(map_params, ("source_map_sha256",), map_hash, "map1")
    _runtime_expect(map_params, ("terrain_roi_enabled",), True, "map1")
    _runtime_expect(
        map_params,
        ("terrain_roi_voxel_size",),
        _number(roi, "voxel_size_m", "terrain_roi", strictly_positive=True),
        "map1",
    )
    roi_minimum = _vector(roi.get("min_xyz"), "terrain_roi.min_xyz", 3)
    roi_maximum = _vector(roi.get("max_xyz"), "terrain_roi.max_xyz", 3)
    for axis_index, axis in enumerate("xyz"):
        _runtime_expect(
            map_params,
            (f"terrain_roi_min_{axis}",),
            roi_minimum[axis_index],
            "map1",
        )
        _runtime_expect(
            map_params,
            (f"terrain_roi_max_{axis}",),
            roi_maximum[axis_index],
            "map1",
        )
    coarse_key = (
        "complete_ground_voxel_size"
        if "complete_ground_voxel_size" in map_params
        else "complete_map_voxel_size"
    )
    _runtime_expect(
        map_params,
        (coarse_key,),
        _number(
            map_profile,
            "complete_ground_voxel_size_m",
            "map",
            strictly_positive=True,
        ),
        "map1",
    )

    active_corridors = _enabled_site_corridors(profile)
    for node_name, local_instance in (
        ("perception_3d_local", True),
        ("perception_3d_global", False),
    ):
        params = _runtime_params(runtime, node_name)
        plugins = _list(
            _nested_value(
                params, ("plugins",), f"runtime.{node_name}.ros__parameters"
            ),
            f"runtime.{node_name}.ros__parameters.plugins",
        )
        if "terrain" not in plugins:
            _fail(
                f"runtime.{node_name}.ros__parameters.plugins",
                "must include the TerrainLayer plugin",
            )
        _runtime_expect(params, ("global_frame",), map_profile["frame_id"], node_name)
        _runtime_expect(
            params, ("map", "ground_topic"), "map1/navigation_ground", node_name
        )
        _runtime_expect(params, ("terrain", "enabled"), True, node_name)
        _runtime_expect(
            params, ("terrain", "is_local_planner"), local_instance, node_name
        )
        _runtime_expect(params, ("terrain", "map_hash"), map_hash, node_name)
        _runtime_expect(
            params,
            ("terrain", "map_identity_topic"),
            "map1/map_sha256",
            node_name,
        )
        _runtime_expect(
            params,
            ("terrain", "terrain_ground_topic"),
            "map1/terrain_ground",
            node_name,
        )
        _runtime_expect(params, ("terrain", "require_live_ground"), True, node_name)
        _runtime_expect(
            params,
            ("terrain", "max_age_sec"),
            _number(model, "max_age_s", "terrain.model", strictly_positive=True),
            node_name,
        )
        _runtime_expect(
            params,
            ("terrain", "status_min_confidence"),
            _number(
                model,
                "min_confidence",
                "terrain.model",
                minimum=0.0,
                maximum=1.0,
            ),
            node_name,
        )
        _runtime_expect(
            params,
            ("terrain", "status_min_support_ratio"),
            _number(
                model,
                "min_support_ratio",
                "terrain.model",
                minimum=0.0,
                maximum=1.0,
            ),
            node_name,
        )
        for runtime_key, profile_key in (
            ("normal_radius_m", "normal_radius_m"),
            ("min_normal_neighbors", "min_neighbors"),
            ("max_plane_residual_m", "max_plane_residual_m"),
        ):
            _runtime_expect(
                params,
                ("terrain", "model", runtime_key),
                model[profile_key],
                node_name,
            )

        if target.startswith("stair"):
            _runtime_expect(
                params,
                ("terrain", "require_live_obstacle_for_stairs"),
                True,
                node_name,
            )
            expected_ids = sorted(int(corridor["id"]) for corridor in active_corridors)
            actual_ids = _list(
                _nested_value(
                    params,
                    ("terrain", "stair_ids"),
                    f"runtime.{node_name}.ros__parameters",
                ),
                f"runtime.{node_name}.ros__parameters.terrain.stair_ids",
            )
            if sorted(actual_ids) != expected_ids:
                _fail(
                    f"runtime.{node_name}.ros__parameters.terrain.stair_ids",
                    f"must equal the enabled site corridor ids {expected_ids!r}",
                )
            for corridor in active_corridors:
                stair_id = str(int(corridor["id"]))
                lower = _mapping(
                    corridor.get("lower_landing"),
                    f"stair.corridors[{stair_id}].lower_landing",
                )
                upper = _mapping(
                    corridor.get("upper_landing"),
                    f"stair.corridors[{stair_id}].upper_landing",
                )
                comparisons = (
                    ("map_hash", map_hash),
                    ("up_axis", corridor["up_axis_xyz"]),
                    ("lower_landing_center", lower["center_xyz"]),
                    ("upper_landing_center", upper["center_xyz"]),
                    ("first_riser_center", corridor["first_riser_center_xyz"]),
                    (
                        "corridor_polygon_xy",
                        _flatten_polygon(
                            corridor["corridor_polygon_xy"],
                            f"stair.corridors[{stair_id}].corridor_polygon_xy",
                        ),
                    ),
                    (
                        "lower_landing_polygon_xy",
                        _flatten_polygon(
                            lower["polygon_xy"],
                            f"stair.corridors[{stair_id}].lower_landing.polygon_xy",
                        ),
                    ),
                    (
                        "upper_landing_polygon_xy",
                        _flatten_polygon(
                            upper["polygon_xy"],
                            f"stair.corridors[{stair_id}].upper_landing.polygon_xy",
                        ),
                    ),
                    ("width_m", corridor["width_m"]),
                    ("riser_height_m", corridor["riser_height_m"]),
                    ("tread_depth_m", corridor["tread_depth_m"]),
                    ("step_count", corridor["step_count"]),
                    ("confidence", corridor["confidence"]),
                    ("allow_up", corridor["allow_up"]),
                    ("allow_down", corridor["allow_down"]),
                )
                for key, expected in comparisons:
                    _runtime_expect(
                        params,
                        ("terrain", "stairs", stair_id, key),
                        expected,
                        node_name,
                    )
        else:
            _runtime_expect(params, ("terrain", "stair_ids"), [], node_name)


def _validate_runtime_global_planner(
    profile: dict[str, Any], runtime: dict[str, Any], target: str
) -> None:
    terrain_profile = _mapping(profile.get("terrain"), "terrain")
    model = _mapping(terrain_profile.get("model"), "terrain.model")
    traversal = _mapping(terrain_profile.get("traversal"), "terrain.traversal")
    ramp = _mapping(traversal.get("ramp"), "terrain.traversal.ramp")
    generic = _mapping(traversal.get("generic"), "terrain.traversal.generic")
    stair = _mapping(profile.get("stair"), "stair")
    map_hash = _sha256(
        _mapping(profile.get("map"), "map").get("sha256"), "map.sha256"
    )
    params = _runtime_params(runtime, "global_planner")
    _runtime_expect(params, ("terrain", "enabled"), True, "global_planner")
    _runtime_expect(params, ("terrain", "fail_closed"), True, "global_planner")
    _runtime_expect(params, ("terrain", "map_hash"), map_hash, "global_planner")
    comparisons = (
        (
            "support_sample_spacing_m",
            _number(
                model,
                "edge_sample_spacing_m",
                "terrain.model",
                strictly_positive=True,
            ),
        ),
        (
            "support_search_radius_m",
            _number(
                model, "support_radius_m", "terrain.model", strictly_positive=True
            ),
        ),
        (
            "continuous_height_residual_m",
            _number(
                model,
                "max_plane_residual_m",
                "terrain.model",
                strictly_positive=True,
            ),
        ),
        (
            "max_up_slope_rad",
            math.radians(
                _number(
                    ramp,
                    "max_up_slope_deg",
                    "terrain.traversal.ramp",
                    minimum=0.0,
                    maximum=90.0,
                )
            ),
        ),
        (
            "max_down_slope_rad",
            math.radians(
                _number(
                    ramp,
                    "max_down_slope_deg",
                    "terrain.traversal.ramp",
                    minimum=0.0,
                    maximum=90.0,
                )
            ),
        ),
        (
            "max_cross_slope_rad",
            math.radians(
                _number(
                    ramp,
                    "max_cross_slope_deg",
                    "terrain.traversal.ramp",
                    minimum=0.0,
                    maximum=90.0,
                )
            ),
        ),
        ("max_roughness_m", traversal["max_roughness_m"]),
        (
            "max_normal_change_rad",
            math.radians(float(traversal["max_normal_change_deg"])),
        ),
        ("max_step_up_m", generic["max_step_up_m"]),
        ("max_step_down_m", generic["max_step_down_m"]),
        ("min_support_ratio", model["min_support_ratio"]),
        ("max_unknown_ratio", model["max_unknown_ratio"]),
        ("min_confidence", model["min_confidence"]),
        ("max_support_sample_spacing_m", model["edge_sample_spacing_m"]),
    )
    for key, expected in comparisons:
        _runtime_expect(params, ("terrain", key), expected, "global_planner")
    _runtime_expect(
        params,
        ("terrain", "require_manual_corridor"),
        True,
        "global_planner",
    )
    _runtime_expect(
        params,
        ("terrain", "require_online_confirmation"),
        True,
        "global_planner",
    )
    _runtime_expect(params, ("terrain", "max_step_index_delta"), 1, "global_planner")
    if _runtime_number(
        params,
        ("terrain", "ground_index_alignment_tolerance_m"),
        "global_planner",
        strictly_positive=True,
    ) > float(model["support_radius_m"]) + EPSILON:
        _fail(
            "runtime.global_planner.ros__parameters.terrain."
            "ground_index_alignment_tolerance_m",
            "must not exceed terrain.model.support_radius_m",
        )
    _runtime_number(
        params,
        ("terrain", "support_vertical_tolerance_m"),
        "global_planner",
        strictly_positive=True,
    )
    if _runtime_number(
        params,
        ("terrain", "distance_cost_weight"),
        "global_planner",
        minimum=1.0,
    ) < 1.0:
        _fail(
            "runtime.global_planner.ros__parameters.terrain.distance_cost_weight",
            "must preserve the A* heuristic",
        )

    stair_target = target.startswith("stair")
    _runtime_expect(
        params, ("terrain", "stair_enabled"), stair_target, "global_planner"
    )
    active_corridors = _enabled_site_corridors(profile)
    allow_up = stair_target and any(bool(value["allow_up"]) for value in active_corridors)
    allow_down = stair_target and any(
        bool(value["allow_down"]) for value in active_corridors
    )
    _runtime_expect(
        params, ("terrain", "allow_stair_up"), allow_up, "global_planner"
    )
    _runtime_expect(
        params, ("terrain", "allow_stair_down"), allow_down, "global_planner"
    )
    if stair_target:
        for runtime_key, profile_key in (
            ("max_stair_riser_height_m", "max_riser_height_m"),
            ("min_stair_tread_depth_m", "min_tread_depth_m"),
            ("max_stair_tread_depth_m", "max_tread_depth_m"),
            ("max_stair_riser_deviation_m", "max_riser_deviation_m"),
            ("max_stair_tread_deviation_m", "max_tread_deviation_m"),
        ):
            _runtime_expect(
                params,
                ("terrain", runtime_key),
                stair[profile_key],
                "global_planner",
            )
        _runtime_expect(
            params,
            ("terrain", "max_stair_heading_error_rad"),
            math.radians(float(stair["max_heading_error_deg"])),
            "global_planner",
        )
    else:
        for key in (
            "max_stair_riser_height_m",
            "min_stair_tread_depth_m",
            "max_stair_tread_depth_m",
            "max_stair_riser_deviation_m",
            "max_stair_tread_deviation_m",
            "max_stair_heading_error_rad",
        ):
            _runtime_expect(params, ("terrain", key), 0.0, "global_planner")


def _validate_runtime_local_control(
    profile: dict[str, Any], runtime: dict[str, Any], target: str
) -> None:
    terrain_profile = _mapping(profile.get("terrain"), "terrain")
    model = _mapping(terrain_profile.get("model"), "terrain.model")
    traversal = _mapping(terrain_profile.get("traversal"), "terrain.traversal")
    ramp = _mapping(traversal.get("ramp"), "terrain.traversal.ramp")
    stair = _mapping(profile.get("stair"), "stair")
    stair_target = target.startswith("stair")

    local = _runtime_params(runtime, "local_planner")
    _runtime_expect(
        local, ("goal_surface_match_required",), True, "local_planner"
    )
    _runtime_number(
        local,
        ("goal_terrain_search_radius",),
        "local_planner",
        strictly_positive=True,
    )

    generator_params = _runtime_params(runtime, "trajectory_generators")
    generator_prefix = ("differential_drive_terrain",)
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_enabled",),
        True,
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_fail_closed",),
        True,
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_min_support_ratio",),
        model["min_support_ratio"],
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_min_confidence",),
        model["min_confidence"],
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_max_normal_change_rad",),
        math.radians(float(traversal["max_normal_change_deg"])),
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_stairs_enabled",),
        stair_target,
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_require_manual_corridor",),
        True,
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("terrain_require_online_confirmation",),
        True,
        "trajectory_generators",
    )
    _runtime_expect(
        generator_params,
        generator_prefix + ("enable_in_place_rotation",),
        False,
        "trajectory_generators",
    )
    _runtime_number(
        generator_params,
        generator_prefix + ("terrain_body_clearance_m",),
        "trajectory_generators",
        strictly_positive=True,
    )
    _runtime_number(
        generator_params,
        generator_prefix + ("terrain_max_support_xy_distance_m",),
        "trajectory_generators",
        strictly_positive=True,
    )
    _runtime_number(
        generator_params,
        generator_prefix + ("terrain_max_support_vertical_distance_m",),
        "trajectory_generators",
        strictly_positive=True,
    )
    upward_normal = _runtime_number(
        generator_params,
        generator_prefix + ("terrain_min_upward_normal_z",),
        "trajectory_generators",
        strictly_positive=True,
    )
    if upward_normal > 1.0 + EPSILON:
        _fail(
            "runtime.trajectory_generators.ros__parameters."
            "differential_drive_terrain.terrain_min_upward_normal_z",
            "must not exceed 1.0",
        )

    if stair_target:
        _runtime_expect(
            generator_params,
            generator_prefix + ("terrain_max_stair_heading_error_rad",),
            math.radians(float(stair["max_heading_error_deg"])),
            "trajectory_generators",
        )
        maximum_forward = float(
            stair["max_up_speed_mps"]
            if target == "stair-up"
            else stair["max_down_speed_mps"]
        )
        minimum_forward = float(stair["v_exec_min_mps"])
        maximum_yaw = float(stair["max_committed_yaw_rps"])
    else:
        _runtime_expect(
            generator_params,
            generator_prefix + ("terrain_max_stair_heading_error_rad",),
            0.0,
            "trajectory_generators",
        )
        maximum_forward = max(
            float(ramp["max_up_speed_mps"]),
            float(ramp["max_down_speed_mps"]),
        )
        minimum_forward = float(
            _mapping(profile.get("ramp_capability"), "ramp_capability")[
                "v_exec_min_mps"
            ]
        )
        maximum_yaw = float(ramp["max_yaw_rps"])
    configured_forward = _runtime_number(
        generator_params,
        generator_prefix + ("max_vel_x",),
        "trajectory_generators",
        strictly_positive=True,
    )
    if configured_forward + EPSILON < minimum_forward or (
        configured_forward > maximum_forward + EPSILON
    ):
        _fail(
            "runtime.trajectory_generators.ros__parameters."
            "differential_drive_terrain.max_vel_x",
            "must be inside the selected target's verified speed envelope",
        )
    configured_yaw = _runtime_number(
        generator_params,
        generator_prefix + ("max_vel_theta",),
        "trajectory_generators",
        minimum=0.0,
    )
    if configured_yaw > maximum_yaw + EPSILON:
        _fail(
            "runtime.trajectory_generators.ros__parameters."
            "differential_drive_terrain.max_vel_theta",
            "exceeds the selected target's verified yaw limit",
        )

    critics = _runtime_params(runtime, "mpc_critics")
    _runtime_expect(
        critics,
        ("terrain_support", "trajectory_generator"),
        "differential_drive_terrain",
        "mpc_critics",
    )
    _runtime_expect(critics, ("terrain_support", "enabled"), True, "mpc_critics")
    _runtime_expect(
        critics, ("terrain_support", "fail_closed"), True, "mpc_critics"
    )
    _runtime_expect(
        critics,
        ("terrain_support", "min_support_ratio"),
        model["min_support_ratio"],
        "mpc_critics",
    )
    _runtime_expect(
        critics,
        ("terrain_support", "min_confidence"),
        model["min_confidence"],
        "mpc_critics",
    )
    _runtime_expect(
        critics,
        ("terrain_support", "stairs_enabled"),
        stair_target,
        "mpc_critics",
    )
    _runtime_expect(
        critics,
        ("terrain_support", "require_manual_corridor"),
        True,
        "mpc_critics",
    )
    _runtime_expect(
        critics,
        ("terrain_support", "require_online_confirmation"),
        True,
        "mpc_critics",
    )
    _runtime_expect(
        critics,
        ("terrain_support", "max_step_index_delta"),
        1,
        "mpc_critics",
    )
    _runtime_number(
        critics,
        ("terrain_support", "max_support_distance_m"),
        "mpc_critics",
        strictly_positive=True,
    )
    _runtime_number(
        critics,
        ("terrain_support", "max_z_gap_m"),
        "mpc_critics",
        strictly_positive=True,
    )

    p2p = _runtime_params(runtime, "p2p_move_base")
    _runtime_expect(
        p2p,
        ("main_trajectory_generator",),
        "differential_drive_terrain",
        "p2p_move_base",
    )
    _runtime_expect(
        p2p,
        ("terrain_supervisor_enabled",),
        stair_target,
        "p2p_move_base",
    )
    if stair_target:
        _runtime_expect(p2p, ("require_gait_monitor",), True, "p2p_move_base")
        _runtime_expect(
            p2p,
            ("terrain_status_timeout_sec",),
            model["max_age_s"],
            "p2p_move_base",
        )
        _runtime_expect(
            p2p,
            ("gait_monitor_timeout_sec",),
            model["max_age_s"],
            "p2p_move_base",
        )
        _runtime_number(
            p2p,
            ("stair_confirmation_cycles",),
            "p2p_move_base",
            strictly_positive=True,
        )
        _runtime_expect(
            p2p,
            ("stair_min_confidence",),
            model["min_confidence"],
            "p2p_move_base",
        )
        _runtime_expect(
            p2p,
            ("stair_min_support_ratio",),
            model["min_support_ratio"],
            "p2p_move_base",
        )
        _runtime_expect(
            p2p,
            ("stair_max_heading_error_rad",),
            math.radians(float(stair["max_heading_error_deg"])),
            "p2p_move_base",
        )
        _runtime_expect(
            p2p,
            ("stair_max_lateral_error_m",),
            stair["max_lateral_error_m"],
            "p2p_move_base",
        )
        _runtime_expect(
            p2p, ("stair_align_max_forward_mps",), 0.0, "p2p_move_base"
        )
        _runtime_expect(
            p2p,
            ("stair_committed_max_yaw_rps",),
            stair["max_committed_yaw_rps"],
            "p2p_move_base",
        )
        _runtime_expect(
            p2p,
            ("stair_align_trajectory_generator",),
            "differential_drive_stair_align",
            "p2p_move_base",
        )


def _validate_runtime_stair_semantics(
    profile: dict[str, Any], runtime: dict[str, Any], target: str
) -> None:
    stair_target = target.startswith("stair")
    stair = _mapping(profile.get("stair"), "stair")
    model = _mapping(
        _mapping(profile.get("terrain"), "terrain").get("model"),
        "terrain.model",
    )
    map_hash = _sha256(
        _mapping(profile.get("map"), "map").get("sha256"), "map.sha256"
    )
    semantics = _mapping(stair.get("riser_semantics"), "stair.riser_semantics")
    critics = _runtime_params(runtime, "mpc_critics")
    collision_prefix = ("terrain_collision", "stair_semantics")
    _runtime_expect(
        critics,
        collision_prefix + ("enabled",),
        stair_target,
        "mpc_critics",
    )
    _runtime_expect(
        critics,
        collision_prefix + ("fail_closed",),
        True,
        "mpc_critics",
    )

    for node_name in ("perception_3d_local", "perception_3d_global"):
        params = _runtime_params(runtime, node_name)
        prefix = ("lidar", "stair_riser_marking")
        _runtime_expect(params, prefix + ("enabled",), stair_target, node_name)
        _runtime_expect(params, prefix + ("fail_closed",), True, node_name)

    if not stair_target:
        return

    common_expected = (
        ("expected_map_hash", map_hash),
        ("max_snapshot_age_sec", semantics["max_snapshot_age_s"]),
        ("minimum_stair_confidence", stair["min_confidence"]),
        ("max_node_match_distance_m", semantics["max_node_match_distance_m"]),
        ("riser_plane_tolerance_m", semantics["riser_plane_tolerance_m"]),
        ("riser_lateral_tolerance_m", semantics["riser_lateral_tolerance_m"]),
        ("riser_vertical_tolerance_m", semantics["riser_vertical_tolerance_m"]),
    )
    for key, expected in common_expected:
        _runtime_expect(
            critics, collision_prefix + (key,), expected, "mpc_critics"
        )
    envelope_min = _vector(
        semantics.get("leg_envelope_min_xyz"),
        "stair.riser_semantics.leg_envelope_min_xyz",
        3,
    )
    envelope_max = _vector(
        semantics.get("leg_envelope_max_xyz"),
        "stair.riser_semantics.leg_envelope_max_xyz",
        3,
    )
    for index, axis in enumerate("xyz"):
        _runtime_expect(
            critics,
            collision_prefix + (f"leg_envelope_min_{axis}_m",),
            envelope_min[index],
            "mpc_critics",
        )
        _runtime_expect(
            critics,
            collision_prefix + (f"leg_envelope_max_{axis}_m",),
            envelope_max[index],
            "mpc_critics",
        )
    for key in (
        "max_support_xy_distance_m",
        "min_body_clearance_m",
        "max_body_clearance_m",
    ):
        _runtime_expect(
            critics, collision_prefix + (key,), semantics[key], "mpc_critics"
        )
    _runtime_expect(
        critics,
        ("terrain_collision", "trajectory_generator"),
        "differential_drive_terrain",
        "mpc_critics",
    )

    for node_name in ("perception_3d_local", "perception_3d_global"):
        params = _runtime_params(runtime, node_name)
        prefix = ("lidar", "stair_riser_marking")
        for key, expected in common_expected:
            _runtime_expect(params, prefix + (key,), expected, node_name)
        _runtime_expect(
            params,
            prefix + ("surface_plane_tolerance_m",),
            semantics["surface_plane_tolerance_m"],
            node_name,
        )
        marking_minimum = _runtime_number(
            params,
            ("lidar", "marking_minimum_height"),
            node_name,
        )
        if marking_minimum > envelope_max[2] + EPSILON:
            _fail(
                f"runtime.{node_name}.ros__parameters.lidar.marking_minimum_height",
                "must include the configured stair leg/riser envelope",
            )
        marking_maximum = _runtime_number(
            params,
            ("lidar", "marking_height"),
            node_name,
        )
        if marking_maximum < envelope_min[2] - EPSILON:
            _fail(
                f"runtime.{node_name}.ros__parameters.lidar.marking_height",
                "must include the configured stair leg/riser envelope",
            )
    if not math.isclose(
        float(semantics["max_snapshot_age_s"]),
        float(model["max_age_s"]),
        rel_tol=1.0e-6,
        abs_tol=1.0e-8,
    ):
        _fail(
            "stair.riser_semantics.max_snapshot_age_s",
            "must equal terrain.model.max_age_s for one freshness contract",
        )


def validate_runtime_config(
    profile: dict[str, Any], runtime: dict[str, Any], target: str
) -> None:
    """Validate the actual Go2 ROS parameter file against a site profile."""
    validate_profile(profile, target)
    if target == "safe-disabled":
        _validate_runtime_safe_disabled(runtime)
        return
    _validate_runtime_map_and_layers(profile, runtime, target)
    _validate_runtime_global_planner(profile, runtime, target)
    _validate_runtime_local_control(profile, runtime, target)
    _validate_runtime_stair_semantics(profile, runtime, target)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline fail-closed validation for a Go2 ramp/stair site profile"
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--target", choices=TARGETS, default="safe-disabled")
    parser.add_argument(
        "--runtime-config",
        type=Path,
        help=(
            "actual go2_xt16_navigation.yaml; required for ramp/stair targets "
            "and optional for safe-disabled auditing"
        ),
    )
    args = parser.parse_args()
    try:
        loaded = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
        profile = _mapping(loaded, "root")
        report = validate_profile(profile, args.target)
        if args.runtime_config is None and args.target != "safe-disabled":
            _fail(
                "--runtime-config",
                "is required for ramp/stair targets; site-only validation is insufficient",
            )
        if args.runtime_config is not None:
            runtime_loaded = yaml.safe_load(
                args.runtime_config.read_text(encoding="utf-8")
            )
            validate_runtime_config(
                profile, _mapping(runtime_loaded, "runtime"), args.target
            )
    except (OSError, ProfileValidationError, yaml.YAMLError) as exc:
        print(f"GO2_XT16_TERRAIN_PROFILE_STATUS=FAIL: {exc}")
        return 1

    print(f"PROFILE={args.profile}")
    print(f"TARGET={report.target}")
    print(f"RUNTIME_CONFIG={args.runtime_config or 'NOT_PROVIDED'}")
    print(f"RUNTIME_CONFIG_VALIDATED={'TRUE' if args.runtime_config else 'FALSE'}")
    print(f"SITE_ID={report.site_id or 'UNSET_DISABLED_TEMPLATE'}")
    print(f"MAP_SHA256={report.map_sha256 or 'UNSET_DISABLED_TEMPLATE'}")
    print(
        "ENABLED_STAIR_CORRIDORS="
        + (",".join(str(value) for value in report.enabled_corridor_ids) or "NONE")
    )
    print("CONFIG_CONTRACT_READY=TRUE")
    print("LIVE_MOTION_READY=NOT_ASSESSED")
    print("NO_ROS_OR_MOTION_ACTIONS=TRUE")
    print("GO2_XT16_TERRAIN_PROFILE_STATUS=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
