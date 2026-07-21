#!/usr/bin/env python3
"""Pure validation helpers for one-map outdoor-to-indoor missions."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


ROUTE_SCHEMA = "dddmr-recorded-route/v1"
MAP_FINGERPRINT_FILES = ("poses.pcd", "map.pcd", "ground.pcd", "edges.pcd")
ROUTE_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def map_fingerprint(map_directory: Path) -> str:
    directory = map_directory.resolve()
    if not directory.is_dir():
        raise ValueError(f"active map directory does not exist: {directory}")
    digest = hashlib.sha256()
    found = 0
    for relative_name in MAP_FINGERPRINT_FILES:
        candidate = directory / relative_name
        if not candidate.is_file():
            continue
        found += 1
        digest.update(relative_name.encode("utf-8"))
        digest.update(sha256_file(candidate).encode("ascii"))
    if found == 0:
        raise ValueError(
            f"active map contains none of {MAP_FINGERPRINT_FILES}: {directory}"
        )
    return digest.hexdigest()


def route_file_for_id(route_directory: Path, route_id: str) -> Path:
    if not ROUTE_ID_PATTERN.fullmatch(route_id):
        raise ValueError(
            "route_id must use only letters, digits, '.', '_' or '-' and must "
            "start with a letter or digit"
        )
    root = route_directory.resolve()
    candidate = (root / f"{route_id}.json").resolve()
    if candidate.parent != root:
        raise ValueError("route_id escapes the configured route directory")
    if not candidate.is_file():
        raise ValueError(f"route file does not exist: {candidate}")
    return candidate


def _finite(values: Iterable[object]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _point_distance(lhs: Mapping[str, object], rhs: Mapping[str, object]) -> float:
    return math.sqrt(
        sum((float(lhs[axis]) - float(rhs[axis])) ** 2 for axis in ("x", "y", "z"))
    )


def validate_route_document(
    document: object,
    route_id: str,
    expected_frame: str,
    active_map_sha256: str,
    max_segment_length: float,
) -> Dict[str, object]:
    if not isinstance(document, dict):
        raise ValueError("route document must be a JSON object")
    if document.get("schema") != ROUTE_SCHEMA:
        raise ValueError(f"unsupported route schema: {document.get('schema')!r}")
    if document.get("route_id") != route_id:
        raise ValueError(
            f"route file id {document.get('route_id')!r} does not match requested {route_id!r}"
        )
    if document.get("frame_id") != expected_frame:
        raise ValueError(
            f"route frame {document.get('frame_id')!r} is not {expected_frame!r}"
        )
    map_info = document.get("map")
    if not isinstance(map_info, dict) or not isinstance(map_info.get("sha256"), str):
        raise ValueError("route does not contain a map fingerprint")
    if map_info["sha256"] != active_map_sha256:
        raise ValueError(
            "route/map fingerprint mismatch; regenerate the route against the active unified map"
        )
    if not math.isfinite(max_segment_length) or max_segment_length <= 0.0:
        raise ValueError("max_segment_length must be finite and positive")

    points = document.get("points")
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("route requires at least three points")
    required = ("x", "y", "z", "qx", "qy", "qz", "qw")
    validated: List[Dict[str, float]] = []
    for index, raw_point in enumerate(points):
        if not isinstance(raw_point, dict) or any(key not in raw_point for key in required):
            raise ValueError(f"route point {index} is missing pose fields")
        if not _finite(raw_point[key] for key in required):
            raise ValueError(f"route point {index} contains a non-finite value")
        point = {key: float(raw_point[key]) for key in required}
        quaternion_norm = math.sqrt(
            sum(point[key] ** 2 for key in ("qx", "qy", "qz", "qw"))
        )
        if quaternion_norm < 1.0e-6 or abs(quaternion_norm - 1.0) > 1.0e-3:
            raise ValueError(f"route point {index} quaternion is not normalized")
        if validated and _point_distance(validated[-1], point) > max_segment_length:
            raise ValueError(
                f"route segment {index - 1}->{index} exceeds {max_segment_length:.3f} m"
            )
        validated.append(point)
    document = dict(document)
    document["points"] = validated
    return document


def load_validated_route(
    route_directory: Path,
    route_id: str,
    active_map_directory: Path,
    expected_frame: str = "map",
    max_segment_length: float = 2.0,
) -> Tuple[Path, Dict[str, object], str]:
    if not expected_frame:
        raise ValueError("expected_frame must not be empty")
    route_path = route_file_for_id(route_directory, route_id)
    active_sha256 = map_fingerprint(active_map_directory)
    with route_path.open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    return (
        route_path,
        validate_route_document(
            document,
            route_id,
            expected_frame,
            active_sha256,
            max_segment_length,
        ),
        active_sha256,
    )


def validate_planar_goal(
    frame_id: str,
    position: Sequence[float],
    quaternion: Sequence[float],
    expected_frame: str = "map",
) -> None:
    if frame_id != expected_frame:
        raise ValueError(f"indoor goal frame must be {expected_frame!r}")
    if len(position) != 3 or not _finite(position):
        raise ValueError("indoor goal position must be finite")
    if len(quaternion) != 4 or not _finite(quaternion):
        raise ValueError("indoor goal quaternion must be finite")
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-6 or abs(norm - 1.0) > 1.0e-3:
        raise ValueError("indoor goal quaternion is not normalized")
    # A navigation target must keep its local Z axis vertical. This is the same
    # planar-orientation contract enforced by p2p_move_base.
    vertical_dot = 1.0 - 2.0 * (qx * qx + qy * qy)
    if abs(vertical_dot - 1.0) > 1.0e-3:
        raise ValueError("indoor goal roll/pitch must be approximately zero")


def normalized_state(status: str) -> str:
    return status.strip().split(":", 1)[0].strip().upper()


class ContinuousStopDetector:
    """Require fresh measured odometry to remain below limits for a duration."""

    def __init__(
        self,
        duration_sec: float,
        odom_timeout_sec: float,
        max_linear_speed: float,
        max_angular_speed: float,
    ) -> None:
        values = (duration_sec, odom_timeout_sec, max_linear_speed, max_angular_speed)
        if not _finite(values) or duration_sec <= 0.0 or odom_timeout_sec <= 0.0:
            raise ValueError("stop detector timing and speed limits are invalid")
        if max_linear_speed < 0.0 or max_angular_speed < 0.0:
            raise ValueError("stop detector speed limits must be non-negative")
        self.duration_sec = duration_sec
        self.odom_timeout_sec = odom_timeout_sec
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.below_since = None

    def update(
        self,
        now_sec: float,
        odom_received_sec: float,
        linear_speed: float,
        angular_speed: float,
    ) -> bool:
        values = (now_sec, odom_received_sec, linear_speed, angular_speed)
        if not _finite(values):
            self.below_since = None
            return False
        age = now_sec - odom_received_sec
        stopped = (
            0.0 <= age <= self.odom_timeout_sec
            and abs(linear_speed) <= self.max_linear_speed
            and abs(angular_speed) <= self.max_angular_speed
        )
        if not stopped:
            self.below_since = None
            return False
        if self.below_since is None:
            self.below_since = now_sec
        return now_sec - self.below_since >= self.duration_sec
