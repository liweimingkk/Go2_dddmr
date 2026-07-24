#!/usr/bin/env python3
"""Versioned JSON I/O shared by sequential P2P waypoint missions."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple


MISSION_VERSION = 1
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
MAX_DWELL_SEC = 3600.0


class MissionValidationError(ValueError):
    """Raised when a mission document cannot be executed safely."""


@dataclass(frozen=True)
class InitialPose:
    frame_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    covariance: Tuple[float, ...]


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    x: float
    y: float
    z: float
    yaw: float
    dwell_sec: float


@dataclass(frozen=True)
class Mission:
    mission_id: str
    path: Path
    initial_pose_path: Path
    waypoints: Tuple[Waypoint, ...]


class ArrivalWindow:
    """Require an arrived pose and stopped command for a continuous window."""

    def __init__(self, stable_sec: float) -> None:
        if not _finite_number(stable_sec) or stable_sec <= 0.0:
            raise MissionValidationError("stable_sec must be finite and positive")
        self.stable_sec = float(stable_sec)
        self._start: Optional[float] = None

    def reset(self) -> None:
        self._start = None

    def update(self, now_sec: float, arrived: bool, stopped: bool) -> bool:
        if not _finite_number(now_sec) or not arrived or not stopped:
            self.reset()
            return False
        if self._start is None or now_sec < self._start:
            self._start = float(now_sec)
            return False
        return now_sec - self._start >= self.stable_sec


class UnhealthyGraceWindow:
    """Track a bounded interval for a recoverable unhealthy input."""

    def __init__(self, grace_sec: float) -> None:
        if not _finite_number(grace_sec) or grace_sec <= 0.0:
            raise MissionValidationError("grace_sec must be finite and positive")
        self.grace_sec = float(grace_sec)
        self._start: Optional[float] = None

    @property
    def active(self) -> bool:
        return self._start is not None

    def reset(self) -> None:
        self._start = None

    def elapsed(self, now_sec: float) -> float:
        if not _finite_number(now_sec) or self._start is None:
            return 0.0
        if now_sec < self._start:
            self._start = float(now_sec)
            return 0.0
        return float(now_sec) - self._start

    def mark_unhealthy(self, now_sec: float) -> bool:
        if not _finite_number(now_sec):
            self.reset()
            return True
        if self._start is None or now_sec < self._start:
            self._start = float(now_sec)
            return False
        return self.elapsed(now_sec) >= self.grace_sec

    def mark_healthy(self, now_sec: float) -> float:
        held_sec = self.elapsed(now_sec)
        self.reset()
        return held_sec


def normalize_angle(angle: float) -> float:
    if not _finite_number(angle):
        raise MissionValidationError("yaw must be finite")
    return math.remainder(float(angle), 2.0 * math.pi)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    yaw = normalize_angle(yaw)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quaternion(values: Sequence[float]) -> float:
    x, y, z, w = _validated_quaternion(values, "orientation")
    return normalize_angle(
        math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
    )


def load_initial_pose(path: os.PathLike[str] | str) -> InitialPose:
    document = _load_object(Path(path), "initial pose")
    if document.get("version") != MISSION_VERSION:
        raise MissionValidationError(
            f"initial pose version must be {MISSION_VERSION}"
        )
    if document.get("frame_id") != "map":
        raise MissionValidationError("initial pose frame_id must be 'map'")
    pose = _mapping(document.get("pose"), "initial pose.pose")
    position_map = _mapping(pose.get("position"), "initial pose.position")
    orientation_map = _mapping(pose.get("orientation"), "initial pose.orientation")
    position = tuple(
        _number(position_map.get(name), f"initial pose.position.{name}")
        for name in ("x", "y", "z")
    )
    orientation = _validated_quaternion(
        [orientation_map.get(name) for name in ("x", "y", "z", "w")],
        "initial pose.orientation",
    )
    raw_covariance = document.get("covariance")
    if not isinstance(raw_covariance, list) or len(raw_covariance) != 36:
        raise MissionValidationError("initial pose covariance must contain 36 values")
    covariance = tuple(
        _number(value, f"initial pose.covariance[{index}]")
        for index, value in enumerate(raw_covariance)
    )
    for index in (0, 7, 14, 21, 28, 35):
        if covariance[index] < 0.0:
            raise MissionValidationError(
                f"initial pose covariance diagonal {index} must be nonnegative"
            )
    return InitialPose("map", position, orientation, covariance)


def load_mission(
    path: os.PathLike[str] | str,
    *,
    allowed_root: os.PathLike[str] | str | None = None,
) -> Mission:
    mission_path = Path(path).expanduser().resolve()
    if allowed_root is not None:
        _require_within(mission_path, Path(allowed_root), "mission file")
    document = _load_object(mission_path, "mission")
    if document.get("version") != MISSION_VERSION:
        raise MissionValidationError(f"mission version must be {MISSION_VERSION}")
    mission_id = document.get("mission_id")
    if not isinstance(mission_id, str) or not IDENTIFIER_PATTERN.fullmatch(mission_id):
        raise MissionValidationError(
            "mission_id must start with an alphanumeric character and contain "
            "only alphanumerics, '.', '_' or '-'"
        )

    initial_reference = document.get("initial_pose_file")
    if not isinstance(initial_reference, str) or not initial_reference:
        raise MissionValidationError("initial_pose_file must be a relative path")
    if Path(initial_reference).is_absolute():
        raise MissionValidationError("initial_pose_file must be relative to the mission")
    initial_pose_path = (mission_path.parent / initial_reference).resolve()
    if allowed_root is not None:
        _require_within(initial_pose_path, Path(allowed_root), "initial pose file")
    load_initial_pose(initial_pose_path)

    raw_waypoints = document.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise MissionValidationError("mission must contain at least one waypoint")
    identifiers = set()
    waypoints = []
    for index, value in enumerate(raw_waypoints):
        raw = _mapping(value, f"waypoints[{index}]")
        waypoint_id = raw.get("id")
        if (
            not isinstance(waypoint_id, str)
            or not IDENTIFIER_PATTERN.fullmatch(waypoint_id)
        ):
            raise MissionValidationError(f"waypoints[{index}].id is invalid")
        if waypoint_id in identifiers:
            raise MissionValidationError(f"duplicate waypoint id: {waypoint_id}")
        identifiers.add(waypoint_id)
        yaw = _number(raw.get("yaw"), f"waypoints[{index}].yaw")
        if yaw < -math.pi or yaw > math.pi:
            raise MissionValidationError(
                f"waypoints[{index}].yaw must be within [-pi, pi]"
            )
        dwell_sec = _number(
            raw.get("dwell_sec"), f"waypoints[{index}].dwell_sec"
        )
        if dwell_sec < 0.0 or dwell_sec > MAX_DWELL_SEC:
            raise MissionValidationError(
                f"waypoints[{index}].dwell_sec must be within "
                f"[0, {MAX_DWELL_SEC:g}]"
            )
        waypoints.append(
            Waypoint(
                waypoint_id,
                _number(raw.get("x"), f"waypoints[{index}].x"),
                _number(raw.get("y"), f"waypoints[{index}].y"),
                _number(raw.get("z"), f"waypoints[{index}].z"),
                normalize_angle(yaw),
                dwell_sec,
            )
        )
    return Mission(
        mission_id, mission_path, initial_pose_path, tuple(waypoints)
    )


def initial_pose_document(
    frame_id: str,
    position: Iterable[float],
    orientation: Iterable[float],
    covariance: Iterable[float],
) -> Dict[str, Any]:
    position_values = tuple(float(value) for value in position)
    if len(position_values) != 3 or not all(
        math.isfinite(value) for value in position_values
    ):
        raise MissionValidationError("position must contain three finite values")
    qx, qy, qz, qw = _validated_quaternion(
        tuple(float(value) for value in orientation), "orientation"
    )
    covariance_values = [float(value) for value in covariance]
    document = {
        "version": MISSION_VERSION,
        "frame_id": frame_id,
        "pose": {
            "position": dict(zip(("x", "y", "z"), position_values)),
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
        "covariance": covariance_values,
    }
    if frame_id != "map" or len(covariance_values) != 36:
        raise MissionValidationError(
            "initial pose must use map with a 36-value covariance"
        )
    if not all(math.isfinite(value) for value in covariance_values):
        raise MissionValidationError("initial pose covariance must be finite")
    for index in (0, 7, 14, 21, 28, 35):
        if covariance_values[index] < 0.0:
            raise MissionValidationError(
                f"initial pose covariance diagonal {index} must be nonnegative"
            )
    return document


def mission_document(
    mission_id: str,
    initial_pose_file: str,
    waypoints: Iterable[Waypoint],
) -> Dict[str, Any]:
    if not IDENTIFIER_PATTERN.fullmatch(mission_id):
        raise MissionValidationError("mission_id is invalid")
    points = list(waypoints)
    if not points:
        raise MissionValidationError("mission must contain at least one waypoint")
    return {
        "version": MISSION_VERSION,
        "mission_id": mission_id,
        "initial_pose_file": initial_pose_file,
        "waypoints": [
            {
                "id": point.waypoint_id,
                "x": point.x,
                "y": point.y,
                "z": point.z,
                "yaw": normalize_angle(point.yaw),
                "dwell_sec": point.dwell_sec,
            }
            for point in points
        ],
    }


def atomic_write_json(path: os.PathLike[str] | str, document: Dict[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(
                json.dumps(document, ensure_ascii=False, indent=2, sort_keys=False)
                + "\n"
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def relative_initial_pose_reference(
    mission_path: os.PathLike[str] | str,
    initial_pose_path: os.PathLike[str] | str,
) -> str:
    mission_parent = Path(mission_path).expanduser().resolve().parent
    initial_path = Path(initial_pose_path).expanduser().resolve()
    return os.path.relpath(initial_path, mission_parent)


def _load_object(path: Path, label: str) -> Dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            return _mapping(json.load(stream), label)
    except FileNotFoundError as error:
        raise MissionValidationError(f"{label} file does not exist: {path}") from error
    except (OSError, json.JSONDecodeError) as error:
        raise MissionValidationError(f"cannot read {label} file {path}: {error}") from error


def _mapping(value: Any, label: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise MissionValidationError(f"{label} must be a JSON object")
    return value


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _number(value: Any, label: str) -> float:
    if not _finite_number(value):
        raise MissionValidationError(f"{label} must be a finite number")
    return float(value)


def _validated_quaternion(
    values: Sequence[Any], label: str
) -> Tuple[float, float, float, float]:
    if len(values) != 4:
        raise MissionValidationError(f"{label} must contain x/y/z/w")
    quaternion = tuple(_number(value, label) for value in values)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-6 or abs(norm - 1.0) > 0.05:
        raise MissionValidationError(f"{label} must be a unit quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _require_within(path: Path, root: Path, label: str) -> None:
    resolved_root = root.expanduser().resolve()
    try:
        path.relative_to(resolved_root)
    except ValueError as error:
        raise MissionValidationError(
            f"{label} must stay under {resolved_root}: {path}"
        ) from error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("validate", "mission-id"))
    parser.add_argument("--mission", required=True)
    parser.add_argument("--root")
    arguments = parser.parse_args()
    try:
        mission = load_mission(arguments.mission, allowed_root=arguments.root)
    except MissionValidationError as error:
        parser.error(str(error))
    if arguments.command == "mission-id":
        print(mission.mission_id)
    else:
        print(
            f"VALID mission_id={mission.mission_id} "
            f"waypoints={len(mission.waypoints)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
