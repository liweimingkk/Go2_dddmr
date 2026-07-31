#!/usr/bin/env python3
"""Validate a DDDMR pose-graph map and render its paired runtime config."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Sequence


class ContractError(ValueError):
    """Raised when a pose-graph map or runtime config is inconsistent."""


@dataclass(frozen=True)
class PcdHeader:
    width: int
    height: int
    points: int
    data: str


@dataclass(frozen=True)
class PoseGraphContract:
    key_frame_count: int
    map_point_count: int
    ground_point_count: int
    edge_point_count: int
    static_layer_timeout_sec: int


KEY_FRAME_SUFFIXES = ("feature", "surface", "ground")
KEY_FRAME_PATTERN = re.compile(r"(?P<index>[0-9]+)_(?P<kind>[a-z]+)[.]pcd")
BASE_STATIC_LAYER_TIMEOUT_SEC = 90
GROUND_POINTS_PER_TIMEOUT_STEP = 75000
MAX_AUTOMATIC_STATIC_LAYER_TIMEOUT_SEC = 900


def _parse_nonnegative_int(path: Path, key: str, values: Sequence[str]) -> int:
    if len(values) != 1 or not values[0].isdigit():
        raise ContractError(f"{path}: {key} must be one nonnegative integer")
    return int(values[0])


def read_pcd_header(path: Path) -> PcdHeader:
    """Read and validate the bounded ASCII header of an ASCII or binary PCD."""

    if not path.is_file():
        raise ContractError(f"missing PCD file: {path}")
    if path.stat().st_size <= 0:
        raise ContractError(f"empty PCD file: {path}")

    fields: Dict[str, Sequence[str]] = {}
    with path.open("rb") as stream:
        for _ in range(128):
            raw_line = stream.readline(65537)
            if not raw_line:
                break
            if len(raw_line) > 65536:
                raise ContractError(
                    f"{path}: PCD header line exceeds 65536 bytes"
                )
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ContractError(
                    f"{path}: PCD header is not ASCII"
                ) from exc
            if not line or line.startswith("#"):
                continue
            tokens = line.split()
            key = tokens[0].upper()
            if key in fields:
                raise ContractError(
                    f"{path}: duplicate PCD header field {key}"
                )
            fields[key] = tokens[1:]
            if key == "DATA":
                break

    required = ("WIDTH", "HEIGHT", "POINTS", "DATA")
    missing = [key for key in required if key not in fields]
    if missing:
        raise ContractError(
            f"{path}: missing PCD header field(s): {', '.join(missing)}"
        )

    width = _parse_nonnegative_int(path, "WIDTH", fields["WIDTH"])
    height = _parse_nonnegative_int(path, "HEIGHT", fields["HEIGHT"])
    points = _parse_nonnegative_int(path, "POINTS", fields["POINTS"])
    if height <= 0:
        raise ContractError(f"{path}: HEIGHT must be positive")
    if width * height != points:
        raise ContractError(
            f"{path}: WIDTH*HEIGHT is {width * height}, but POINTS is {points}"
        )

    data_values = fields["DATA"]
    if len(data_values) != 1:
        raise ContractError(f"{path}: DATA must contain exactly one encoding")
    data = data_values[0].lower()
    if data not in {"ascii", "binary", "binary_compressed"}:
        raise ContractError(f"{path}: unsupported PCD DATA encoding {data!r}")
    return PcdHeader(width=width, height=height, points=points, data=data)


def _format_indices(indices: Iterable[int]) -> str:
    values = sorted(indices)
    shown = ", ".join(str(value) for value in values[:12])
    if len(values) > 12:
        shown += f", ... ({len(values)} total)"
    return shown


def _validate_key_frame_files(map_dir: Path, key_frame_count: int) -> None:
    pcd_dir = map_dir / "pcd"
    if not pcd_dir.is_dir():
        raise ContractError(f"missing key-frame directory: {pcd_dir}")

    found = {kind: {} for kind in KEY_FRAME_SUFFIXES}
    for path in pcd_dir.iterdir():
        if not path.is_file():
            continue
        match = KEY_FRAME_PATTERN.fullmatch(path.name)
        if match is None:
            continue
        kind = match.group("kind")
        if kind not in found:
            continue
        index_text = match.group("index")
        index = int(index_text)
        if index_text != str(index):
            raise ContractError(
                f"{pcd_dir}: non-canonical key-frame name {path.name}; "
                f"expected {index}_{kind}.pcd"
            )
        if index in found[kind]:
            raise ContractError(
                f"{pcd_dir}: duplicate {kind} key-frame index {index} "
                f"({found[kind][index].name}, {path.name})"
            )
        found[kind][index] = path

    expected = set(range(key_frame_count))
    for kind in KEY_FRAME_SUFFIXES:
        actual = set(found[kind])
        missing = expected - actual
        extra = actual - expected
        if missing or extra:
            details = []
            if missing:
                details.append(f"missing indices [{_format_indices(missing)}]")
            if extra:
                details.append(
                    f"unexpected indices [{_format_indices(extra)}]"
                )
            raise ContractError(
                f"{pcd_dir}: {kind} key-frame set does not match poses.pcd: "
                + "; ".join(details)
            )
        for index in range(key_frame_count):
            read_pcd_header(found[kind][index])


def recommended_static_layer_timeout(ground_point_count: int) -> int:
    """Scale only the fail-closed wait, never the map geometry."""

    scale = max(
        1,
        (
            ground_point_count
            + GROUND_POINTS_PER_TIMEOUT_STEP
            - 1
        )
        // GROUND_POINTS_PER_TIMEOUT_STEP,
    )
    return min(
        MAX_AUTOMATIC_STATIC_LAYER_TIMEOUT_SEC,
        scale * BASE_STATIC_LAYER_TIMEOUT_SEC,
    )


def inspect_pose_graph(map_dir: Path) -> PoseGraphContract:
    """Return the map contract after validating all required files."""

    map_dir = map_dir.resolve()
    if not map_dir.is_dir():
        raise ContractError(f"pose-graph map is not a directory: {map_dir}")

    poses = read_pcd_header(map_dir / "poses.pcd")
    if poses.height != 1:
        raise ContractError(f"{map_dir / 'poses.pcd'}: HEIGHT must be 1")
    if poses.points <= 0:
        raise ContractError(
            f"{map_dir / 'poses.pcd'}: POINTS must be positive"
        )

    map_cloud = read_pcd_header(map_dir / "map.pcd")
    ground = read_pcd_header(map_dir / "ground.pcd")
    edges = read_pcd_header(map_dir / "edges.pcd")
    if map_cloud.points <= 0:
        raise ContractError(
            f"{map_dir / 'map.pcd'}: POINTS must be positive"
        )
    if ground.points <= 0:
        raise ContractError(
            f"{map_dir / 'ground.pcd'}: POINTS must be positive"
        )

    _validate_key_frame_files(map_dir, poses.points)
    return PoseGraphContract(
        key_frame_count=poses.points,
        map_point_count=map_cloud.points,
        ground_point_count=ground.points,
        edge_point_count=edges.points,
        static_layer_timeout_sec=recommended_static_layer_timeout(
            ground.points
        ),
    )


def render_runtime_config(
    source: Path,
    destination: Path,
    pose_graph_dir: str,
    expected_key_frame_count: int,
) -> None:
    """Pair a map override with the exact MCL key-frame count."""

    if expected_key_frame_count <= 0:
        raise ContractError("expected_key_frame_count must be positive")
    if not source.is_file():
        raise ContractError(f"navigation config does not exist: {source}")

    targets = {
        ("map1", "pose_graph_dir"): json.dumps(pose_graph_dir),
        ("sub_maps", "expected_key_frame_count"): str(
            expected_key_frame_count
        ),
    }
    replacements = {target: 0 for target in targets}
    current_section = ""
    output = []
    for line in source.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if line and not line[0].isspace() and stripped.endswith(":"):
            current_section = stripped[:-1]

        replaced = False
        for (section, key), value in targets.items():
            if current_section != section:
                continue
            if stripped.split(":", 1)[0] != key:
                continue
            indent = line[: len(line) - len(line.lstrip())]
            output.append(f"{indent}{key}: {value}")
            replacements[(section, key)] += 1
            replaced = True
            break
        if not replaced:
            output.append(line)

    invalid = [
        f"{section}.{key} ({count} matches)"
        for (section, key), count in replacements.items()
        if count != 1
    ]
    if invalid:
        raise ContractError(
            "runtime config must contain every map contract field once: "
            + ", ".join(invalid)
        )
    destination.write_text("\n".join(output) + "\n", encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser(
        "inspect", help="validate a pose-graph map and print its contract"
    )
    inspect_parser.add_argument("--map-dir", required=True, type=Path)

    render_parser = subparsers.add_parser(
        "render-config", help="render the map-paired navigation config"
    )
    render_parser.add_argument("--source", required=True, type=Path)
    render_parser.add_argument("--destination", required=True, type=Path)
    render_parser.add_argument("--pose-graph-dir", required=True)
    render_parser.add_argument(
        "--expected-key-frame-count", required=True, type=int
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    try:
        if args.command == "inspect":
            contract = inspect_pose_graph(args.map_dir)
            print(f"KEY_FRAME_COUNT={contract.key_frame_count}")
            print(f"MAP_POINT_COUNT={contract.map_point_count}")
            print(f"GROUND_POINT_COUNT={contract.ground_point_count}")
            print(f"EDGE_POINT_COUNT={contract.edge_point_count}")
            print(
                "STATIC_LAYER_TIMEOUT_SEC="
                f"{contract.static_layer_timeout_sec}"
            )
        else:
            render_runtime_config(
                args.source,
                args.destination,
                args.pose_graph_dir,
                args.expected_key_frame_count,
            )
    except (ContractError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
