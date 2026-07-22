#!/usr/bin/env python3
"""Safely remove globally selected points from a DDDMR pose-graph copy.

DDDMR stores feature, surface, and ground clouds in each keyframe's local
``base_link`` frame.  A normal PCD editor only changes an aggregate map and
therefore does not affect the keyframe files consumed by
``dddmr_pg_map_server``.  This tool maps a global selection back through every
saved pose and writes a new pose-graph directory.

The source directory is never modified.  Ground clouds, poses, and graph edges
are protected by SHA-256 verification.  Only ASCII PCD files are supported so
that every retained row and non-coordinate field can be preserved exactly.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import tempfile
from typing import Iterable, Iterator, Sequence


REPORT_SCHEMA = "dddmr-pose-graph-cleaning/v1"
EDITABLE_LAYERS = ("feature", "surface")
PROTECTED_TOP_LEVEL_FILES = ("poses.pcd", "edges.pcd", "ground.pcd")
DEFAULT_SELECTION_VOXEL_SIZE = 0.10


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    rotation: tuple[tuple[float, float, float], ...]

    def transform(self, point: tuple[float, float, float]) -> tuple[float, ...]:
        px, py, pz = point
        rotation = self.rotation
        return (
            rotation[0][0] * px
            + rotation[0][1] * py
            + rotation[0][2] * pz
            + self.x,
            rotation[1][0] * px
            + rotation[1][1] * py
            + rotation[1][2] * pz
            + self.y,
            rotation[2][0] * px
            + rotation[2][1] * py
            + rotation[2][2] * pz
            + self.z,
        )


@dataclass
class PcdDocument:
    header_lines: list[str]
    fields: tuple[str, ...]
    rows: list[tuple[str, ...]]
    xyz_indices: tuple[int, int, int]

    @classmethod
    def read(cls, path: Path) -> "PcdDocument":
        header_lines: list[str] = []
        fields: tuple[str, ...] = ()
        counts: tuple[int, ...] = ()
        rows: list[tuple[str, ...]] = []
        data_seen = False

        try:
            stream = path.open("r", encoding="ascii")
        except (OSError, UnicodeDecodeError) as exc:
            raise ValueError(f"cannot read ASCII PCD {path}: {exc}") from exc

        with stream:
            for line_number, raw_line in enumerate(stream, start=1):
                line = raw_line.rstrip("\r\n")
                if not data_seen:
                    header_lines.append(line)
                    tokens = line.split()
                    if not tokens or tokens[0].startswith("#"):
                        continue
                    keyword = tokens[0].upper()
                    if keyword in {"FIELDS", "COLUMNS"}:
                        fields = tuple(tokens[1:])
                    elif keyword == "COUNT":
                        try:
                            counts = tuple(int(value) for value in tokens[1:])
                        except ValueError as exc:
                            raise ValueError(
                                f"{path}:{line_number}: invalid COUNT header"
                            ) from exc
                    elif keyword == "DATA":
                        if len(tokens) != 2 or tokens[1].lower() != "ascii":
                            kind = tokens[1] if len(tokens) > 1 else "missing"
                            raise ValueError(
                                f"{path} uses unsupported PCD DATA {kind}; "
                                "only DATA ascii is supported"
                            )
                        data_seen = True
                    continue

                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                values = tuple(stripped.split())
                if len(values) != len(fields):
                    raise ValueError(
                        f"{path}:{line_number}: found {len(values)} values; "
                        f"expected {len(fields)}"
                    )
                rows.append(values)

        if not data_seen:
            raise ValueError(f"{path} has no DATA ascii header")
        if not fields:
            raise ValueError(f"{path} has no FIELDS header")
        if counts and (len(counts) != len(fields) or any(value != 1 for value in counts)):
            raise ValueError(f"{path} uses unsupported multi-count PCD fields")
        try:
            xyz_indices = tuple(fields.index(axis) for axis in ("x", "y", "z"))
        except ValueError as exc:
            raise ValueError(f"{path} is missing an x, y, or z field") from exc

        document = cls(header_lines, fields, rows, xyz_indices)
        document.validate_finite_xyz(path)
        return document

    def validate_finite_xyz(self, path: Path) -> None:
        for row_number, row in enumerate(self.rows, start=1):
            try:
                xyz = tuple(float(row[index]) for index in self.xyz_indices)
            except ValueError as exc:
                raise ValueError(
                    f"{path}: data row {row_number} has a non-numeric coordinate"
                ) from exc
            if not all(math.isfinite(value) for value in xyz):
                raise ValueError(
                    f"{path}: data row {row_number} has a non-finite coordinate"
                )

    def xyz(self, row: Sequence[str]) -> tuple[float, float, float]:
        return tuple(float(row[index]) for index in self.xyz_indices)  # type: ignore[return-value]

    def field_float(self, row: Sequence[str], name: str, default: float) -> float:
        if name not in self.fields:
            return default
        value = float(row[self.fields.index(name)])
        if not math.isfinite(value):
            raise ValueError(f"PCD field {name} is non-finite")
        return value

    def with_rows(self, rows: Iterable[tuple[str, ...]]) -> "PcdDocument":
        return PcdDocument(
            list(self.header_lines), self.fields, list(rows), self.xyz_indices
        )

    def write_atomic(self, path: Path) -> None:
        original_mode = stat.S_IMODE(path.stat().st_mode)
        height = 1
        for line in self.header_lines:
            tokens = line.split()
            if tokens and tokens[0].upper() == "HEIGHT":
                if len(tokens) != 2:
                    raise ValueError(f"{path}: malformed HEIGHT header")
                height = int(tokens[1])
        if height != 1:
            raise ValueError(f"{path}: organized PCD clouds are not supported")

        rendered: list[str] = []
        width_seen = False
        points_seen = False
        for line in self.header_lines:
            tokens = line.split()
            keyword = tokens[0].upper() if tokens else ""
            if keyword == "WIDTH":
                rendered.append(f"WIDTH {len(self.rows)}")
                width_seen = True
            elif keyword == "POINTS":
                rendered.append(f"POINTS {len(self.rows)}")
                points_seen = True
            else:
                rendered.append(line)
        if not width_seen or not points_seen:
            raise ValueError(f"{path}: WIDTH or POINTS header is missing")

        descriptor = None
        try:
            descriptor, temporary_name = tempfile.mkstemp(
                prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
            )
            with os.fdopen(descriptor, "w", encoding="ascii", newline="\n") as stream:
                descriptor = None
                os.fchmod(stream.fileno(), original_mode)
                for line in rendered:
                    stream.write(line)
                    stream.write("\n")
                for row in self.rows:
                    stream.write(" ".join(row))
                    stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
        finally:
            if descriptor is not None:
                os.close(descriptor)
            if "temporary_name" in locals() and os.path.exists(temporary_name):
                os.unlink(temporary_name)


class Selection:
    def matches(self, point: tuple[float, float, float]) -> bool:
        raise NotImplementedError

    def description(self) -> dict[str, object]:
        raise NotImplementedError


@dataclass(frozen=True)
class BoxSelection(Selection):
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    def matches(self, point: tuple[float, float, float]) -> bool:
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(point, self.minimum, self.maximum)
        )

    def description(self) -> dict[str, object]:
        return {
            "kind": "axis_aligned_box",
            "minimum": list(self.minimum),
            "maximum": list(self.maximum),
        }


@dataclass(frozen=True)
class VoxelSelection(Selection):
    voxel_size: float
    selected_voxels: frozenset[tuple[int, int, int]]
    selection_path: Path
    selected_point_count: int

    def voxel(self, point: tuple[float, float, float]) -> tuple[int, int, int]:
        return tuple(math.floor(value / self.voxel_size) for value in point)  # type: ignore[return-value]

    def matches(self, point: tuple[float, float, float]) -> bool:
        return self.voxel(point) in self.selected_voxels

    def description(self) -> dict[str, object]:
        return {
            "kind": "rviz_selected_voxels",
            "selection_pcd": str(self.selection_path),
            "selection_voxel_size": self.voxel_size,
            "selected_points": self.selected_point_count,
            "selected_voxels": len(self.selected_voxels),
        }


def rotation_matrix(
    roll: float, pitch: float, yaw: float
) -> tuple[tuple[float, float, float], ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def load_poses(map_dir: Path) -> list[Pose]:
    path = map_dir / "poses.pcd"
    document = PcdDocument.read(path)
    required = ("x", "y", "z", "roll", "pitch", "yaw")
    missing = [name for name in required if name not in document.fields]
    if missing:
        raise ValueError(f"{path} is missing pose fields: {missing}")
    indices = {name: document.fields.index(name) for name in required}
    poses: list[Pose] = []
    for row_number, row in enumerate(document.rows):
        values = {name: float(row[index]) for name, index in indices.items()}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError(f"{path}: pose {row_number} contains a non-finite value")
        poses.append(
            Pose(
                values["x"],
                values["y"],
                values["z"],
                rotation_matrix(values["roll"], values["pitch"], values["yaw"]),
            )
        )
    if not poses:
        raise ValueError(f"{path} contains no poses")
    return poses


def validate_pose_graph(map_dir: Path, layers: Sequence[str]) -> list[Pose]:
    if not map_dir.is_dir():
        raise ValueError(f"pose-graph directory does not exist: {map_dir}")
    symlinks = [path for path in map_dir.rglob("*") if path.is_symlink()]
    if symlinks:
        preview = ", ".join(str(path.relative_to(map_dir)) for path in symlinks[:5])
        suffix = "" if len(symlinks) <= 5 else ", ..."
        raise ValueError(
            "pose-graph directory contains symbolic links; refusing an edit "
            f"that could escape the map copy: {preview}{suffix}"
        )
    for name in ("poses.pcd", "edges.pcd", "map.pcd", "ground.pcd"):
        if not (map_dir / name).is_file():
            raise ValueError(f"pose-graph file is missing: {map_dir / name}")
    poses = load_poses(map_dir)
    pcd_dir = map_dir / "pcd"
    if not pcd_dir.is_dir():
        raise ValueError(f"pose-graph keyframe directory is missing: {pcd_dir}")
    for index in range(len(poses)):
        for layer in tuple(layers) + ("ground",):
            path = pcd_dir / f"{index}_{layer}.pcd"
            if not path.is_file():
                raise ValueError(f"pose-graph keyframe file is missing: {path}")
    return poses


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def protected_hashes(map_dir: Path, pose_count: int) -> dict[str, str]:
    relative_paths = list(PROTECTED_TOP_LEVEL_FILES)
    relative_paths.extend(f"pcd/{index}_ground.pcd" for index in range(pose_count))
    return {name: sha256_file(map_dir / name) for name in relative_paths}


def make_selection(args: argparse.Namespace) -> Selection:
    if args.box is not None:
        values = tuple(float(value) for value in args.box)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("box bounds must be finite")
        minimum = (values[0], values[2], values[4])
        maximum = (values[1], values[3], values[5])
        if any(lower >= upper for lower, upper in zip(minimum, maximum)):
            raise ValueError("each box minimum must be smaller than its maximum")
        return BoxSelection(minimum, maximum)

    selection_path = args.selection_pcd.resolve()
    voxel_size = float(args.selection_voxel_size)
    if not math.isfinite(voxel_size) or voxel_size <= 0.0:
        raise ValueError("selection voxel size must be finite and positive")
    document = PcdDocument.read(selection_path)
    if not document.rows:
        raise ValueError(f"selection PCD is empty: {selection_path}")
    voxelizer = VoxelSelection(voxel_size, frozenset(), selection_path, 0)
    voxels = frozenset(voxelizer.voxel(document.xyz(row)) for row in document.rows)
    return VoxelSelection(voxel_size, voxels, selection_path, len(document.rows))


def point_stats(
    map_dir: Path,
    poses: Sequence[Pose],
    selection: Selection,
    layers: Sequence[str],
) -> dict[str, object]:
    layer_reports: dict[str, object] = {}
    for layer in layers:
        total = 0
        selected = 0
        touched: dict[str, int] = {}
        for index, pose in enumerate(poses):
            document = PcdDocument.read(map_dir / "pcd" / f"{index}_{layer}.pcd")
            count = 0
            for row in document.rows:
                total += 1
                if selection.matches(pose.transform(document.xyz(row))):
                    selected += 1
                    count += 1
            if count:
                touched[str(index)] = count
        layer_reports[layer] = {
            "before": total,
            "selected": selected,
            "touched_keyframes": touched,
        }
    return layer_reports


def filter_document(
    document: PcdDocument,
    pose: Pose | None,
    selection: Selection,
) -> tuple[PcdDocument, int]:
    retained: list[tuple[str, ...]] = []
    removed = 0
    for row in document.rows:
        point = document.xyz(row)
        global_point = pose.transform(point) if pose is not None else point
        if selection.matches(global_point):
            removed += 1
        else:
            retained.append(row)
    return document.with_rows(retained), removed


def clean_copy(
    source: Path,
    output: Path,
    selection: Selection,
    layers: Sequence[str],
) -> dict[str, object]:
    source = source.resolve()
    output = output.resolve()
    if source == output:
        raise ValueError("source and output pose-graph directories must differ")
    if source in output.parents:
        raise ValueError("output directory must not be inside the source directory")
    if output.exists():
        raise ValueError(f"output directory already exists: {output}")

    poses = validate_pose_graph(source, layers)
    before_protected = protected_hashes(source, len(poses))
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f".{output.name}.staging-", dir=output.parent)
    )
    completed = False
    try:
        shutil.copytree(source, staging, dirs_exist_ok=True)
        reports: dict[str, object] = {}
        total_removed = 0
        for layer in layers:
            layer_before = 0
            layer_removed = 0
            touched: dict[str, int] = {}
            for index, pose in enumerate(poses):
                path = staging / "pcd" / f"{index}_{layer}.pcd"
                document = PcdDocument.read(path)
                layer_before += len(document.rows)
                filtered, removed = filter_document(document, pose, selection)
                if removed:
                    filtered.write_atomic(path)
                    touched[str(index)] = removed
                    layer_removed += removed
            reports[layer] = {
                "before": layer_before,
                "removed": layer_removed,
                "after": layer_before - layer_removed,
                "touched_keyframes": touched,
            }
            total_removed += layer_removed

        top_level_path = staging / "map.pcd"
        top_level = PcdDocument.read(top_level_path)
        filtered_top_level, top_level_removed = filter_document(
            top_level, None, selection
        )
        if top_level_removed:
            filtered_top_level.write_atomic(top_level_path)

        after_protected = protected_hashes(staging, len(poses))
        if before_protected != after_protected:
            changed = sorted(
                name
                for name, digest in before_protected.items()
                if after_protected.get(name) != digest
            )
            raise RuntimeError(f"protected pose-graph files changed: {changed}")
        if total_removed == 0:
            raise ValueError("selection removed no feature or surface points")

        residual = point_stats(staging, poses, selection, layers)
        residual_counts = {
            layer: int(report["selected"])
            for layer, report in residual.items()
        }
        if any(residual_counts.values()):
            raise RuntimeError(f"selected points remain after filtering: {residual_counts}")

        report: dict[str, object] = {
            "schema": REPORT_SCHEMA,
            "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "source": str(source),
            "output": str(output),
            "pose_count": len(poses),
            "editable_layers": list(layers),
            "selection": selection.description(),
            "layers": reports,
            "top_level_map": {
                "before": len(top_level.rows),
                "removed": top_level_removed,
                "after": len(top_level.rows) - top_level_removed,
            },
            "protected_files": {
                "count": len(before_protected),
                "unchanged": True,
                "sha256": before_protected,
            },
            "residual_selected_points": residual_counts,
        }
        report_path = staging / "pose_graph_cleaning_report.json"
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.rename(staging, output)
        completed = True
        return report
    finally:
        if not completed and staging.exists():
            shutil.rmtree(staging)


def global_rows(
    map_dir: Path,
    poses: Sequence[Pose],
    layers: Sequence[str],
) -> Iterator[tuple[float, float, float, float]]:
    for layer in layers:
        for index, pose in enumerate(poses):
            document = PcdDocument.read(map_dir / "pcd" / f"{index}_{layer}.pcd")
            for row in document.rows:
                x, y, z = pose.transform(document.xyz(row))
                yield x, y, z, document.field_float(row, "intensity", 0.0)


def write_global_pcd(
    path: Path, rows: Iterable[tuple[float, float, float, float]]
) -> int:
    body_descriptor, body_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".body", dir=path.parent
    )
    count = 0
    try:
        with os.fdopen(body_descriptor, "w", encoding="ascii", newline="\n") as body:
            body_descriptor = -1
            for row in rows:
                body.write(" ".join(format(value, ".9g") for value in row))
                body.write("\n")
                count += 1
        with path.open("w", encoding="ascii", newline="\n") as output:
            output.write("# .PCD v0.7 - Point Cloud Data file format\n")
            output.write("VERSION 0.7\n")
            output.write("FIELDS x y z intensity\n")
            output.write("SIZE 4 4 4 4\n")
            output.write("TYPE F F F F\n")
            output.write("COUNT 1 1 1 1\n")
            output.write(f"WIDTH {count}\n")
            output.write("HEIGHT 1\n")
            output.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            output.write(f"POINTS {count}\n")
            output.write("DATA ascii\n")
            with open(body_name, "r", encoding="ascii") as body:
                shutil.copyfileobj(body, output)
        return count
    finally:
        if body_descriptor >= 0:
            os.close(body_descriptor)
        if os.path.exists(body_name):
            os.unlink(body_name)


def export_editor_clouds(
    map_dir: Path, output: Path, layers: Sequence[str]
) -> dict[str, object]:
    map_dir = map_dir.resolve()
    output = output.resolve()
    if output.exists():
        raise ValueError(f"editor output directory already exists: {output}")
    if map_dir in output.parents:
        raise ValueError("editor output must not be inside the pose-graph directory")
    poses = validate_pose_graph(map_dir, layers)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f".{output.name}.staging-", dir=output.parent)
    )
    completed = False
    try:
        map_count = write_global_pcd(
            staging / "editor_map.pcd", global_rows(map_dir, poses, layers)
        )
        ground_count = write_global_pcd(
            staging / "editor_ground.pcd", global_rows(map_dir, poses, ("ground",))
        )
        metadata: dict[str, object] = {
            "schema": REPORT_SCHEMA,
            "source": str(map_dir),
            "pose_count": len(poses),
            "map_layers": list(layers),
            "map_points": map_count,
            "ground_points": ground_count,
            "recommended_selection_voxel_size": DEFAULT_SELECTION_VOXEL_SIZE,
        }
        (staging / "editor_clouds.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.rename(staging, output)
        completed = True
        return metadata
    finally:
        if not completed and staging.exists():
            shutil.rmtree(staging)


def add_selection_arguments(parser: argparse.ArgumentParser) -> None:
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--box",
        nargs=6,
        type=float,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"),
        help="global map-frame axis-aligned deletion box",
    )
    group.add_argument(
        "--selection-pcd",
        type=Path,
        help="RViz-exported deleted_points PCD in the map frame",
    )
    parser.add_argument(
        "--selection-voxel-size",
        type=float,
        default=DEFAULT_SELECTION_VOXEL_SIZE,
        help="voxel size used by the RViz editor map publisher (default: 0.10)",
    )


def add_layer_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--layers",
        nargs="+",
        choices=EDITABLE_LAYERS,
        default=list(EDITABLE_LAYERS),
        help="keyframe layers allowed to change; ground is never editable",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser(
        "inspect", help="report how a global selection maps to keyframes"
    )
    inspect_parser.add_argument("--map-dir", type=Path, required=True)
    add_selection_arguments(inspect_parser)
    add_layer_argument(inspect_parser)

    clean_parser = subparsers.add_parser(
        "clean-copy", help="atomically create and verify a cleaned pose-graph copy"
    )
    clean_parser.add_argument("--source", type=Path, required=True)
    clean_parser.add_argument("--output", type=Path, required=True)
    add_selection_arguments(clean_parser)
    add_layer_argument(clean_parser)

    export_parser = subparsers.add_parser(
        "export-editor", help="export global feature/surface and ground clouds for RViz"
    )
    export_parser.add_argument("--map-dir", type=Path, required=True)
    export_parser.add_argument("--output", type=Path, required=True)
    add_layer_argument(export_parser)
    return parser


def print_layer_stats(stats: dict[str, object], selected_key: str) -> None:
    for layer, raw_report in stats.items():
        report = raw_report
        print(f"{layer.upper()}_POINTS_BEFORE={report['before']}")
        print(f"{layer.upper()}_POINTS_{selected_key.upper()}={report[selected_key]}")
        print(f"{layer.upper()}_KEYFRAMES_TOUCHED={len(report['touched_keyframes'])}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        layers = tuple(dict.fromkeys(args.layers))
        if args.command == "export-editor":
            metadata = export_editor_clouds(args.map_dir, args.output, layers)
            print(f"EDITOR_OUTPUT={args.output.resolve()}")
            print(f"EDITOR_MAP_POINTS={metadata['map_points']}")
            print(f"EDITOR_GROUND_POINTS={metadata['ground_points']}")
            print("POSE_GRAPH_EDITOR_EXPORT_STATUS=PASS")
            return 0

        selection = make_selection(args)
        if args.command == "inspect":
            map_dir = args.map_dir.resolve()
            poses = validate_pose_graph(map_dir, layers)
            stats = point_stats(map_dir, poses, selection, layers)
            print(f"POSE_GRAPH_DIR={map_dir}")
            print(f"POSE_COUNT={len(poses)}")
            print_layer_stats(stats, "selected")
            print("POSE_GRAPH_SELECTION_INSPECTION_STATUS=PASS")
            return 0

        report = clean_copy(args.source, args.output, selection, layers)
        print(f"POSE_GRAPH_SOURCE={report['source']}")
        print(f"POSE_GRAPH_OUTPUT={report['output']}")
        print(f"POSE_COUNT={report['pose_count']}")
        print_layer_stats(report["layers"], "removed")
        print(f"TOP_LEVEL_MAP_POINTS_REMOVED={report['top_level_map']['removed']}")
        print("PROTECTED_GROUND_POSES_EDGES_UNCHANGED=PASS")
        print("POSE_GRAPH_CLEAN_COPY_STATUS=PASS")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"ERROR: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
