#!/usr/bin/env python3
"""No-ROS unit tests for pose_graph_pcd_cleaner.py."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT_PATH = Path(__file__).with_name("pose_graph_pcd_cleaner.py")
SPEC = importlib.util.spec_from_file_location("pose_graph_pcd_cleaner", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_pcd(path: Path, fields: tuple[str, ...], rows: list[tuple[float, ...]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS " + " ".join(fields),
                "SIZE " + " ".join("4" for _ in fields),
                "TYPE " + " ".join("F" for _ in fields),
                "COUNT " + " ".join("1" for _ in fields),
                f"WIDTH {len(rows)}",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                f"POINTS {len(rows)}",
                "DATA ascii",
                *(" ".join(str(value) for value in row) for row in rows),
                "",
            ]
        ),
        encoding="ascii",
    )


def create_pose_graph(root: Path) -> None:
    pose_fields = ("x", "y", "z", "intensity", "roll", "pitch", "yaw", "time")
    write_pcd(
        root / "poses.pcd",
        pose_fields,
        [
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            (10.0, 0.0, 0.0, 1.0, 0.0, 0.0, math.pi / 2.0, 2.0),
        ],
    )
    write_pcd(root / "edges.pcd", ("x", "y", "z"), [(0.0, 0.0, 1.0)])
    write_pcd(
        root / "map.pcd",
        ("x", "y", "z", "intensity"),
        [(1.0, 2.0, 0.5, 7.0), (8.0, 0.0, 0.5, 8.0), (20.0, 0.0, 0.5, 9.0)],
    )
    write_pcd(
        root / "ground.pcd",
        ("x", "y", "z", "intensity"),
        [(1.0, 2.0, 0.0, 0.0), (8.0, 0.0, 0.0, 0.0)],
    )
    layer_rows = {
        (0, "feature"): [(1.0, 2.0, 0.5, 7.0), (20.0, 0.0, 0.5, 9.0)],
        (0, "surface"): [(1.1, 2.0, 0.5, 17.0), (20.0, 1.0, 0.5, 19.0)],
        (0, "ground"): [(1.0, 2.0, 0.0, 0.0)],
        # Pose 1 rotates local (0, 2, z) to global (8, 0, z).
        (1, "feature"): [(0.0, 2.0, 0.5, 8.0), (0.0, -2.0, 0.5, 10.0)],
        (1, "surface"): [(0.0, 2.1, 0.5, 18.0), (0.0, -2.1, 0.5, 20.0)],
        (1, "ground"): [(0.0, 2.0, 0.0, 0.0)],
    }
    for (index, layer), rows in layer_rows.items():
        write_pcd(
            root / "pcd" / f"{index}_{layer}.pcd",
            ("x", "y", "z", "intensity"),
            rows,
        )


class BoxCleaningTest(unittest.TestCase):
    def test_box_maps_back_to_each_keyframe_and_preserves_protected_files(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            source = temp / "source"
            output = temp / "cleaned"
            create_pose_graph(source)
            poses = MODULE.validate_pose_graph(source, ("feature", "surface"))
            before = MODULE.protected_hashes(source, len(poses))
            selection = MODULE.BoxSelection((0.8, -0.2, 0.4), (8.2, 2.2, 0.6))

            report = MODULE.clean_copy(
                source, output, selection, ("feature", "surface")
            )

            self.assertEqual(report["layers"]["feature"]["removed"], 2)
            self.assertEqual(report["layers"]["surface"]["removed"], 2)
            self.assertEqual(report["top_level_map"]["removed"], 2)
            self.assertEqual(before, MODULE.protected_hashes(output, len(poses)))
            self.assertEqual(
                len(MODULE.PcdDocument.read(output / "pcd/0_feature.pcd").rows), 1
            )
            self.assertEqual(
                len(MODULE.PcdDocument.read(output / "pcd/1_feature.pcd").rows), 1
            )
            saved_report = json.loads(
                (output / "pose_graph_cleaning_report.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertTrue(saved_report["protected_files"]["unchanged"])

    def test_existing_output_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            source = temp / "source"
            output = temp / "cleaned"
            create_pose_graph(source)
            output.mkdir()
            with self.assertRaisesRegex(ValueError, "already exists"):
                MODULE.clean_copy(
                    source,
                    output,
                    MODULE.BoxSelection((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
                    ("feature", "surface"),
                )

    def test_symbolic_link_in_source_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            source = temp / "source"
            create_pose_graph(source)
            (source / "unexpected_link").symlink_to(source / "map.pcd")

            with self.assertRaisesRegex(ValueError, "symbolic links"):
                MODULE.clean_copy(
                    source,
                    temp / "cleaned",
                    MODULE.BoxSelection((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
                    ("feature", "surface"),
                )

    def test_edited_files_keep_original_permissions(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            source = temp / "source"
            output = temp / "cleaned"
            create_pose_graph(source)
            edited_path = source / "pcd/0_feature.pcd"
            edited_path.chmod(0o640)

            MODULE.clean_copy(
                source,
                output,
                MODULE.BoxSelection((0.9, 1.9, 0.4), (1.2, 2.1, 0.6)),
                ("feature", "surface"),
            )

            self.assertEqual((output / "pcd/0_feature.pcd").stat().st_mode & 0o777, 0o640)


class RvizVoxelSelectionTest(unittest.TestCase):
    def test_selected_voxel_removes_raw_points_from_multiple_layers(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            source = temp / "source"
            output = temp / "cleaned"
            selection_path = temp / "deleted_points.pcd"
            create_pose_graph(source)
            write_pcd(
                selection_path,
                ("x", "y", "z"),
                [(1.04, 2.04, 0.54)],
            )
            selection_document = MODULE.PcdDocument.read(selection_path)
            voxelizer = MODULE.VoxelSelection(
                0.1, frozenset(), selection_path, 1
            )
            voxels = frozenset(
                voxelizer.voxel(selection_document.xyz(row))
                for row in selection_document.rows
            )
            selection = MODULE.VoxelSelection(0.1, voxels, selection_path, 1)

            report = MODULE.clean_copy(
                source, output, selection, ("feature", "surface")
            )

            self.assertEqual(report["layers"]["feature"]["removed"], 1)
            self.assertEqual(report["layers"]["surface"]["removed"], 0)
            retained = MODULE.PcdDocument.read(output / "pcd/0_feature.pcd")
            self.assertEqual(len(retained.rows), 1)
            self.assertEqual(float(retained.rows[0][3]), 9.0)


class EditorExportTest(unittest.TestCase):
    def test_export_contains_global_feature_surface_and_ground(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            source = temp / "source"
            output = temp / "editor"
            create_pose_graph(source)

            metadata = MODULE.export_editor_clouds(
                source, output, ("feature", "surface")
            )

            self.assertEqual(metadata["map_points"], 8)
            self.assertEqual(metadata["ground_points"], 2)
            editor = MODULE.PcdDocument.read(output / "editor_map.pcd")
            points = [editor.xyz(row) for row in editor.rows]
            self.assertTrue(
                any(
                    abs(x - 8.0) < 1.0e-6
                    and abs(y) < 1.0e-6
                    and abs(z - 0.5) < 1.0e-6
                    for x, y, z in points
                )
            )


if __name__ == "__main__":
    unittest.main()
