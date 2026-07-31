#!/usr/bin/env python3
"""Tests for the Go2 pose-graph map contract helper."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Optional


SCRIPT_DIR = Path(__file__).resolve().parent
MODULE_PATH = SCRIPT_DIR / "go2_pose_graph_map_contract.py"
SPEC = importlib.util.spec_from_file_location(
    "go2_pose_graph_map_contract", MODULE_PATH
)
assert SPEC is not None
assert SPEC.loader is not None
CONTRACT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CONTRACT
SPEC.loader.exec_module(CONTRACT)


def write_pcd(path: Path, points: int, *, width: Optional[int] = None) -> None:
    if width is None:
        width = points
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            (
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z intensity",
                "SIZE 4 4 4 4",
                "TYPE F F F F",
                "COUNT 1 1 1 1",
                f"WIDTH {width}",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                f"POINTS {points}",
                "DATA ascii",
            )
        )
        + "\n",
        encoding="ascii",
    )


def make_pose_graph(root: Path, key_frames: int = 3) -> Path:
    map_dir = root / "map"
    write_pcd(map_dir / "poses.pcd", key_frames)
    write_pcd(map_dir / "map.pcd", 120)
    write_pcd(map_dir / "ground.pcd", 450)
    write_pcd(map_dir / "edges.pcd", 0)
    for index in range(key_frames):
        for kind in CONTRACT.KEY_FRAME_SUFFIXES:
            write_pcd(map_dir / "pcd" / f"{index}_{kind}.pcd", index + 1)
    return map_dir


class PoseGraphContractTest(unittest.TestCase):
    def test_valid_map_reports_exact_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            contract = CONTRACT.inspect_pose_graph(
                make_pose_graph(Path(temporary), key_frames=3)
            )
        self.assertEqual(contract.key_frame_count, 3)
        self.assertEqual(contract.map_point_count, 120)
        self.assertEqual(contract.ground_point_count, 450)
        self.assertEqual(contract.edge_point_count, 0)
        self.assertEqual(contract.static_layer_timeout_sec, 90)

    def test_dense_map_scales_only_the_readiness_timeout(self) -> None:
        self.assertEqual(
            CONTRACT.recommended_static_layer_timeout(469816), 630
        )
        self.assertEqual(
            CONTRACT.recommended_static_layer_timeout(10000000), 900
        )

    def test_missing_key_frame_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            map_dir = make_pose_graph(Path(temporary), key_frames=3)
            (map_dir / "pcd" / "1_surface.pcd").unlink()
            with self.assertRaisesRegex(
                CONTRACT.ContractError,
                "surface key-frame set.*missing indices",
            ):
                CONTRACT.inspect_pose_graph(map_dir)

    def test_extra_key_frame_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            map_dir = make_pose_graph(Path(temporary), key_frames=2)
            write_pcd(map_dir / "pcd" / "2_ground.pcd", 1)
            with self.assertRaisesRegex(
                CONTRACT.ContractError,
                "ground key-frame set.*unexpected indices",
            ):
                CONTRACT.inspect_pose_graph(map_dir)

    def test_noncanonical_key_frame_name_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            map_dir = make_pose_graph(Path(temporary), key_frames=2)
            canonical = map_dir / "pcd" / "0_feature.pcd"
            canonical.rename(map_dir / "pcd" / "00_feature.pcd")
            with self.assertRaisesRegex(
                CONTRACT.ContractError, "non-canonical key-frame name"
            ):
                CONTRACT.inspect_pose_graph(map_dir)

    def test_inconsistent_pcd_dimensions_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            map_dir = make_pose_graph(Path(temporary), key_frames=2)
            write_pcd(map_dir / "poses.pcd", 2, width=3)
            with self.assertRaisesRegex(
                CONTRACT.ContractError, r"WIDTH[*]HEIGHT"
            ):
                CONTRACT.inspect_pose_graph(map_dir)

    def test_runtime_config_pairs_path_and_key_frame_count(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.yaml"
            destination = root / "runtime.yaml"
            source.write_text(
                """
map1:
  ros__parameters:
    pose_graph_dir: "/old/map"
sub_maps:
  ros__parameters:
    expected_key_frame_count: 169
unrelated:
  ros__parameters:
    expected_key_frame_count: 7
""".lstrip(),
                encoding="utf-8",
            )
            CONTRACT.render_runtime_config(
                source, destination, "/root/dddmr_bags/new_map", 132
            )
            rendered = destination.read_text(encoding="utf-8")
        self.assertIn('pose_graph_dir: "/root/dddmr_bags/new_map"', rendered)
        self.assertIn("    expected_key_frame_count: 132", rendered)
        self.assertIn("    expected_key_frame_count: 7", rendered)
        self.assertNotIn('pose_graph_dir: "/old/map"', rendered)

    def test_runtime_config_requires_both_contract_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.yaml"
            destination = root / "runtime.yaml"
            source.write_text(
                """
map1:
  ros__parameters:
    pose_graph_dir: "/old/map"
sub_maps:
  ros__parameters:
    sub_map_search_radius: 50.0
""".lstrip(),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                CONTRACT.ContractError, "sub_maps.expected_key_frame_count"
            ):
                CONTRACT.render_runtime_config(
                    source, destination, "/root/dddmr_bags/new_map", 132
                )


if __name__ == "__main__":
    unittest.main()
