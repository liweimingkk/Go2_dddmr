#!/usr/bin/env python3
"""Approximate DDDMR static-layer connectivity on a saved pose-graph map."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


DEFAULT_MAP = (
    Path(__file__).resolve().parents[2]
    / "bags/go2_xt16_mouth_mapping_20260709_130235_map_2026_07_09_05_02_34"
)


def read_ascii_pcd(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    data = False
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("DATA"):
                if line != "DATA ascii":
                    raise ValueError(f"only ASCII PCD is supported: {path}")
                data = True
                continue
            if data:
                rows.append([float(value) for value in line.split()])
    return np.asarray(rows, dtype=np.float64)


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def merge_keyframes(map_dir: Path, suffix: str) -> np.ndarray:
    poses = read_ascii_pcd(map_dir / "poses.pcd")
    clouds: list[np.ndarray] = []
    for index, pose in enumerate(poses):
        path = map_dir / "pcd" / f"{index}_{suffix}.pcd"
        if not path.is_file():
            continue
        cloud = read_ascii_pcd(path)
        if cloud.size == 0:
            continue
        rotation = rotation_matrix(pose[4], pose[5], pose[6])
        transformed = cloud.copy()
        transformed[:, :3] = cloud[:, :3] @ rotation.T + pose[:3]
        clouds.append(transformed)
    if not clouds:
        return np.empty((0, 4), dtype=np.float64)
    return np.concatenate(clouds)


class UnionFind:
    def __init__(self, size: int) -> None:
        self.parent = np.arange(size, dtype=np.int64)
        self.rank = np.zeros(size, dtype=np.int8)

    def find(self, value: int) -> int:
        parent = self.parent
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = int(parent[value])
        return value

    def union(self, left: int, right: int) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return
        if self.rank[left_root] < self.rank[right_root]:
            left_root, right_root = right_root, left_root
        self.parent[right_root] = left_root
        if self.rank[left_root] == self.rank[right_root]:
            self.rank[left_root] += 1


def keep_euclidean_clusters(points: np.ndarray, tolerance: float, minimum: int) -> np.ndarray:
    if points.size == 0:
        return points
    pairs = cKDTree(points[:, :3]).query_pairs(tolerance, output_type="ndarray")
    groups = UnionFind(len(points))
    for left, right in pairs:
        groups.union(int(left), int(right))
    roots = np.asarray([groups.find(index) for index in range(len(points))])
    _, inverse, counts = np.unique(roots, return_inverse=True, return_counts=True)
    return points[counts[inverse] >= minimum]


def voxel_centroids(points: np.ndarray, leaf: float) -> np.ndarray:
    if points.size == 0:
        return points
    keys = np.floor(points[:, :3] / leaf).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((int(inverse.max()) + 1, points.shape[1]), dtype=np.float64)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse)
    return sums / counts[:, None]


def static_distances(
    map_points: np.ndarray,
    ground: np.ndarray,
    imposing_radius: float,
    xy_radius: float,
    minimum_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    tree = cKDTree(map_points[:, :3])
    distances = np.full(len(ground), np.inf, dtype=np.float64)
    counts = np.zeros(len(ground), dtype=np.int64)
    for index, node in enumerate(ground[:, :3]):
        candidate_ids = tree.query_ball_point(node, imposing_radius)
        if not candidate_ids:
            continue
        candidates = map_points[np.asarray(candidate_ids, dtype=np.int64), :3]
        delta = candidates - node
        mask = (
            (delta[:, 2] >= 0.1)
            & (delta[:, 2] <= 1.0)
            & (np.abs(delta[:, 0]) <= xy_radius)
            & (np.abs(delta[:, 1]) <= xy_radius)
        )
        accepted = delta[mask]
        counts[index] = len(accepted)
        if len(accepted) >= minimum_points:
            distances[index] = float(np.min(np.hypot(accepted[:, 0], accepted[:, 1])))
    return distances, counts


def connectivity(
    ground: np.ndarray,
    free: np.ndarray,
    start_xyz: np.ndarray,
    expanding_radius: float,
) -> tuple[int, int, int, float]:
    tree = cKDTree(ground[:, :3])
    groups = UnionFind(len(ground))
    for index, node in enumerate(ground[:, :3]):
        if not free[index]:
            continue
        neighbors = tree.query_ball_point(node, expanding_radius)
        if len(neighbors) < 8:
            neighbors = tree.query_ball_point(node, 2.0 * expanding_radius)
        for neighbor in neighbors:
            if free[neighbor]:
                groups.union(index, int(neighbor))
    free_ids = np.flatnonzero(free)
    roots = np.asarray([groups.find(int(index)) for index in free_ids])
    _, component_sizes = np.unique(roots, return_counts=True)
    largest = int(component_sizes.max()) if len(component_sizes) else 0
    _, start_index = cKDTree(ground[:, :2]).query(start_xyz[:2])
    start_index = int(start_index)
    start_root = groups.find(start_index)
    start_size = int(np.count_nonzero(roots == start_root)) if free[start_index] else 0
    start_distance = float(np.linalg.norm(ground[start_index, :2] - start_xyz[:2]))
    return int(len(component_sizes)), largest, start_size, start_distance


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--voxel-size", type=float, default=0.2)
    parser.add_argument("--imposing-radius", type=float, default=1.5)
    parser.add_argument("--minimum-points", type=int, default=11)
    parser.add_argument("--inscribed-radius", type=float, default=0.32)
    parser.add_argument("--expanding-radius", type=float, default=0.5)
    parser.add_argument("--start-x", type=float, default=2.785)
    parser.add_argument("--start-y", type=float, default=-0.223)
    parser.add_argument("--start-z", type=float, default=0.307)
    parser.add_argument(
        "--xy-radii",
        type=float,
        nargs="+",
        default=[0.25, 0.30, 0.32, 0.35, 0.50],
    )
    args = parser.parse_args()

    features = merge_keyframes(args.map_dir, "feature")
    ground_raw = merge_keyframes(args.map_dir, "ground")
    clustered = keep_euclidean_clusters(features, tolerance=0.2, minimum=10)
    map_points = voxel_centroids(clustered, args.voxel_size)
    ground = voxel_centroids(ground_raw, args.voxel_size)
    start = np.asarray([args.start_x, args.start_y, args.start_z])

    print(f"MERGED_FEATURE_POINTS={len(features)}")
    print(f"FILTERED_VOXEL_FEATURE_POINTS={len(map_points)}")
    print(f"MERGED_GROUND_POINTS={len(ground_raw)}")
    print(f"VOXEL_GROUND_POINTS={len(ground)}")
    print("xy_radius classified lethal free components largest start_component start_ground_distance")
    for xy_radius in args.xy_radii:
        distances, counts = static_distances(
            map_points,
            ground,
            args.imposing_radius,
            xy_radius,
            args.minimum_points,
        )
        classified = np.isfinite(distances)
        lethal = distances < args.inscribed_radius
        free = ~lethal
        components, largest, start_size, start_distance = connectivity(
            ground, free, start, args.expanding_radius
        )
        print(
            f"{xy_radius:.2f} {classified.sum()} {lethal.sum()} {free.sum()} "
            f"{components} {largest} {start_size} {start_distance:.3f} "
            f"count_p50={np.percentile(counts, 50):.0f} "
            f"count_p90={np.percentile(counts, 90):.0f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
