"""Fit a bounded glass rectangle from a carefully selected PCD region."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .configuration import plane_configuration_document
from .geometry import GlassPlane
from .pcd import read_pcd_xyz


@dataclass(frozen=True)
class PlaneFit:
    vertices: np.ndarray
    normal: np.ndarray
    inlier_mask: np.ndarray
    residual_median: float
    residual_p95: float
    residual_maximum: float


def fit_bounded_plane(
    points: np.ndarray,
    *,
    distance_threshold: float = 0.05,
    iterations: int = 1000,
    minimum_inlier_ratio: float = 0.60,
    padding: float = 0.10,
    random_seed: int = 0,
) -> PlaneFit:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1:] != (3,) or len(cloud) < 3:
        raise ValueError("plane fitting needs at least three 3-D points")
    cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
    if len(cloud) < 3:
        raise ValueError("plane fitting needs at least three finite points")
    if distance_threshold <= 0.0 or not np.isfinite(distance_threshold):
        raise ValueError("distance_threshold must be finite and positive")
    if iterations <= 0:
        raise ValueError("iterations must be positive")
    if not 0.0 < minimum_inlier_ratio <= 1.0:
        raise ValueError("minimum_inlier_ratio must be in (0, 1]")
    if padding < 0.0 or not np.isfinite(padding):
        raise ValueError("padding must be finite and non-negative")

    rng = np.random.default_rng(random_seed)
    best_mask = np.zeros(len(cloud), dtype=bool)
    best_score = (-1, -np.inf)
    for _ in range(iterations):
        sample = cloud[rng.choice(len(cloud), size=3, replace=False)]
        candidate = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = float(np.linalg.norm(candidate))
        if length <= 1.0e-9:
            continue
        normal = candidate / length
        residuals = np.abs((cloud - sample[0]) @ normal)
        mask = residuals <= distance_threshold
        count = int(np.count_nonzero(mask))
        score = (count, -float(np.median(residuals[mask])) if count else -np.inf)
        if score > best_score:
            best_score = score
            best_mask = mask

    inlier_count = int(np.count_nonzero(best_mask))
    inlier_ratio = inlier_count / len(cloud)
    if inlier_ratio < minimum_inlier_ratio:
        raise ValueError(
            "no dominant plane reached the minimum inlier ratio: "
            f"{inlier_count}/{len(cloud)} ({inlier_ratio:.3f})"
        )

    inliers = cloud[best_mask]
    centroid = np.mean(inliers, axis=0)
    _, _, basis_rows = np.linalg.svd(inliers - centroid, full_matrices=False)
    axis_u = basis_rows[0]
    normal = basis_rows[-1]
    largest_component = int(np.argmax(np.abs(normal)))
    if normal[largest_component] < 0.0:
        normal = -normal
    axis_u = axis_u - normal * float(axis_u @ normal)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    axis_v /= np.linalg.norm(axis_v)

    coordinates_u = (inliers - centroid) @ axis_u
    coordinates_v = (inliers - centroid) @ axis_v
    lower_u, upper_u = (
        float(np.min(coordinates_u)) - padding,
        float(np.max(coordinates_u)) + padding,
    )
    lower_v, upper_v = (
        float(np.min(coordinates_v)) - padding,
        float(np.max(coordinates_v)) + padding,
    )
    corners_uv = np.asarray(
        [
            [lower_u, lower_v],
            [upper_u, lower_v],
            [upper_u, upper_v],
            [lower_u, upper_v],
        ]
    )
    vertices = (
        centroid
        + corners_uv[:, 0, np.newaxis] * axis_u
        + corners_uv[:, 1, np.newaxis] * axis_v
    )
    # Re-run the production polygon validator before writing configuration.
    GlassPlane.from_vertices("fitted_plane", vertices)

    residuals = np.abs((cloud - centroid) @ normal)
    refined_mask = residuals <= distance_threshold
    inlier_residuals = residuals[refined_mask]
    return PlaneFit(
        vertices=vertices,
        normal=normal,
        inlier_mask=refined_mask,
        residual_median=float(np.median(inlier_residuals)),
        residual_p95=float(np.quantile(inlier_residuals, 0.95)),
        residual_maximum=float(np.max(inlier_residuals)),
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit one bounded glass plane from an RViz-selected PCD. Select only "
            "the glass/frame surface, not the ghost wall behind it."
        )
    )
    parser.add_argument("--selection-pcd", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--plane-id", default="glass_facade")
    parser.add_argument("--distance-threshold", type=float, default=0.05)
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--minimum-inlier-ratio", type=float, default=0.60)
    parser.add_argument("--padding", type=float, default=0.10)
    parser.add_argument("--minimum-behind-distance", type=float, default=0.15)
    parser.add_argument("--random-seed", type=int, default=0)
    return parser


def main(argv=None) -> None:
    args = build_argument_parser().parse_args(argv)
    output = Path(args.output).expanduser()
    if output.exists():
        raise SystemExit(f"refusing to overwrite existing output: {output}")
    points = read_pcd_xyz(args.selection_pcd)
    fit = fit_bounded_plane(
        points,
        distance_threshold=args.distance_threshold,
        iterations=args.iterations,
        minimum_inlier_ratio=args.minimum_inlier_ratio,
        padding=args.padding,
        random_seed=args.random_seed,
    )
    document = plane_configuration_document(
        args.frame_id,
        args.plane_id,
        fit.vertices,
        minimum_behind_distance=args.minimum_behind_distance,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, sort_keys=False)

    inlier_count = int(np.count_nonzero(fit.inlier_mask))
    print(f"OUTPUT={output}")
    print(f"INPUT_POINTS={len(points)}")
    print(f"INLIER_POINTS={inlier_count}")
    print(f"INLIER_RATIO={inlier_count / len(points):.6f}")
    print("NORMAL=" + " ".join(f"{value:.9f}" for value in fit.normal))
    print(f"RESIDUAL_MEDIAN_M={fit.residual_median:.6f}")
    print(f"RESIDUAL_P95_M={fit.residual_p95:.6f}")
    print(f"RESIDUAL_MAX_M={fit.residual_maximum:.6f}")
    for index, vertex in enumerate(fit.vertices):
        print(f"VERTEX_{index}=" + " ".join(f"{value:.9f}" for value in vertex))


if __name__ == "__main__":
    main()
