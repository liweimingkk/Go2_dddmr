"""Geometry primitives for filtering returns behind known glass polygons."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np


_EPSILON = 1.0e-9


def _as_vector3(value: Sequence[float], label: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must contain three finite numbers")
    return vector


def _normalize(vector: np.ndarray, label: str) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if not np.isfinite(length) or length <= _EPSILON:
        raise ValueError(f"{label} must have non-zero length")
    return vector / length


@dataclass(frozen=True)
class GlassPlane:
    """A bounded, convex glass polygon in one fixed coordinate frame."""

    plane_id: str
    vertices: np.ndarray
    origin: np.ndarray
    normal: np.ndarray
    axis_u: np.ndarray
    axis_v: np.ndarray
    polygon_uv: np.ndarray
    orientation: float
    minimum_behind_distance: Optional[float] = None

    @classmethod
    def from_vertices(
        cls,
        plane_id: str,
        vertices: Iterable[Sequence[float]],
        *,
        minimum_behind_distance: Optional[float] = None,
        coplanar_tolerance: float = 0.02,
    ) -> "GlassPlane":
        points = np.asarray(list(vertices), dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 3:
            raise ValueError(f"plane {plane_id!r} needs at least three 3-D vertices")
        if not np.all(np.isfinite(points)):
            raise ValueError(f"plane {plane_id!r} contains a non-finite vertex")
        if not plane_id:
            raise ValueError("plane id must not be empty")
        if coplanar_tolerance < 0.0 or not np.isfinite(coplanar_tolerance):
            raise ValueError("coplanar_tolerance must be finite and non-negative")
        if minimum_behind_distance is not None:
            minimum_behind_distance = float(minimum_behind_distance)
            if minimum_behind_distance < 0.0 or not np.isfinite(minimum_behind_distance):
                raise ValueError(
                    f"plane {plane_id!r} minimum_behind_distance must be finite and non-negative"
                )

        origin = points[0]
        axis_u = None
        normal = None
        for first_index in range(1, len(points)):
            first_edge = points[first_index] - origin
            if np.linalg.norm(first_edge) <= _EPSILON:
                continue
            for second_index in range(first_index + 1, len(points)):
                second_edge = points[second_index] - origin
                candidate = np.cross(first_edge, second_edge)
                if np.linalg.norm(candidate) > _EPSILON:
                    axis_u = _normalize(first_edge, "plane edge")
                    normal = _normalize(candidate, "plane normal")
                    break
            if normal is not None:
                break
        if normal is None or axis_u is None:
            raise ValueError(f"plane {plane_id!r} vertices are collinear")

        signed_distances = (points - origin) @ normal
        maximum_residual = float(np.max(np.abs(signed_distances)))
        if maximum_residual > coplanar_tolerance:
            raise ValueError(
                f"plane {plane_id!r} vertices are not coplanar; "
                f"maximum residual {maximum_residual:.6f} m"
            )

        axis_v = _normalize(np.cross(normal, axis_u), "plane secondary axis")
        relative = points - origin
        polygon_uv = np.column_stack((relative @ axis_u, relative @ axis_v))
        edges = np.roll(polygon_uv, -1, axis=0) - polygon_uv
        following_edges = np.roll(edges, -1, axis=0)
        cross_values = (
            edges[:, 0] * following_edges[:, 1]
            - edges[:, 1] * following_edges[:, 0]
        )
        meaningful = cross_values[np.abs(cross_values) > _EPSILON]
        if len(meaningful) == 0:
            raise ValueError(f"plane {plane_id!r} polygon has zero area")
        orientation = float(np.sign(meaningful[0]))
        if np.any(meaningful * orientation <= 0.0):
            raise ValueError(
                f"plane {plane_id!r} vertices must form an ordered convex polygon"
            )

        return cls(
            plane_id=plane_id,
            vertices=points,
            origin=origin.copy(),
            normal=normal,
            axis_u=axis_u,
            axis_v=axis_v,
            polygon_uv=polygon_uv,
            orientation=orientation,
            minimum_behind_distance=minimum_behind_distance,
        )

    def contains(self, points: np.ndarray, boundary_tolerance: float = 0.0) -> np.ndarray:
        """Return a mask for points whose projection lies inside the polygon."""
        query = np.asarray(points, dtype=np.float64)
        if query.ndim != 2 or query.shape[1:] != (3,):
            raise ValueError("query points must have shape (N, 3)")
        if boundary_tolerance < 0.0 or not np.isfinite(boundary_tolerance):
            raise ValueError("boundary_tolerance must be finite and non-negative")

        relative = query - self.origin
        query_uv = np.column_stack((relative @ self.axis_u, relative @ self.axis_v))
        starts = self.polygon_uv
        edges = np.roll(starts, -1, axis=0) - starts
        inside = np.ones(len(query), dtype=bool)
        for start, edge in zip(starts, edges):
            delta = query_uv - start
            cross_value = edge[0] * delta[:, 1] - edge[1] * delta[:, 0]
            allowed_error = boundary_tolerance * float(np.linalg.norm(edge))
            inside &= self.orientation * cross_value >= -allowed_error
        return inside

    def intersect_segments(
        self,
        sensor_origin: Sequence[float],
        endpoints: np.ndarray,
        *,
        minimum_behind_distance: float,
        boundary_tolerance: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Classify endpoints whose sensor ray meets this glass polygon first.

        Returns ``(mask, intersections, ray_fraction)``.  Only a finite endpoint
        at least ``minimum_behind_distance`` beyond the bounded plane is marked.
        """
        sensor = _as_vector3(sensor_origin, "sensor_origin")
        points = np.asarray(endpoints, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError("endpoints must have shape (N, 3)")
        if minimum_behind_distance < 0.0 or not np.isfinite(minimum_behind_distance):
            raise ValueError("minimum_behind_distance must be finite and non-negative")

        directions = points - sensor
        finite = np.all(np.isfinite(points), axis=1)
        denominator = directions @ self.normal
        numerator = float((self.origin - sensor) @ self.normal)
        usable = finite & (np.abs(denominator) > _EPSILON)
        fraction = np.full(len(points), np.nan, dtype=np.float64)
        fraction[usable] = numerator / denominator[usable]

        candidates = usable & (fraction > _EPSILON) & (fraction < 1.0 - _EPSILON)
        intersections = np.full_like(points, np.nan, dtype=np.float64)
        intersections[candidates] = (
            sensor + fraction[candidates, np.newaxis] * directions[candidates]
        )
        inside = np.zeros(len(points), dtype=bool)
        if np.any(candidates):
            inside[candidates] = self.contains(
                intersections[candidates], boundary_tolerance=boundary_tolerance
            )
        remaining_distance = np.full(len(points), np.nan, dtype=np.float64)
        remaining_distance[candidates] = np.linalg.norm(
            points[candidates] - intersections[candidates], axis=1
        )
        mask = candidates & inside & (remaining_distance >= minimum_behind_distance)
        return mask, intersections, fraction


@dataclass(frozen=True)
class FilterResult:
    mask: np.ndarray
    intersections: np.ndarray
    plane_indices: np.ndarray
    ray_fractions: np.ndarray


def filter_points_behind_planes(
    sensor_origin: Sequence[float],
    endpoints: np.ndarray,
    planes: Sequence[GlassPlane],
    *,
    minimum_behind_distance: float,
    boundary_tolerance: float = 0.0,
) -> FilterResult:
    """Find the first configured glass polygon crossed by every sensor ray."""
    points = np.asarray(endpoints, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("endpoints must have shape (N, 3)")

    count = len(points)
    selected_mask = np.zeros(count, dtype=bool)
    selected_fraction = np.full(count, np.inf, dtype=np.float64)
    selected_intersection = np.full((count, 3), np.nan, dtype=np.float64)
    selected_plane = np.full(count, -1, dtype=np.int32)

    for plane_index, plane in enumerate(planes):
        plane_minimum = (
            minimum_behind_distance
            if plane.minimum_behind_distance is None
            else plane.minimum_behind_distance
        )
        mask, intersections, fractions = plane.intersect_segments(
            sensor_origin,
            points,
            minimum_behind_distance=plane_minimum,
            boundary_tolerance=boundary_tolerance,
        )
        nearer = mask & (fractions < selected_fraction)
        selected_mask[nearer] = True
        selected_fraction[nearer] = fractions[nearer]
        selected_intersection[nearer] = intersections[nearer]
        selected_plane[nearer] = plane_index

    selected_fraction[~selected_mask] = np.nan
    return FilterResult(
        mask=selected_mask,
        intersections=selected_intersection,
        plane_indices=selected_plane,
        ray_fractions=selected_fraction,
    )


def sample_plane_surface(plane: GlassPlane, spacing: float) -> np.ndarray:
    """Sample a bounded plane for visualization and no-entry-zone authoring."""
    if spacing <= 0.0 or not np.isfinite(spacing):
        raise ValueError("spacing must be finite and positive")
    lower = np.min(plane.polygon_uv, axis=0)
    upper = np.max(plane.polygon_uv, axis=0)
    u_values = np.arange(lower[0], upper[0] + spacing * 0.5, spacing)
    v_values = np.arange(lower[1], upper[1] + spacing * 0.5, spacing)
    grid_u, grid_v = np.meshgrid(u_values, v_values, indexing="xy")
    flat_uv = np.column_stack((grid_u.ravel(), grid_v.ravel()))
    points = (
        plane.origin
        + flat_uv[:, 0, np.newaxis] * plane.axis_u
        + flat_uv[:, 1, np.newaxis] * plane.axis_v
    )
    return points[plane.contains(points, boundary_tolerance=spacing * 0.25)]
