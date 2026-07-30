import numpy as np
import pytest

from dddmr_glass_filter.geometry import (
    GlassPlane,
    filter_points_behind_planes,
    sample_plane_surface,
)


def rectangle_at_x(x_value: float, plane_id: str = "glass") -> GlassPlane:
    return GlassPlane.from_vertices(
        plane_id,
        [
            [x_value, -2.0, -1.0],
            [x_value, 2.0, -1.0],
            [x_value, 2.0, 3.0],
            [x_value, -2.0, 3.0],
        ],
    )


def test_marks_only_returns_safely_behind_bounded_glass():
    plane = rectangle_at_x(5.0)
    points = np.asarray(
        [
            [10.0, 0.0, 1.0],
            [3.0, 0.0, 1.0],
            [5.10, 0.0, 1.0],
            [10.0, 6.0, 1.0],
            [np.nan, 0.0, 0.0],
        ]
    )
    result = filter_points_behind_planes(
        [0.0, 0.0, 0.0],
        points,
        [plane],
        minimum_behind_distance=0.15,
    )

    assert result.mask.tolist() == [True, False, False, False, False]
    np.testing.assert_allclose(result.intersections[0], [5.0, 0.0, 0.5])
    assert result.plane_indices.tolist() == [0, -1, -1, -1, -1]


def test_works_from_either_side_of_the_plane():
    plane = rectangle_at_x(5.0)
    result = filter_points_behind_planes(
        [10.0, 0.0, 1.0],
        np.asarray([[0.0, 0.0, 1.0]]),
        [plane],
        minimum_behind_distance=0.15,
    )

    assert result.mask.tolist() == [True]
    np.testing.assert_allclose(result.intersections[0], [5.0, 0.0, 1.0])


def test_selects_nearest_crossed_plane():
    near = rectangle_at_x(3.0, "near")
    far = rectangle_at_x(5.0, "far")
    result = filter_points_behind_planes(
        [0.0, 0.0, 1.0],
        np.asarray([[10.0, 0.0, 1.0]]),
        [far, near],
        minimum_behind_distance=0.15,
    )

    assert result.mask.tolist() == [True]
    assert result.plane_indices.tolist() == [1]
    np.testing.assert_allclose(result.intersections[0], [3.0, 0.0, 1.0])


def test_rejects_non_convex_or_non_coplanar_vertices():
    with pytest.raises(ValueError, match="convex"):
        GlassPlane.from_vertices(
            "concave",
            [[0, 0, 0], [0, 2, 0], [0, 1, 1], [0, 2, 2], [0, 0, 2]],
        )
    with pytest.raises(ValueError, match="not coplanar"):
        GlassPlane.from_vertices(
            "warped", [[0, 0, 0], [0, 1, 0], [0, 1, 1], [0.2, 0, 1]]
        )


def test_surface_sampling_stays_inside_polygon():
    plane = rectangle_at_x(5.0)
    samples = sample_plane_surface(plane, spacing=0.25)

    assert len(samples) > 100
    assert np.all(plane.contains(samples))
    np.testing.assert_allclose(samples[:, 0], 5.0)
