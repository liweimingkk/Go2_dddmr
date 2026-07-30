import numpy as np

from dddmr_glass_filter.fit_plane import fit_bounded_plane
from dddmr_glass_filter.pcd import read_pcd_xyz


def test_ransac_fit_ignores_outliers_and_returns_bounded_rectangle():
    rng = np.random.default_rng(7)
    inliers = np.column_stack(
        (
            5.0 + rng.normal(0.0, 0.005, 500),
            rng.uniform(-2.0, 2.0, 500),
            rng.uniform(-0.5, 3.0, 500),
        )
    )
    outliers = rng.uniform(-5.0, 5.0, (50, 3))

    fit = fit_bounded_plane(
        np.concatenate((inliers, outliers)),
        distance_threshold=0.03,
        iterations=300,
        minimum_inlier_ratio=0.8,
        padding=0.0,
        random_seed=3,
    )

    assert np.count_nonzero(fit.inlier_mask) >= 495
    assert abs(abs(fit.normal[0]) - 1.0) < 0.01
    assert np.max(np.abs(fit.vertices[:, 0] - 5.0)) < 0.02
    assert fit.residual_p95 < 0.012


def test_reads_ascii_pcd_xyz(tmp_path):
    path = tmp_path / "selection.pcd"
    path.write_text(
        """# .PCD v0.7
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 2
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 2
DATA ascii
1 2 3 9
4 5 6 10
""",
        encoding="ascii",
    )

    np.testing.assert_allclose(read_pcd_xyz(path), [[1, 2, 3], [4, 5, 6]])
