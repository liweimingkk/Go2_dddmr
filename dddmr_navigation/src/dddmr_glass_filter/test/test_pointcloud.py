import struct
from dataclasses import dataclass

import numpy as np
import pytest

from dddmr_glass_filter.analyze_bag import analyze_pointcloud_message
from dddmr_glass_filter.pointcloud import FLOAT32, FLOAT64, UINT16, PointCloudLayout


@dataclass
class FakeField:
    name: str
    offset: int
    datatype: int
    count: int = 1


class FakeCloud:
    pass


def make_cloud(points):
    message = FakeCloud()
    message.height = 1
    message.width = len(points)
    message.fields = [
        FakeField(name="x", offset=0, datatype=FLOAT32),
        FakeField(name="y", offset=4, datatype=FLOAT32),
        FakeField(name="z", offset=8, datatype=FLOAT32),
        FakeField(name="intensity", offset=12, datatype=FLOAT32),
        FakeField(name="ring", offset=16, datatype=UINT16),
        FakeField(name="timestamp", offset=18, datatype=FLOAT64),
    ]
    message.is_bigendian = False
    message.point_step = 26
    message.row_step = message.width * message.point_step
    message.data = b"".join(struct.pack("<ffffHd", *point) for point in points)
    message.is_dense = True
    return message


def test_reads_and_replaces_strided_xyz_without_touching_other_fields():
    cloud = make_cloud(
        [(1.0, 2.0, 3.0, 11.0, 4, 1.25), (4.0, 5.0, 6.0, 22.0, 5, 1.5)]
    )
    layout = PointCloudLayout.from_message(cloud)
    xyz = layout.read_xyz(cloud.data)
    replacement = xyz.copy()
    replacement[1] = [7.0, 8.0, 9.0]

    output = layout.replace_xyz(cloud.data, np.asarray([False, True]), replacement)

    np.testing.assert_allclose(layout.read_xyz(output), [[1, 2, 3], [7, 8, 9]])
    np.testing.assert_allclose(layout.read_field(output, "intensity"), [11, 22])
    assert layout.read_field(output, "ring").tolist() == [4, 5]
    np.testing.assert_allclose(layout.read_field(output, "timestamp"), [1.25, 1.5])


def test_rejects_row_padding():
    cloud = make_cloud([(1.0, 2.0, 3.0, 11.0, 4, 1.25)])
    cloud.row_step += 1
    with pytest.raises(ValueError, match="padding"):
        PointCloudLayout.from_message(cloud)


def test_analyzer_detects_same_direction_separated_returns():
    cloud = make_cloud(
        [
            (2.0, 0.0, 0.0, 200.0, 0, 1.0),
            (5.0, 0.0, 0.0, 30.0, 0, 1.0),
            (0.0, 3.0, 0.0, 20.0, 0, 1.01),
            (4.0, 0.0, 0.0, 10.0, 1, 1.0),
        ]
    )

    analysis = analyze_pointcloud_message(
        cloud, horizontal_bins=4, pair_separation_threshold=0.1
    )

    assert analysis.ring_count == 2
    assert analysis.duplicate_direction_count == 1
    assert analysis.separated_return_count == 1
    np.testing.assert_allclose(analysis.separations, [3.0])
    assert analysis.range_image.shape == (2, 4)
    assert analysis.range_image[0, 0] == pytest.approx(2.0)
    assert analysis.intensity_image[0, 0] == pytest.approx(200.0)
