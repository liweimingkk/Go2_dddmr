import numpy as np

from dddmr_glass_filter.export_odom_cloud import nearest_pose_index
from dddmr_glass_filter.transforms import (
    PoseBuffer,
    PoseSample,
    RigidTransform,
)


def test_transform_composition_and_inverse():
    base_from_sensor = RigidTransform.from_quaternion(
        [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    )
    odom_from_base = RigidTransform.from_quaternion(
        [0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    )
    odom_from_sensor = base_from_sensor.then(odom_from_base)

    transformed = odom_from_sensor.apply(np.asarray([[0.0, 0.0, 0.0]]))
    np.testing.assert_allclose(transformed, [[1.0, 2.0, 0.0]])
    np.testing.assert_allclose(
        odom_from_sensor.inverse().apply(transformed), [[0.0, 0.0, 0.0]]
    )


def test_pose_buffer_enforces_maximum_time_delta():
    buffer = PoseBuffer(maximum_size=3)
    identity = RigidTransform.identity()
    buffer.append(PoseSample(100, "odom", "base_link", identity))
    buffer.append(PoseSample(200, "odom", "base_link", identity))

    assert buffer.closest(180, 30).stamp_ns == 200
    assert buffer.closest(180, 10) is None


def test_rpy_transform_rotates_sensor_x_to_base_y():
    transform = RigidTransform.from_rpy(
        [0.14543, 0.0, 0.13312], 0.0, 0.0, np.pi / 2.0
    )

    np.testing.assert_allclose(
        transform.apply(np.asarray([[1.0, 0.0, 0.0]])),
        [[0.14543, 1.0, 0.13312]],
        atol=1.0e-8,
    )


def test_nearest_pose_index_checks_both_sides():
    stamps = np.asarray([100, 200, 400], dtype=np.int64)

    assert nearest_pose_index(stamps, 50) == 0
    assert nearest_pose_index(stamps, 180) == 1
    assert nearest_pose_index(stamps, 320) == 2
    assert nearest_pose_index(stamps, 500) == 2
