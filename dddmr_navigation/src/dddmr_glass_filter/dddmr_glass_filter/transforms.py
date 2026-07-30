"""Small, ROS-independent rigid-transform and pose-buffer helpers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


def rpy_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    values = np.asarray([roll, pitch, yaw], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("roll, pitch, and yaw must be finite")
    sr, sp, sy = np.sin(values)
    cr, cp, cy = np.cos(values)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def quaternion_rotation_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite numbers")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm must be non-zero")
    x, y, z, w = quaternion / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class RigidTransform:
    """Transform points from a source frame into a target frame."""

    rotation: np.ndarray
    translation: np.ndarray

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls(rotation=np.eye(3, dtype=np.float64), translation=np.zeros(3))

    @classmethod
    def from_quaternion(
        cls, translation: Sequence[float], quaternion_xyzw: Sequence[float]
    ) -> "RigidTransform":
        offset = np.asarray(translation, dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("translation must contain three finite numbers")
        return cls(
            rotation=quaternion_rotation_matrix(quaternion_xyzw),
            translation=offset,
        )

    @classmethod
    def from_rpy(
        cls,
        translation: Sequence[float],
        roll: float,
        pitch: float,
        yaw: float,
    ) -> "RigidTransform":
        offset = np.asarray(translation, dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("translation must contain three finite numbers")
        return cls(
            rotation=rpy_rotation_matrix(roll, pitch, yaw),
            translation=offset,
        )

    def apply(self, points: np.ndarray) -> np.ndarray:
        query = np.asarray(points, dtype=np.float64)
        if query.ndim == 1:
            if query.shape != (3,):
                raise ValueError("point must have shape (3,)")
            return self.rotation @ query + self.translation
        if query.ndim != 2 or query.shape[1:] != (3,):
            raise ValueError("points must have shape (N, 3)")
        return query @ self.rotation.T + self.translation

    def inverse(self) -> "RigidTransform":
        inverse_rotation = self.rotation.T
        return RigidTransform(
            rotation=inverse_rotation,
            translation=-(inverse_rotation @ self.translation),
        )

    def then(self, target_from_current: "RigidTransform") -> "RigidTransform":
        """Return ``target_from_current * self``."""
        return RigidTransform(
            rotation=target_from_current.rotation @ self.rotation,
            translation=(
                target_from_current.rotation @ self.translation
                + target_from_current.translation
            ),
        )


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    parent_frame: str
    child_frame: str
    parent_from_child: RigidTransform


class PoseBuffer:
    def __init__(self, maximum_size: int = 500) -> None:
        if maximum_size <= 0:
            raise ValueError("maximum_size must be positive")
        self._samples: deque[PoseSample] = deque(maxlen=maximum_size)

    def append(self, sample: PoseSample) -> None:
        self._samples.append(sample)

    def closest(self, stamp_ns: int, maximum_delta_ns: int) -> Optional[PoseSample]:
        if maximum_delta_ns < 0:
            raise ValueError("maximum_delta_ns must be non-negative")
        if not self._samples:
            return None
        sample = min(self._samples, key=lambda candidate: abs(candidate.stamp_ns - stamp_ns))
        if abs(sample.stamp_ns - stamp_ns) > maximum_delta_ns:
            return None
        return sample

    def __len__(self) -> int:
        return len(self._samples)
