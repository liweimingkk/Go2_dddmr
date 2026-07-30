"""Zero-copy PointCloud2 field readers and safe XYZ replacement."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Union

import numpy as np


# sensor_msgs/msg/PointField numeric constants.  Keeping these local makes the
# parser usable by offline tests without importing a running ROS environment.
INT8 = 1
UINT8 = 2
INT16 = 3
UINT16 = 4
INT32 = 5
UINT32 = 6
FLOAT32 = 7
FLOAT64 = 8


_DTYPES = {
    INT8: "i1",
    UINT8: "u1",
    INT16: "i2",
    UINT16: "u2",
    INT32: "i4",
    UINT32: "u4",
    FLOAT32: "f4",
    FLOAT64: "f8",
}


@dataclass(frozen=True)
class FieldSpec:
    name: str
    offset: int
    datatype: int
    count: int


class PointCloudLayout:
    def __init__(
        self,
        *,
        width: int,
        height: int,
        point_step: int,
        row_step: int,
        is_bigendian: bool,
        fields: Iterable[Any],
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.point_step = int(point_step)
        self.row_step = int(row_step)
        self.is_bigendian = bool(is_bigendian)
        self.fields = {
            str(field.name): FieldSpec(
                name=str(field.name),
                offset=int(field.offset),
                datatype=int(field.datatype),
                count=int(field.count),
            )
            for field in fields
        }
        if self.width < 0 or self.height < 0 or self.point_step <= 0:
            raise ValueError("invalid PointCloud2 dimensions")
        if self.row_step != self.width * self.point_step:
            raise ValueError("PointCloud2 rows with padding are not supported")

    @classmethod
    def from_message(cls, message: Any) -> "PointCloudLayout":
        return cls(
            width=message.width,
            height=message.height,
            point_step=message.point_step,
            row_step=message.row_step,
            is_bigendian=message.is_bigendian,
            fields=message.fields,
        )

    @property
    def point_count(self) -> int:
        return self.width * self.height

    def _dtype(self, spec: FieldSpec) -> np.dtype:
        base = _DTYPES.get(spec.datatype)
        if base is None:
            raise ValueError(f"unsupported datatype {spec.datatype} for field {spec.name!r}")
        prefix = ">" if self.is_bigendian else "<"
        return np.dtype(prefix + base)

    def read_field(
        self, data: Union[bytes, bytearray, memoryview], name: str
    ) -> np.ndarray:
        spec = self.fields.get(name)
        if spec is None:
            raise ValueError(f"PointCloud2 is missing field {name!r}")
        if spec.count != 1:
            raise ValueError(f"PointCloud2 field {name!r} must have count 1")
        dtype = self._dtype(spec)
        if spec.offset < 0 or spec.offset + dtype.itemsize > self.point_step:
            raise ValueError(f"PointCloud2 field {name!r} exceeds point_step")
        required_size = self.point_count * self.point_step
        if len(data) < required_size:
            raise ValueError(
                f"PointCloud2 data has {len(data)} bytes, expected at least {required_size}"
            )
        return np.ndarray(
            shape=(self.point_count,),
            dtype=dtype,
            buffer=data,
            offset=spec.offset,
            strides=(self.point_step,),
        )

    def read_xyz(self, data: Union[bytes, bytearray, memoryview]) -> np.ndarray:
        columns = []
        for name in ("x", "y", "z"):
            spec = self.fields.get(name)
            if spec is None or spec.datatype != FLOAT32 or spec.count != 1:
                raise ValueError(f"PointCloud2 {name!r} must be a scalar FLOAT32 field")
            columns.append(self.read_field(data, name).astype(np.float64, copy=False))
        return np.column_stack(columns)

    def replace_xyz(
        self,
        data: Union[bytes, bytearray, memoryview],
        mask: np.ndarray,
        replacement_xyz: np.ndarray,
    ) -> bytes:
        selected = np.asarray(mask, dtype=bool)
        replacement = np.asarray(replacement_xyz, dtype=np.float64)
        if selected.shape != (self.point_count,):
            raise ValueError("replacement mask length does not match PointCloud2")
        if replacement.shape != (self.point_count, 3):
            raise ValueError("replacement_xyz must have shape (point_count, 3)")
        output = bytearray(data)
        for column, name in enumerate(("x", "y", "z")):
            values = self.read_field(output, name)
            values[selected] = replacement[selected, column].astype(values.dtype, copy=False)
        return bytes(output)


def message_stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
