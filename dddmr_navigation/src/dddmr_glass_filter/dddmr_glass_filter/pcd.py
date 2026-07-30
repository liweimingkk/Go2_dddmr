"""Minimal ASCII/binary PCD reader for offline plane fitting."""

from __future__ import annotations

from io import BytesIO
from pathlib import Path
from typing import Union

import numpy as np


_PCD_DTYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
}


def read_pcd_xyz(path: Union[str, Path]) -> np.ndarray:
    pcd_path = Path(path).expanduser()
    if not pcd_path.is_file():
        raise ValueError(f"PCD file does not exist: {pcd_path}")

    header: dict[str, list[str]] = {}
    with pcd_path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD header has no DATA line: {pcd_path}")
            stripped = line.decode("ascii", errors="strict").strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                payload = stream.read()
                break

    fields = header.get("FIELDS") or header.get("FIELD")
    sizes = header.get("SIZE")
    types = header.get("TYPE")
    counts = header.get("COUNT", ["1"] * len(fields or []))
    if not fields or not sizes or not types:
        raise ValueError(f"PCD header is missing FIELDS/SIZE/TYPE: {pcd_path}")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"PCD field metadata lengths do not match: {pcd_path}")
    for coordinate in ("x", "y", "z"):
        if coordinate not in fields:
            raise ValueError(f"PCD file is missing {coordinate!r}: {pcd_path}")

    data_mode = header["DATA"][0].lower()
    if data_mode == "ascii":
        matrix = np.loadtxt(BytesIO(payload), dtype=np.float64, ndmin=2)
        expected_columns = sum(int(value) for value in counts)
        if matrix.shape[1] != expected_columns:
            raise ValueError(
                f"PCD row has {matrix.shape[1]} columns, expected {expected_columns}"
            )
        field_offsets = {}
        column = 0
        for name, count in zip(fields, counts):
            field_offsets[name] = column
            column += int(count)
        xyz = np.column_stack(
            [matrix[:, field_offsets[coordinate]] for coordinate in ("x", "y", "z")]
        )
    elif data_mode == "binary":
        dtype_fields = []
        for name, size, type_code, count in zip(fields, sizes, types, counts):
            base = _PCD_DTYPES.get((type_code.upper(), int(size)))
            if base is None:
                raise ValueError(
                    f"unsupported PCD field type {type_code}{size} for {name!r}"
                )
            item_count = int(count)
            dtype_fields.append(
                (name, np.dtype(base), (item_count,)) if item_count > 1 else (name, np.dtype(base))
            )
        records = np.frombuffer(payload, dtype=np.dtype(dtype_fields))
        xyz = np.column_stack(
            [records[coordinate].astype(np.float64) for coordinate in ("x", "y", "z")]
        )
    elif data_mode == "binary_compressed":
        raise ValueError("binary_compressed PCD is not supported; export ASCII or binary PCD")
    else:
        raise ValueError(f"unsupported PCD DATA mode: {data_mode!r}")

    finite = np.all(np.isfinite(xyz), axis=1)
    return xyz[finite]
