"""Load and validate known-glass-plane YAML files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Union

import yaml

from .geometry import GlassPlane


@dataclass(frozen=True)
class PlaneConfiguration:
    frame_id: str
    planes: tuple[GlassPlane, ...]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def load_plane_configuration(path: Union[str, Path]) -> PlaneConfiguration:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise ValueError(f"glass plane config does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        document = _mapping(yaml.safe_load(stream), "glass plane config")

    version = document.get("version")
    if version != 1:
        raise ValueError(f"unsupported glass plane config version: {version!r}")
    frame_id = document.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("glass plane config frame_id must be a non-empty string")

    plane_documents = document.get("planes")
    if not isinstance(plane_documents, Sequence) or isinstance(
        plane_documents, (str, bytes)
    ):
        raise ValueError("glass plane config planes must be a sequence")
    if not plane_documents:
        raise ValueError("glass plane config must define at least one plane")

    planes = []
    seen_ids = set()
    for index, raw_plane in enumerate(plane_documents):
        plane_document = _mapping(raw_plane, f"planes[{index}]")
        plane_id = plane_document.get("id")
        if not isinstance(plane_id, str) or not plane_id.strip():
            raise ValueError(f"planes[{index}].id must be a non-empty string")
        if plane_id in seen_ids:
            raise ValueError(f"duplicate glass plane id: {plane_id!r}")
        seen_ids.add(plane_id)

        vertices = plane_document.get("vertices")
        minimum = plane_document.get("minimum_behind_distance")
        coplanar_tolerance = float(plane_document.get("coplanar_tolerance", 0.02))
        planes.append(
            GlassPlane.from_vertices(
                plane_id,
                vertices if vertices is not None else [],
                minimum_behind_distance=minimum,
                coplanar_tolerance=coplanar_tolerance,
            )
        )

    return PlaneConfiguration(frame_id=frame_id.strip(), planes=tuple(planes))


def plane_configuration_document(
    frame_id: str,
    plane_id: str,
    vertices: Sequence[Sequence[float]],
    *,
    minimum_behind_distance: Optional[float] = None,
) -> dict[str, Any]:
    """Build a YAML-serializable version-1 configuration document."""
    plane: dict[str, Any] = {
        "id": plane_id,
        "vertices": [[float(component) for component in vertex] for vertex in vertices],
    }
    if minimum_behind_distance is not None:
        plane["minimum_behind_distance"] = float(minimum_behind_distance)
    return {"version": 1, "frame_id": frame_id, "planes": [plane]}
