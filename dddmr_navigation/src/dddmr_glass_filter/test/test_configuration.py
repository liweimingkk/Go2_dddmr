from pathlib import Path

import pytest
import yaml

from dddmr_glass_filter.configuration import load_plane_configuration


def write_config(path: Path, document):
    path.write_text(yaml.safe_dump(document), encoding="utf-8")


def test_loads_versioned_plane_configuration(tmp_path):
    path = tmp_path / "planes.yaml"
    write_config(
        path,
        {
            "version": 1,
            "frame_id": "odom",
            "planes": [
                {
                    "id": "facade",
                    "minimum_behind_distance": 0.2,
                    "vertices": [
                        [5, -1, 0],
                        [5, 1, 0],
                        [5, 1, 2],
                        [5, -1, 2],
                    ],
                }
            ],
        },
    )

    configuration = load_plane_configuration(path)

    assert configuration.frame_id == "odom"
    assert len(configuration.planes) == 1
    assert configuration.planes[0].plane_id == "facade"
    assert configuration.planes[0].minimum_behind_distance == pytest.approx(0.2)


def test_rejects_duplicate_plane_ids(tmp_path):
    path = tmp_path / "planes.yaml"
    plane = {
        "id": "same",
        "vertices": [[5, -1, 0], [5, 1, 0], [5, 1, 2], [5, -1, 2]],
    }
    write_config(
        path,
        {"version": 1, "frame_id": "odom", "planes": [plane, plane]},
    )

    with pytest.raises(ValueError, match="duplicate"):
        load_plane_configuration(path)
