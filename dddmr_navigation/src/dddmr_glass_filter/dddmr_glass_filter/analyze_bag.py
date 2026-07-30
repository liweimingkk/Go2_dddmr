"""Read-only XT16 bag diagnostics for intensity and multi-return evidence."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Optional

import numpy as np

from .pointcloud import PointCloudLayout, message_stamp_ns


@dataclass(frozen=True)
class FrameAnalysis:
    point_count: int
    finite_point_count: int
    ring_count: int
    duplicate_direction_count: int
    separated_return_count: int
    separations: np.ndarray
    intensities: np.ndarray
    timestamp_span_sec: Optional[float]
    range_image: np.ndarray
    intensity_image: np.ndarray
    separation_image: np.ndarray


def analyze_pointcloud_message(
    message: Any,
    *,
    horizontal_bins: int = 2000,
    pair_separation_threshold: float = 0.10,
) -> FrameAnalysis:
    if horizontal_bins <= 0:
        raise ValueError("horizontal_bins must be positive")
    if pair_separation_threshold < 0.0 or not np.isfinite(
        pair_separation_threshold
    ):
        raise ValueError("pair_separation_threshold must be finite and non-negative")

    layout = PointCloudLayout.from_message(message)
    xyz = layout.read_xyz(message.data)
    ranges = np.linalg.norm(xyz, axis=1)
    finite = np.all(np.isfinite(xyz), axis=1) & (ranges > 0.05)
    ring = layout.read_field(message.data, "ring").astype(np.int64, copy=False)
    valid_ring_values = np.unique(ring[finite])
    ring_to_row = {int(value): index for index, value in enumerate(valid_ring_values)}
    rows = np.asarray([ring_to_row.get(int(value), -1) for value in ring], dtype=np.int64)

    azimuth = np.mod(np.arctan2(xyz[:, 1], xyz[:, 0]), 2.0 * np.pi)
    columns = np.mod(
        np.rint(azimuth / (2.0 * np.pi) * horizontal_bins).astype(np.int64),
        horizontal_bins,
    )
    valid = finite & (rows >= 0)
    keys = rows[valid] * horizontal_bins + columns[valid]
    valid_ranges = ranges[valid]

    image_size = max(len(valid_ring_values), 1) * horizontal_bins
    flat_range = np.full(image_size, np.inf, dtype=np.float64)
    np.minimum.at(flat_range, keys, valid_ranges)

    if "intensity" in layout.fields:
        intensity = layout.read_field(message.data, "intensity").astype(
            np.float64, copy=False
        )
        finite_intensity = intensity[valid & np.isfinite(intensity)]
        flat_intensity = np.full(image_size, np.nan, dtype=np.float64)
        valid_intensity_mask = valid & np.isfinite(intensity)
        np.fmax.at(
            flat_intensity,
            rows[valid_intensity_mask] * horizontal_bins
            + columns[valid_intensity_mask],
            intensity[valid_intensity_mask],
        )
    else:
        finite_intensity = np.empty(0, dtype=np.float64)
        flat_intensity = np.full(image_size, np.nan, dtype=np.float64)

    flat_separation = np.full(image_size, np.nan, dtype=np.float64)
    separations = np.empty(0, dtype=np.float64)
    duplicate_count = 0
    separated_count = 0
    if len(keys):
        order = np.argsort(keys, kind="stable")
        sorted_keys = keys[order]
        sorted_ranges = valid_ranges[order]
        unique_keys, starts, counts = np.unique(
            sorted_keys, return_index=True, return_counts=True
        )
        duplicate_mask = counts >= 2
        duplicate_count = int(np.count_nonzero(duplicate_mask))
        if duplicate_count:
            minimum = np.minimum.reduceat(sorted_ranges, starts)
            maximum = np.maximum.reduceat(sorted_ranges, starts)
            all_separations = maximum - minimum
            separations = all_separations[duplicate_mask]
            duplicate_keys = unique_keys[duplicate_mask]
            flat_separation[duplicate_keys] = separations
            separated_count = int(
                np.count_nonzero(separations >= pair_separation_threshold)
            )

    timestamp_span = None
    if "timestamp" in layout.fields:
        timestamps = layout.read_field(message.data, "timestamp").astype(
            np.float64, copy=False
        )
        timestamps = timestamps[np.isfinite(timestamps) & (timestamps > 0.0)]
        if len(timestamps):
            timestamp_span = float(np.max(timestamps) - np.min(timestamps))

    shape = (max(len(valid_ring_values), 1), horizontal_bins)
    range_image = flat_range.reshape(shape)
    range_image[~np.isfinite(range_image)] = np.nan
    return FrameAnalysis(
        point_count=layout.point_count,
        finite_point_count=int(np.count_nonzero(finite)),
        ring_count=len(valid_ring_values),
        duplicate_direction_count=duplicate_count,
        separated_return_count=separated_count,
        separations=separations,
        intensities=finite_intensity,
        timestamp_span_sec=timestamp_span,
        range_image=range_image,
        intensity_image=flat_intensity.reshape(shape),
        separation_image=flat_separation.reshape(shape),
    )


def _quantiles(values: np.ndarray) -> Optional[dict[str, float]]:
    finite = values[np.isfinite(values)]
    if not len(finite):
        return None
    return {
        label: float(np.quantile(finite, quantile))
        for label, quantile in (
            ("min", 0.0),
            ("p01", 0.01),
            ("p50", 0.50),
            ("p90", 0.90),
            ("p99", 0.99),
            ("p999", 0.999),
            ("max", 1.0),
        )
    }


def _write_pgm(path: Path, image: np.ndarray) -> None:
    values = np.asarray(image, dtype=np.float64)
    finite = values[np.isfinite(values)]
    output = np.zeros(values.shape, dtype=np.uint8)
    if len(finite):
        lower = float(np.quantile(finite, 0.01))
        upper = float(np.quantile(finite, 0.99))
        if upper <= lower:
            upper = lower + 1.0
        scaled = (values - lower) / (upper - lower)
        valid = np.isfinite(values)
        output[valid] = np.clip(1.0 + 254.0 * scaled[valid], 1.0, 255.0).astype(
            np.uint8
        )
    with path.open("xb") as stream:
        stream.write(f"P5\n{output.shape[1]} {output.shape[0]}\n255\n".encode("ascii"))
        stream.write(output.tobytes())


def analyze_bag(
    bag_path: str,
    topic: str,
    *,
    sample_every: int,
    maximum_sampled_frames: int,
    horizontal_bins: int,
    pair_separation_threshold: float,
) -> tuple[dict[str, Any], Optional[FrameAnalysis]]:
    if sample_every <= 0 or maximum_sampled_frames <= 0:
        raise ValueError("sample_every and maximum_sampled_frames must be positive")

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(Path(bag_path).expanduser()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    message_type_name = topic_types.get(topic)
    if message_type_name is None:
        raise ValueError(f"bag has no topic {topic!r}; available: {sorted(topic_types)}")
    if message_type_name != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"{topic} has unsupported type {message_type_name}")
    message_type = get_message(message_type_name)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    total_frames = 0
    sampled_frames = 0
    first_record_time = None
    last_record_time = None
    point_counts = set()
    shapes = set()
    field_report = None
    finite_counts = []
    ring_counts = []
    duplicate_counts = []
    separated_counts = []
    timestamp_spans = []
    intensity_samples = []
    separation_samples = []
    representative = None
    representative_score = -1
    first_header_stamp = None
    last_header_stamp = None

    while reader.has_next():
        _, serialized, record_time = reader.read_next()
        total_frames += 1
        first_record_time = record_time if first_record_time is None else first_record_time
        last_record_time = record_time
        if (total_frames - 1) % sample_every != 0:
            continue
        if sampled_frames >= maximum_sampled_frames:
            continue

        message = deserialize_message(serialized, message_type)
        sampled_frames += 1
        header_stamp = message_stamp_ns(message)
        first_header_stamp = header_stamp if first_header_stamp is None else first_header_stamp
        last_header_stamp = header_stamp
        if field_report is None:
            field_report = [
                {
                    "name": field.name,
                    "offset": int(field.offset),
                    "datatype": int(field.datatype),
                    "count": int(field.count),
                }
                for field in message.fields
            ]
        analysis = analyze_pointcloud_message(
            message,
            horizontal_bins=horizontal_bins,
            pair_separation_threshold=pair_separation_threshold,
        )
        point_counts.add(analysis.point_count)
        shapes.add((int(message.width), int(message.height), int(message.point_step)))
        finite_counts.append(analysis.finite_point_count)
        ring_counts.append(analysis.ring_count)
        duplicate_counts.append(analysis.duplicate_direction_count)
        separated_counts.append(analysis.separated_return_count)
        if analysis.timestamp_span_sec is not None:
            timestamp_spans.append(analysis.timestamp_span_sec)
        if len(analysis.intensities):
            intensity_samples.append(analysis.intensities)
        if len(analysis.separations):
            separation_samples.append(analysis.separations)
        if analysis.duplicate_direction_count > representative_score:
            representative = analysis
            representative_score = analysis.duplicate_direction_count

    if total_frames == 0:
        raise ValueError(f"bag topic {topic!r} contains no messages")
    recording_duration = (
        float(last_record_time - first_record_time) * 1.0e-9
        if last_record_time is not None and first_record_time is not None
        else 0.0
    )
    header_duration = (
        float(last_header_stamp - first_header_stamp) * 1.0e-9
        if last_header_stamp is not None
        and first_header_stamp is not None
        and last_header_stamp >= first_header_stamp
        else None
    )
    intensity_values = (
        np.concatenate(intensity_samples) if intensity_samples else np.empty(0)
    )
    separation_values = (
        np.concatenate(separation_samples) if separation_samples else np.empty(0)
    )
    summary: dict[str, Any] = {
        "bag": str(Path(bag_path).expanduser()),
        "topic": topic,
        "type": message_type_name,
        "total_frames": total_frames,
        "sample_every": sample_every,
        "sampled_frames": sampled_frames,
        "recording_duration_sec": recording_duration,
        "recording_rate_hz": (
            (total_frames - 1) / recording_duration if recording_duration > 0.0 else None
        ),
        "sampled_header_duration_sec": header_duration,
        "point_counts": sorted(point_counts),
        "shape_width_height_point_step": [list(value) for value in sorted(shapes)],
        "fields": field_report,
        "finite_points_per_sample": _quantiles(np.asarray(finite_counts)),
        "rings_per_sample": sorted(set(ring_counts)),
        "duplicate_directions_per_sample": _quantiles(np.asarray(duplicate_counts)),
        "separated_returns_per_sample": _quantiles(np.asarray(separated_counts)),
        "pair_separation_threshold_m": pair_separation_threshold,
        "pair_separation_m": _quantiles(separation_values),
        "intensity": _quantiles(intensity_values),
        "point_timestamp_span_sec": _quantiles(np.asarray(timestamp_spans)),
        "interpretation": (
            "multi-return candidates present; inspect separation image and raw packet mode"
            if any(value > 0 for value in separated_counts)
            else "no separated same-ring/same-azimuth return pairs in sampled frames"
        ),
    }
    return summary, representative


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only PointCloud2 intensity and return-pair analysis for an XT16 bag."
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--output-dir")
    parser.add_argument("--sample-every", type=int, default=100)
    parser.add_argument("--max-sampled-frames", type=int, default=100)
    parser.add_argument("--horizontal-bins", type=int, default=2000)
    parser.add_argument("--pair-separation-threshold", type=float, default=0.10)
    return parser


def main(argv=None) -> None:
    args = build_argument_parser().parse_args(argv)
    summary, representative = analyze_bag(
        args.bag,
        args.topic,
        sample_every=args.sample_every,
        maximum_sampled_frames=args.max_sampled_frames,
        horizontal_bins=args.horizontal_bins,
        pair_separation_threshold=args.pair_separation_threshold,
    )
    rendered = json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False)
    if args.output_dir:
        output_dir = Path(args.output_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=False)
        (output_dir / "summary.json").write_text(rendered + "\n", encoding="utf-8")
        if representative is not None:
            _write_pgm(output_dir / "range_image.pgm", representative.range_image)
            _write_pgm(
                output_dir / "intensity_image.pgm", representative.intensity_image
            )
            _write_pgm(
                output_dir / "return_separation_image.pgm",
                representative.separation_image,
            )
        print(f"OUTPUT_DIR={output_dir}")
    print(rendered)


if __name__ == "__main__":
    main()
