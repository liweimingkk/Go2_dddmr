"""Export a downsampled odom-frame XYZI cloud from a calibration bag."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from .pointcloud import PointCloudLayout, message_stamp_ns
from .transforms import RigidTransform


def nearest_pose_index(stamps_ns: np.ndarray, target_ns: int) -> int:
    if stamps_ns.ndim != 1 or not len(stamps_ns):
        raise ValueError("odometry stamp array must not be empty")
    insertion = int(np.searchsorted(stamps_ns, target_ns, side="left"))
    candidates = []
    if insertion < len(stamps_ns):
        candidates.append(insertion)
    if insertion > 0:
        candidates.append(insertion - 1)
    return min(candidates, key=lambda index: abs(int(stamps_ns[index]) - target_ns))


def _open_reader(bag_path: str):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(Path(bag_path).expanduser()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def _load_odometry(
    bag_path: str,
    odometry_topic: str,
    *,
    time_offset_sec: float,
    expected_parent_frame: str,
) -> tuple[np.ndarray, list[RigidTransform]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _open_reader(bag_path)
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    type_name = topic_types.get(odometry_topic)
    if type_name != "nav_msgs/msg/Odometry":
        raise ValueError(
            f"{odometry_topic} type is {type_name!r}, expected nav_msgs/msg/Odometry"
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[odometry_topic]))
    message_type = get_message(type_name)
    offset_ns = int(time_offset_sec * 1_000_000_000)
    samples = []
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, message_type)
        parent_frame = message.header.frame_id.lstrip("/")
        if parent_frame != expected_parent_frame.lstrip("/"):
            raise ValueError(
                f"odometry parent frame is {parent_frame!r}, expected {expected_parent_frame!r}"
            )
        pose = message.pose.pose
        samples.append(
            (
                message_stamp_ns(message) + offset_ns,
                RigidTransform.from_quaternion(
                    [pose.position.x, pose.position.y, pose.position.z],
                    [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ],
                ),
            )
        )
    if not samples:
        raise ValueError(f"bag contains no messages on {odometry_topic}")
    samples.sort(key=lambda item: item[0])
    return np.asarray([item[0] for item in samples], dtype=np.int64), [
        item[1] for item in samples
    ]


def _write_ascii_xyzi(path: Path, points: np.ndarray) -> None:
    with path.open("x", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z intensity\n")
        stream.write("SIZE 4 4 4 4\n")
        stream.write("TYPE F F F F\n")
        stream.write("COUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\n")
        stream.write("HEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\n")
        stream.write("DATA ascii\n")
        np.savetxt(stream, points, fmt="%.7g %.7g %.7g %.7g")


def export_odom_cloud(
    bag_path: str,
    output_path: str,
    *,
    lidar_topic: str,
    odometry_topic: str,
    odom_frame: str,
    expected_lidar_frame: str,
    base_to_sensor: RigidTransform,
    odom_time_offset_sec: float,
    maximum_odom_delta_sec: float,
    sample_every: int,
    maximum_clouds: int,
    minimum_range: float,
    maximum_range: float,
    voxel_size: float,
) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    output = Path(output_path).expanduser()
    if output.exists():
        raise ValueError(f"refusing to overwrite existing output: {output}")
    if sample_every <= 0 or maximum_clouds < 0:
        raise ValueError("sample_every must be positive and maximum_clouds non-negative")
    if not 0.0 <= minimum_range < maximum_range or not np.isfinite(maximum_range):
        raise ValueError("range limits must be finite and satisfy 0 <= minimum < maximum")
    if voxel_size <= 0.0 or not np.isfinite(voxel_size):
        raise ValueError("voxel_size must be finite and positive")
    if maximum_odom_delta_sec < 0.0 or not np.isfinite(maximum_odom_delta_sec):
        raise ValueError("maximum_odom_delta_sec must be finite and non-negative")

    odom_stamps, odom_poses = _load_odometry(
        bag_path,
        odometry_topic,
        time_offset_sec=odom_time_offset_sec,
        expected_parent_frame=odom_frame,
    )
    reader = _open_reader(bag_path)
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    type_name = topic_types.get(lidar_topic)
    if type_name != "sensor_msgs/msg/PointCloud2":
        raise ValueError(
            f"{lidar_topic} type is {type_name!r}, expected sensor_msgs/msg/PointCloud2"
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[lidar_topic]))
    message_type = get_message(type_name)

    selected_clouds = 0
    encountered_clouds = 0
    skipped_for_time = 0
    accumulated = []
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        encountered_clouds += 1
        if (encountered_clouds - 1) % sample_every != 0:
            continue
        if maximum_clouds and selected_clouds >= maximum_clouds:
            break
        message = deserialize_message(serialized, message_type)
        frame_id = message.header.frame_id.lstrip("/")
        if frame_id != expected_lidar_frame.lstrip("/"):
            raise ValueError(
                f"lidar frame is {frame_id!r}, expected {expected_lidar_frame!r}"
            )
        lidar_stamp = message_stamp_ns(message)
        pose_index = nearest_pose_index(odom_stamps, lidar_stamp)
        delta_ns = abs(int(odom_stamps[pose_index]) - lidar_stamp)
        if delta_ns > int(maximum_odom_delta_sec * 1_000_000_000):
            skipped_for_time += 1
            continue

        layout = PointCloudLayout.from_message(message)
        sensor_xyz = layout.read_xyz(message.data)
        ranges = np.linalg.norm(sensor_xyz, axis=1)
        valid = (
            np.all(np.isfinite(sensor_xyz), axis=1)
            & (ranges >= minimum_range)
            & (ranges <= maximum_range)
        )
        if not np.any(valid):
            continue
        odom_from_sensor = base_to_sensor.then(odom_poses[pose_index])
        odom_xyz = odom_from_sensor.apply(sensor_xyz[valid])
        if "intensity" in layout.fields:
            intensity = layout.read_field(message.data, "intensity").astype(
                np.float64, copy=False
            )[valid]
        else:
            intensity = np.zeros(len(odom_xyz), dtype=np.float64)
        accumulated.append(np.column_stack((odom_xyz, intensity)))
        selected_clouds += 1

    if not accumulated:
        raise ValueError("no lidar clouds passed the odometry/time/range checks")
    all_points = np.concatenate(accumulated, axis=0)
    voxel_keys = np.floor(all_points[:, :3] / voxel_size).astype(np.int64)
    _, retained_indices = np.unique(voxel_keys, axis=0, return_index=True)
    retained = all_points[np.sort(retained_indices)]
    output.parent.mkdir(parents=True, exist_ok=True)
    _write_ascii_xyzi(output, retained)
    return {
        "output": str(output),
        "encountered_clouds": encountered_clouds,
        "selected_clouds": selected_clouds,
        "skipped_for_time": skipped_for_time,
        "input_points": len(all_points),
        "output_points": len(retained),
        "voxel_size": voxel_size,
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export a read-only, downsampled odom-frame PCD from raw XT16 and "
            "odometry topics for glass-plane selection."
        )
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--lidar-topic", default="/lidar_points")
    parser.add_argument("--odometry-topic", default="/utlidar/robot_odom")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--expected-lidar-frame", default="hesai_lidar")
    parser.add_argument(
        "--base-to-sensor-translation",
        type=float,
        nargs=3,
        default=[0.14543, 0.0, 0.13312],
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--base-to-sensor-rpy",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 1.57079632679],
        metavar=("ROLL", "PITCH", "YAW"),
    )
    parser.add_argument("--odom-time-offset-sec", type=float, default=0.0)
    parser.add_argument("--maximum-odom-delta-sec", type=float, default=0.05)
    parser.add_argument("--sample-every", type=int, default=5)
    parser.add_argument("--maximum-clouds", type=int, default=60)
    parser.add_argument("--minimum-range", type=float, default=0.5)
    parser.add_argument("--maximum-range", type=float, default=100.0)
    parser.add_argument("--voxel-size", type=float, default=0.10)
    return parser


def main(argv=None) -> None:
    args = build_argument_parser().parse_args(argv)
    base_to_sensor = RigidTransform.from_rpy(
        args.base_to_sensor_translation,
        args.base_to_sensor_rpy[0],
        args.base_to_sensor_rpy[1],
        args.base_to_sensor_rpy[2],
    )
    result = export_odom_cloud(
        args.bag,
        args.output,
        lidar_topic=args.lidar_topic,
        odometry_topic=args.odometry_topic,
        odom_frame=args.odom_frame,
        expected_lidar_frame=args.expected_lidar_frame,
        base_to_sensor=base_to_sensor,
        odom_time_offset_sec=args.odom_time_offset_sec,
        maximum_odom_delta_sec=args.maximum_odom_delta_sec,
        sample_every=args.sample_every,
        maximum_clouds=args.maximum_clouds,
        minimum_range=args.minimum_range,
        maximum_range=args.maximum_range,
        voxel_size=args.voxel_size,
    )
    for key, value in result.items():
        print(f"{key.upper()}={value}")


if __name__ == "__main__":
    main()
