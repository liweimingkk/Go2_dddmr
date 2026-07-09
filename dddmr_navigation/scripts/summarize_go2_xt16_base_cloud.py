#!/usr/bin/env python3
"""Summarize live XT16 points in base_link without publishing anything."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


@dataclass
class CloudSummary:
    total_points: int = 0
    sampled_points: int = 0
    kept_points: int = 0
    front_count: int = 0
    back_count: int = 0
    left_count: int = 0
    right_count: int = 0
    front_band_count: int = 0
    back_band_count: int = 0
    nearest_front: tuple[float, float, float, float] | None = None
    nearest_back: tuple[float, float, float, float] | None = None
    min_x: float = math.inf
    max_x: float = -math.inf
    min_y: float = math.inf
    max_y: float = -math.inf
    min_z: float = math.inf
    max_z: float = -math.inf
    sum_x: float = 0.0
    sum_y: float = 0.0
    sum_z: float = 0.0

    def add(self, x: float, y: float, z: float, front_band_y_abs: float) -> None:
        range_xy = math.hypot(x, y)
        self.kept_points += 1
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)
        self.min_z = min(self.min_z, z)
        self.max_z = max(self.max_z, z)
        self.sum_x += x
        self.sum_y += y
        self.sum_z += z

        if x >= 0.0:
            self.front_count += 1
            if abs(y) <= front_band_y_abs:
                self.front_band_count += 1
            if self.nearest_front is None or range_xy < self.nearest_front[3]:
                self.nearest_front = (x, y, z, range_xy)
        else:
            self.back_count += 1
            if abs(y) <= front_band_y_abs:
                self.back_band_count += 1
            if self.nearest_back is None or range_xy < self.nearest_back[3]:
                self.nearest_back = (x, y, z, range_xy)

        if y >= 0.0:
            self.left_count += 1
        else:
            self.right_count += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read one live PointCloud2 sample, transform points into base_link, "
            "and summarize sectors for RViz/front-object sanity checks."
        )
    )
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--max-points", type=int, default=20000)
    parser.add_argument("--min-range", type=float, default=0.20)
    parser.add_argument("--max-range", type=float, default=8.0)
    parser.add_argument("--z-min", type=float, default=-1.5)
    parser.add_argument("--z-max", type=float, default=2.0)
    parser.add_argument("--front-band-y-abs", type=float, default=0.75)
    parser.add_argument("--min-kept-points", type=int, default=100)
    return parser.parse_args()


def quaternion_to_matrix(q) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def quaternion_to_rpy(q) -> tuple[float, float, float]:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def transform_point(matrix, translation, point: tuple[float, float, float]) -> tuple[float, float, float]:
    px, py, pz = point
    return (
        matrix[0][0] * px + matrix[0][1] * py + matrix[0][2] * pz + translation.x,
        matrix[1][0] * px + matrix[1][1] * py + matrix[1][2] * pz + translation.y,
        matrix[2][0] * px + matrix[2][1] * py + matrix[2][2] * pz + translation.z,
    )


class CloudCollector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("summarize_go2_xt16_base_cloud")
        self.args = args
        self.cloud: PointCloud2 | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, args.topic, self._cloud_cb, qos_profile_sensor_data)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if self.cloud is None:
            self.cloud = msg

    def wait_for_cloud(self) -> PointCloud2:
        deadline = time.monotonic() + self.args.timeout_sec
        while time.monotonic() < deadline and self.cloud is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cloud is None:
            raise RuntimeError(f"no cloud received on {self.args.topic}")
        return self.cloud

    def lookup_transform(self, source_frame: str):
        deadline = time.monotonic() + self.args.timeout_sec
        last_error = ""
        while time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(self.args.target_frame, source_frame, Time())
            except Exception as exc:  # tf2 exposes multiple exception classes.
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"could not lookup {self.args.target_frame}<-{source_frame}: {last_error}")


def summarize_cloud(msg: PointCloud2, transform, args: argparse.Namespace) -> CloudSummary:
    summary = CloudSummary(total_points=int(msg.width) * int(msg.height))
    matrix = quaternion_to_matrix(transform.transform.rotation)
    translation = transform.transform.translation
    stride = max(1, math.ceil(max(summary.total_points, 1) / max(args.max_points, 1)))

    for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
        if index % stride != 0:
            continue
        lx, ly, lz = float(point[0]), float(point[1]), float(point[2])
        x, y, z = transform_point(matrix, translation, (lx, ly, lz))
        range_xy = math.hypot(x, y)
        summary.sampled_points += 1
        if range_xy < args.min_range or range_xy > args.max_range:
            continue
        if z < args.z_min or z > args.z_max:
            continue
        summary.add(x, y, z, args.front_band_y_abs)
    return summary


def print_point(prefix: str, point: tuple[float, float, float, float] | None) -> None:
    if point is None:
        print(f"{prefix}=none")
        return
    x, y, z, range_xy = point
    print(f"{prefix}_X={x:.6f}")
    print(f"{prefix}_Y={y:.6f}")
    print(f"{prefix}_Z={z:.6f}")
    print(f"{prefix}_RANGE_XY={range_xy:.6f}")


def main() -> int:
    args = parse_args()
    if args.timeout_sec <= 0.0:
        raise SystemExit("--timeout-sec must be > 0")

    rclpy.init()
    node = CloudCollector(args)
    try:
        cloud = node.wait_for_cloud()
        transform = node.lookup_transform(cloud.header.frame_id)
        summary = summarize_cloud(cloud, transform, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    roll, pitch, yaw = quaternion_to_rpy(rotation)
    passed = summary.kept_points >= args.min_kept_points

    print(f"BASE_CLOUD_TOPIC={args.topic}")
    print(f"BASE_CLOUD_SOURCE_FRAME={cloud.header.frame_id}")
    print(f"BASE_CLOUD_TARGET_FRAME={args.target_frame}")
    print(f"BASE_CLOUD_STAMP_SEC={cloud.header.stamp.sec}.{cloud.header.stamp.nanosec:09d}")
    print(f"BASE_CLOUD_TOTAL_POINTS={summary.total_points}")
    print(f"BASE_CLOUD_SAMPLED_POINTS={summary.sampled_points}")
    print(f"BASE_CLOUD_KEPT_POINTS={summary.kept_points}")
    print(f"BASE_CLOUD_TRANSFORM_XYZ={translation.x:.6f},{translation.y:.6f},{translation.z:.6f}")
    print(
        "BASE_CLOUD_TRANSFORM_RPY_DEG="
        f"{math.degrees(roll):.3f},{math.degrees(pitch):.3f},{math.degrees(yaw):.3f}"
    )
    if summary.kept_points > 0:
        print(f"BASE_CLOUD_MIN_X={summary.min_x:.6f}")
        print(f"BASE_CLOUD_MAX_X={summary.max_x:.6f}")
        print(f"BASE_CLOUD_MIN_Y={summary.min_y:.6f}")
        print(f"BASE_CLOUD_MAX_Y={summary.max_y:.6f}")
        print(f"BASE_CLOUD_MIN_Z={summary.min_z:.6f}")
        print(f"BASE_CLOUD_MAX_Z={summary.max_z:.6f}")
        print(f"BASE_CLOUD_CENTROID_X={summary.sum_x / summary.kept_points:.6f}")
        print(f"BASE_CLOUD_CENTROID_Y={summary.sum_y / summary.kept_points:.6f}")
        print(f"BASE_CLOUD_CENTROID_Z={summary.sum_z / summary.kept_points:.6f}")
    print(f"BASE_CLOUD_FRONT_COUNT={summary.front_count}")
    print(f"BASE_CLOUD_BACK_COUNT={summary.back_count}")
    print(f"BASE_CLOUD_LEFT_COUNT={summary.left_count}")
    print(f"BASE_CLOUD_RIGHT_COUNT={summary.right_count}")
    print(f"BASE_CLOUD_FRONT_BAND_COUNT={summary.front_band_count}")
    print(f"BASE_CLOUD_BACK_BAND_COUNT={summary.back_band_count}")
    print_point("BASE_CLOUD_NEAREST_FRONT", summary.nearest_front)
    print_point("BASE_CLOUD_NEAREST_BACK", summary.nearest_back)
    print(f"BASE_CLOUD_STATUS={'PASS' if passed else 'FAIL'}")
    if passed:
        print("BASE_CLOUD_NOTE=manual_rviz_front_object_check_still_required")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
