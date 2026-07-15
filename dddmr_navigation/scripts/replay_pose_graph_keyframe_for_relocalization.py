#!/usr/bin/env python3

"""Replay one saved pose-graph keyframe as a stationary MCL observation.

This is an offline/no-motion diagnostic. It publishes saved feature, surface,
and ground PCDs plus identity odometry; it never publishes velocity or Unitree
requests.
"""

import argparse
import pathlib
from typing import List, Sequence, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


Point = Tuple[float, float, float, float]


def read_ascii_pcd(path: pathlib.Path) -> List[Point]:
    fields: Sequence[str] = ()
    data_kind = ""
    rows: List[Point] = []
    with path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if not data_kind:
                key, *values = line.split()
                if key == "FIELDS":
                    fields = values
                elif key == "DATA":
                    data_kind = values[0].lower()
                    if data_kind != "ascii":
                        raise ValueError(f"{path} uses unsupported PCD DATA {data_kind}")
                continue

            values = [float(value) for value in line.split()]
            by_name = dict(zip(fields, values))
            rows.append(
                (
                    by_name["x"],
                    by_name["y"],
                    by_name["z"],
                    by_name.get("intensity", 1.0),
                )
            )
    if not data_kind or not rows:
        raise ValueError(f"{path} is not a non-empty ASCII PCD")
    return rows


class KeyframeReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pose_graph_keyframe_replay")
        pcd_dir = pathlib.Path(args.pose_graph_dir) / "pcd"
        self.feature_points = read_ascii_pcd(
            pcd_dir / f"{args.keyframe}_feature.pcd"
        )
        self.ground_points = read_ascii_pcd(
            pcd_dir / f"{args.keyframe}_ground.pcd"
        )
        self.surface_points = read_ascii_pcd(
            pcd_dir / f"{args.keyframe}_surface.pcd"
        )
        self.frame_id = args.frame_id
        self.end_ns = self.get_clock().now().nanoseconds + int(args.duration * 1e9)
        self.finished = False

        self.cloud_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        self.sharp_pub = self.create_publisher(
            PointCloud2, "/laser_cloud_sharp", 5
        )
        self.less_sharp_pub = self.create_publisher(
            PointCloud2, "/laser_cloud_less_sharp", 5
        )
        self.flat_pub = self.create_publisher(
            PointCloud2, "/laser_cloud_flat", 5
        )
        self.less_flat_pub = self.create_publisher(
            PointCloud2, "/laser_cloud_less_flat", 5
        )
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.create_timer(1.0 / args.rate, self.publish_sample)

        self.get_logger().info(
            "Offline keyframe %d: %d feature, %d surface, %d ground points for %.1fs"
            % (
                args.keyframe,
                len(self.feature_points),
                len(self.surface_points),
                len(self.ground_points),
                args.duration,
            )
        )

    def publish_sample(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds >= self.end_ns:
            self.finished = True
            return

        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.frame_id
        feature = point_cloud2.create_cloud(
            header, self.cloud_fields, self.feature_points
        )
        ground = point_cloud2.create_cloud(
            header, self.cloud_fields, self.ground_points
        )
        surface = point_cloud2.create_cloud(
            header, self.cloud_fields, self.surface_points
        )
        self.sharp_pub.publish(feature)
        self.less_sharp_pub.publish(feature)
        self.flat_pub.publish(ground)
        self.less_flat_pub.publish(surface)

        odom = Odometry()
        odom.header = header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose_graph_dir")
    parser.add_argument("--keyframe", type=int, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument(
        "--odom-topic", default="/dddmr_go2/robot_odom_standard"
    )
    args = parser.parse_args()
    if args.keyframe < 0 or args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("keyframe must be non-negative; duration and rate must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = KeyframeReplay(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
