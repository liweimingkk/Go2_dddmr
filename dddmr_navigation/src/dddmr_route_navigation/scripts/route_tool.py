#!/usr/bin/env python3
"""Create, inspect, publish, or record a DDDMR 3D route.

The conversion and validation paths intentionally have no ROS dependency so a
saved pose graph can be checked before a robot stack is started.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Dict, List, Optional, Sequence, Tuple


SCHEMA = "dddmr-recorded-route/v1"
MAP_FINGERPRINT_FILES = ("poses.pcd", "map.pcd", "ground.pcd", "edges.pcd")


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, ...]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


def yaw_from_quaternion(point: Dict[str, float]) -> float:
    qx = float(point["qx"])
    qy = float(point["qy"])
    qz = float(point["qz"])
    qw = float(point["qw"])
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def angular_distance(lhs: float, rhs: float) -> float:
    return abs(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


def distance3d(lhs: Dict[str, float], rhs: Dict[str, float]) -> float:
    return math.sqrt(
        (float(lhs["x"]) - float(rhs["x"])) ** 2
        + (float(lhs["y"]) - float(rhs["y"])) ** 2
        + (float(lhs["z"]) - float(rhs["z"])) ** 2
    )


def read_ascii_pcd(path: Path) -> Tuple[List[str], List[Dict[str, float]]]:
    fields: Optional[List[str]] = None
    rows: List[Dict[str, float]] = []
    data_ascii = False
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not data_ascii:
                if not line or line.startswith("#"):
                    continue
                tokens = line.split()
                keyword = tokens[0].upper()
                if keyword == "FIELDS":
                    fields = tokens[1:]
                elif keyword == "DATA":
                    if len(tokens) != 2 or tokens[1].lower() != "ascii":
                        raise ValueError("only DATA ascii PCD files are supported")
                    if not fields:
                        raise ValueError("PCD FIELDS header is missing")
                    data_ascii = True
                continue

            if not line or line.startswith("#"):
                continue
            values = line.split()
            if len(values) != len(fields):
                raise ValueError(
                    f"PCD line {line_number} has {len(values)} values; "
                    f"expected {len(fields)}"
                )
            try:
                row = {name: float(value) for name, value in zip(fields, values)}
            except ValueError as exc:
                raise ValueError(f"PCD line {line_number} contains a non-number") from exc
            rows.append(row)

    if not data_ascii:
        raise ValueError("PCD DATA ascii header is missing")
    required = {"x", "y", "z", "roll", "pitch", "yaw"}
    missing = required.difference(fields or [])
    if missing:
        raise ValueError(f"PCD is missing required fields: {sorted(missing)}")
    if not rows:
        raise ValueError("PCD contains no poses")
    return fields or [], rows


def points_from_pose_graph(path: Path) -> List[Dict[str, float]]:
    _, rows = read_ascii_pcd(path)
    points: List[Dict[str, float]] = []
    for index, row in enumerate(rows):
        values = (row["x"], row["y"], row["z"], row["roll"], row["pitch"], row["yaw"])
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"pose graph row {index} contains a non-finite value")
        qx, qy, qz, qw = quaternion_from_rpy(row["roll"], row["pitch"], row["yaw"])
        points.append(
            {
                "x": row["x"],
                "y": row["y"],
                "z": row["z"],
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
            }
        )
    return points


def filter_duplicate_points(
    points: Sequence[Dict[str, float]], duplicate_distance: float
) -> List[Dict[str, float]]:
    if not math.isfinite(duplicate_distance) or duplicate_distance < 0.0:
        raise ValueError("duplicate_distance must be finite and non-negative")
    filtered: List[Dict[str, float]] = []
    for point in points:
        current = dict(point)
        if filtered and distance3d(filtered[-1], current) <= duplicate_distance:
            # Keep the distinct position but retain the most recent orientation.
            for field in ("qx", "qy", "qz", "qw"):
                filtered[-1][field] = current[field]
            continue
        filtered.append(current)
    return filtered


def route_length(points: Sequence[Dict[str, float]]) -> float:
    return sum(distance3d(start, end) for start, end in zip(points, points[1:]))


def validate_points(
    points: Sequence[Dict[str, float]], max_segment_length: float = 2.0
) -> None:
    if len(points) < 3:
        raise ValueError("a route requires at least three distinct points")
    if not math.isfinite(max_segment_length) or max_segment_length <= 0.0:
        raise ValueError("max_segment_length must be finite and positive")
    required = ("x", "y", "z", "qx", "qy", "qz", "qw")
    for index, point in enumerate(points):
        missing = [field for field in required if field not in point]
        if missing:
            raise ValueError(f"route point {index} is missing {missing}")
        if not all(_finite(point[field]) for field in required):
            raise ValueError(f"route point {index} contains a non-finite value")
        norm = math.sqrt(sum(float(point[field]) ** 2 for field in ("qx", "qy", "qz", "qw")))
        if norm < 1.0e-6 or abs(norm - 1.0) > 1.0e-3:
            raise ValueError(f"route point {index} quaternion is not normalized")
    for index, (start, end) in enumerate(zip(points, points[1:])):
        segment = distance3d(start, end)
        if segment > max_segment_length:
            raise ValueError(
                f"route segment {index}->{index + 1} is {segment:.3f} m; "
                f"maximum is {max_segment_length:.3f} m"
            )
    if route_length(points) <= 0.0:
        raise ValueError("route length is zero")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def map_fingerprint(map_dir: Optional[Path]) -> Optional[Dict[str, object]]:
    if map_dir is None:
        return None
    map_dir = map_dir.resolve()
    if not map_dir.is_dir():
        raise ValueError(f"map directory does not exist: {map_dir}")
    files: Dict[str, str] = {}
    digest = hashlib.sha256()
    for relative_name in MAP_FINGERPRINT_FILES:
        candidate = map_dir / relative_name
        if not candidate.is_file():
            continue
        file_hash = sha256_file(candidate)
        files[relative_name] = file_hash
        digest.update(relative_name.encode("utf-8"))
        digest.update(file_hash.encode("ascii"))
    if not files:
        raise ValueError(f"map directory has none of {MAP_FINGERPRINT_FILES}: {map_dir}")
    return {"directory": str(map_dir), "sha256": digest.hexdigest(), "files": files}


def build_document(
    points: Sequence[Dict[str, float]],
    route_id: str,
    frame_id: str,
    source: Dict[str, object],
    map_info: Optional[Dict[str, object]],
    max_segment_length: float,
) -> Dict[str, object]:
    if not route_id.strip():
        raise ValueError("route_id must not be empty")
    if not frame_id.strip():
        raise ValueError("frame_id must not be empty")
    validate_points(points, max_segment_length)
    document: Dict[str, object] = {
        "schema": SCHEMA,
        "route_id": route_id,
        "frame_id": frame_id,
        "source": source,
        "route_length_3d_m": route_length(points),
        "points": [dict(point) for point in points],
    }
    if map_info is not None:
        document["map"] = map_info
    return document


def validate_document(document: object, max_segment_length: float = 2.0) -> Dict[str, object]:
    if not isinstance(document, dict):
        raise ValueError("route document must be a JSON object")
    if document.get("schema") != SCHEMA:
        raise ValueError(f"unsupported route schema: {document.get('schema')!r}")
    if not isinstance(document.get("route_id"), str) or not document["route_id"].strip():
        raise ValueError("route_id must be a non-empty string")
    if not isinstance(document.get("frame_id"), str) or not document["frame_id"].strip():
        raise ValueError("frame_id must be a non-empty string")
    points = document.get("points")
    if not isinstance(points, list):
        raise ValueError("points must be an array")
    validate_points(points, max_segment_length)
    return document


def load_document(path: Path, max_segment_length: float = 2.0) -> Dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    return validate_document(document, max_segment_length)


def write_document(path: Path, document: Dict[str, object]) -> None:
    validate_document(document)
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            # Route files are shared from a root-run Docker container back to
            # the host workspace.  mkstemp defaults to 0600, which would make
            # a successful recording unreadable to the host operator.
            os.fchmod(stream.fileno(), 0o644)
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def convert_pose_graph(args: argparse.Namespace) -> int:
    input_path = args.input.resolve()
    points = filter_duplicate_points(
        points_from_pose_graph(input_path), args.duplicate_distance
    )
    document = build_document(
        points=points,
        route_id=args.route_id or args.output.stem,
        frame_id=args.frame_id,
        source={
            "type": "dddmr_pose_graph_pcd",
            "path": str(input_path),
            "sha256": sha256_file(input_path),
        },
        map_info=map_fingerprint(args.map_dir),
        max_segment_length=args.max_segment_length,
    )
    write_document(args.output, document)
    print(
        f"wrote {args.output}: {len(points)} points, "
        f"{route_length(points):.3f} m in frame {args.frame_id}"
    )
    return 0


def inspect_route(args: argparse.Namespace) -> int:
    document = load_document(args.route_file, args.max_segment_length)
    points = document["points"]
    print(
        json.dumps(
            {
                "route_id": document["route_id"],
                "frame_id": document["frame_id"],
                "point_count": len(points),
                "route_length_3d_m": route_length(points),
                "map_sha256": (document.get("map") or {}).get("sha256"),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def publish_route(args: argparse.Namespace) -> int:
    if not args.topic.strip():
        raise ValueError("topic must not be empty")
    if not math.isfinite(args.wait_seconds) or args.wait_seconds <= 0.0:
        raise ValueError("wait_seconds must be finite and positive")
    document = load_document(args.route_file, args.max_segment_length)
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as PathMessage
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python packages are required for publish") from exc

    class RoutePublisher(Node):
        def __init__(self) -> None:
            super().__init__("recorded_route_file_publisher")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.publisher = self.create_publisher(PathMessage, args.topic, qos)

        def message(self) -> PathMessage:
            message = PathMessage()
            message.header.frame_id = str(document["frame_id"])
            message.header.stamp = self.get_clock().now().to_msg()
            for point_value in document["points"]:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = float(point_value["x"])
                pose.pose.position.y = float(point_value["y"])
                pose.pose.position.z = float(point_value["z"])
                pose.pose.orientation.x = float(point_value["qx"])
                pose.pose.orientation.y = float(point_value["qy"])
                pose.pose.orientation.z = float(point_value["qz"])
                pose.pose.orientation.w = float(point_value["qw"])
                message.poses.append(pose)
            return message

    rclpy.init()
    node = RoutePublisher()
    try:
        message = node.message()
        deadline = time.monotonic() + args.wait_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            node.publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.10)
        print(f"published {len(message.poses)} route poses on {args.topic}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def record_route(args: argparse.Namespace) -> int:
    if not math.isfinite(args.sample_distance) or args.sample_distance <= 0.0:
        raise ValueError("sample_distance must be finite and positive")
    if not math.isfinite(args.sample_yaw) or args.sample_yaw <= 0.0:
        raise ValueError("sample_yaw must be finite and positive")
    if not math.isfinite(args.duplicate_distance) or args.duplicate_distance < 0.0:
        raise ValueError("duplicate_distance must be finite and non-negative")
    if not math.isfinite(args.max_segment_length) or args.max_segment_length <= 0.0:
        raise ValueError("max_segment_length must be finite and positive")
    try:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python packages are required for record") from exc

    recorded: List[Dict[str, float]] = []

    class RouteRecorder(Node):
        def __init__(self) -> None:
            super().__init__("recorded_route_recorder")
            self.localization_status = ""
            self.create_subscription(String, args.status_topic, self.status_callback, 10)
            self.create_subscription(
                PoseWithCovarianceStamped, args.pose_topic, self.pose_callback, 10
            )

        def status_callback(self, message: String) -> None:
            self.localization_status = message.data.strip().upper()

        def pose_callback(self, message: PoseWithCovarianceStamped) -> None:
            if self.localization_status != "TRACKING":
                return
            if message.header.frame_id and message.header.frame_id != args.frame_id:
                self.get_logger().error(
                    f"ignoring pose in frame {message.header.frame_id!r}; "
                    f"expected {args.frame_id!r}"
                )
                return
            pose = message.pose.pose
            point = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "qx": float(pose.orientation.x),
                "qy": float(pose.orientation.y),
                "qz": float(pose.orientation.z),
                "qw": float(pose.orientation.w),
            }
            try:
                required = ("x", "y", "z", "qx", "qy", "qz", "qw")
                if not all(_finite(point[field]) for field in required):
                    raise ValueError("pose contains a non-finite value")
                quaternion_norm = math.sqrt(
                    sum(point[field] ** 2 for field in ("qx", "qy", "qz", "qw"))
                )
                if quaternion_norm < 1.0e-6:
                    raise ValueError("pose quaternion has zero norm")
                for field in ("qx", "qy", "qz", "qw"):
                    point[field] /= quaternion_norm
                if recorded and distance3d(recorded[-1], point) > args.max_segment_length:
                    raise ValueError("pose gap exceeds max_segment_length")
            except ValueError as exc:
                self.get_logger().error(f"ignoring invalid pose: {exc}")
                return
            if recorded:
                moved = distance3d(recorded[-1], point) >= args.sample_distance
                turned = angular_distance(
                    yaw_from_quaternion(recorded[-1]), yaw_from_quaternion(point)
                ) >= args.sample_yaw
                if not moved and not turned:
                    return
            recorded.append(point)
            self.get_logger().info(f"recorded route point {len(recorded)}")

    rclpy.init()
    node = RouteRecorder()
    print(
        "recording localization poses only while status is TRACKING; "
        "drive manually and press Ctrl-C to save"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # Humble's SIGINT handler may already have shut down the default
        # context before spin() raises KeyboardInterrupt.  try_shutdown() is
        # idempotent, so route serialization below still runs after Ctrl-C.
        rclpy.try_shutdown()

    filtered = filter_duplicate_points(recorded, args.duplicate_distance)
    document = build_document(
        points=filtered,
        route_id=args.route_id or args.output.stem,
        frame_id=args.frame_id,
        source={
            "type": "localized_manual_traverse",
            "pose_topic": args.pose_topic,
            "status_topic": args.status_topic,
        },
        map_info=map_fingerprint(args.map_dir),
        max_segment_length=args.max_segment_length,
    )
    write_document(args.output, document)
    print(
        f"wrote {args.output}: {len(filtered)} points, "
        f"{route_length(filtered):.3f} m"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    convert = subparsers.add_parser(
        "convert-pose-graph", help="convert an ASCII DDDMR poses.pcd to route JSON"
    )
    convert.add_argument("input", type=Path)
    convert.add_argument("output", type=Path)
    convert.add_argument("--route-id")
    convert.add_argument("--frame-id", default="map")
    convert.add_argument("--map-dir", type=Path)
    convert.add_argument("--duplicate-distance", type=float, default=0.02)
    convert.add_argument("--max-segment-length", type=float, default=2.0)
    convert.set_defaults(handler=convert_pose_graph)

    inspect_parser = subparsers.add_parser("inspect", help="validate and summarize route JSON")
    inspect_parser.add_argument("route_file", type=Path)
    inspect_parser.add_argument("--max-segment-length", type=float, default=2.0)
    inspect_parser.set_defaults(handler=inspect_route)

    publish = subparsers.add_parser("publish", help="publish route JSON as transient nav_msgs/Path")
    publish.add_argument("route_file", type=Path)
    publish.add_argument("--topic", default="/recorded_route")
    publish.add_argument("--wait-seconds", type=float, default=2.0)
    publish.add_argument("--max-segment-length", type=float, default=2.0)
    publish.set_defaults(handler=publish_route)

    record = subparsers.add_parser(
        "record", help="record localized poses while an operator manually drives"
    )
    record.add_argument("output", type=Path)
    record.add_argument("--route-id")
    record.add_argument("--frame-id", default="map")
    record.add_argument("--map-dir", type=Path, required=True)
    record.add_argument("--pose-topic", default="/mcl_pose")
    record.add_argument("--status-topic", default="/localization_status")
    record.add_argument("--sample-distance", type=float, default=0.10)
    record.add_argument("--sample-yaw", type=float, default=0.15)
    record.add_argument("--duplicate-distance", type=float, default=0.02)
    record.add_argument("--max-segment-length", type=float, default=2.0)
    record.set_defaults(handler=record_route)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args, unknown = parser.parse_known_args(argv)
    if unknown and "--ros-args" not in unknown:
        parser.error(f"unrecognized arguments: {' '.join(unknown)}")
    try:
        return int(args.handler(args))
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
