"""ROS 2 node that replaces returns behind known glass with plane hits."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import PlaneConfiguration, load_plane_configuration
from .geometry import filter_points_behind_planes, sample_plane_surface
from .pointcloud import PointCloudLayout, message_stamp_ns
from .transforms import PoseBuffer, PoseSample, RigidTransform


def _frame_id(value: str) -> str:
    return value.lstrip("/")


def _rigid_from_transform(transform) -> RigidTransform:
    translation = transform.translation
    rotation = transform.rotation
    return RigidTransform.from_quaternion(
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )


def _rigid_from_pose(pose) -> RigidTransform:
    position = pose.position
    orientation = pose.orientation
    return RigidTransform.from_quaternion(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )


class GlassPlaneFilter(Node):
    def __init__(self) -> None:
        super().__init__("glass_plane_filter")

        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.input_topic = str(
            self.declare_parameter("input_topic", "/lidar_points").value
        )
        self.output_topic = str(
            self.declare_parameter(
                "output_topic", "/lidar_points_glass_filtered"
            ).value
        )
        self.obstacle_topic = str(
            self.declare_parameter(
                "obstacle_topic", "/glass_plane_obstacles"
            ).value
        )
        self.plane_config_file = str(
            self.declare_parameter("plane_config_file", "").value
        )
        self.pose_source = str(
            self.declare_parameter("pose_source", "odometry").value
        ).lower()
        self.odometry_topic = str(
            self.declare_parameter(
                "odometry_topic", "/dddmr_go2/robot_odom_standard"
            ).value
        )
        self.base_frame = _frame_id(
            str(self.declare_parameter("base_frame", "base_link").value)
        )
        self.filtered_point_action = str(
            self.declare_parameter(
                "filtered_point_action", "glass_surface"
            ).value
        ).lower()
        self.minimum_behind_distance = float(
            self.declare_parameter("minimum_behind_distance", 0.15).value
        )
        self.boundary_tolerance = float(
            self.declare_parameter("boundary_tolerance", 0.05).value
        )
        self.maximum_odom_delta_sec = float(
            self.declare_parameter("maximum_odom_delta_sec", 0.05).value
        )
        self.transform_timeout_sec = float(
            self.declare_parameter("transform_timeout_sec", 0.10).value
        )
        self.surface_point_spacing = float(
            self.declare_parameter("surface_point_spacing", 0.10).value
        )

        self._validate_parameters()
        self.configuration: Optional[PlaneConfiguration] = None
        if self.enabled:
            self.configuration = load_plane_configuration(self.plane_config_file)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_buffer = PoseBuffer(maximum_size=1000)
        self.frames_received = 0
        self.frames_published = 0
        self.frames_dropped = 0
        self.points_replaced = 0
        self._last_log_ns: dict[str, int] = {}

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        obstacle_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos
        )
        self.obstacle_publisher = self.create_publisher(
            PointCloud2, self.obstacle_topic, obstacle_qos
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.odom_subscription = None
        if self.enabled and self.pose_source == "odometry":
            self.odom_subscription = self.create_subscription(
                Odometry,
                self.odometry_topic,
                self._odom_callback,
                qos_profile_sensor_data,
            )

        if self.enabled:
            self._publish_obstacle_reference()
            self.get_logger().info(
                "glass filtering ENABLED: %s -> %s, %d plane(s) in %s, "
                "pose_source=%s, action=%s"
                % (
                    self.input_topic,
                    self.output_topic,
                    len(self.configuration.planes),
                    self.configuration.frame_id,
                    self.pose_source,
                    self.filtered_point_action,
                )
            )
        else:
            self.get_logger().warning(
                "glass filtering is disabled; %s is a byte-for-byte passthrough of %s"
                % (self.output_topic, self.input_topic)
            )

    def _validate_parameters(self) -> None:
        if self.input_topic == self.output_topic:
            raise ValueError("input_topic and output_topic must be different")
        if self.pose_source not in ("tf", "odometry"):
            raise ValueError("pose_source must be 'tf' or 'odometry'")
        if self.filtered_point_action not in ("glass_surface", "nan"):
            raise ValueError("filtered_point_action must be 'glass_surface' or 'nan'")
        for label, value in (
            ("minimum_behind_distance", self.minimum_behind_distance),
            ("boundary_tolerance", self.boundary_tolerance),
            ("maximum_odom_delta_sec", self.maximum_odom_delta_sec),
            ("transform_timeout_sec", self.transform_timeout_sec),
        ):
            if value < 0.0 or not math.isfinite(value):
                raise ValueError(f"{label} must be finite and non-negative")
        if self.surface_point_spacing <= 0.0 or not math.isfinite(
            self.surface_point_spacing
        ):
            raise ValueError("surface_point_spacing must be finite and positive")
        if self.enabled and not self.plane_config_file:
            raise ValueError("plane_config_file is required when filtering is enabled")

    def _log_throttled(self, key: str, level: str, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns.get(key, -10_000_000_000) < 5_000_000_000:
            return
        self._last_log_ns[key] = now_ns
        if level == "debug":
            self.get_logger().debug(message)
        elif level == "info":
            self.get_logger().info(message)
        elif level == "warning":
            self.get_logger().warning(message)
        elif level == "error":
            self.get_logger().error(message)
        else:
            raise ValueError(f"unsupported log level: {level}")

    def _odom_callback(self, message: Odometry) -> None:
        try:
            transform = _rigid_from_pose(message.pose.pose)
        except ValueError as error:
            self._log_throttled("bad_odom", "error", f"reject invalid odometry: {error}")
            return
        self.pose_buffer.append(
            PoseSample(
                stamp_ns=message_stamp_ns(message),
                parent_frame=_frame_id(message.header.frame_id),
                child_frame=_frame_id(message.child_frame_id),
                parent_from_child=transform,
            )
        )

    def _target_from_sensor(self, message: PointCloud2) -> Optional[RigidTransform]:
        assert self.configuration is not None
        target_frame = _frame_id(self.configuration.frame_id)
        sensor_frame = _frame_id(message.header.frame_id)
        if target_frame == sensor_frame:
            return RigidTransform.identity()

        if self.pose_source == "tf":
            try:
                stamped = self.tf_buffer.lookup_transform(
                    target_frame,
                    sensor_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self.transform_timeout_sec),
                )
                return _rigid_from_transform(stamped.transform)
            except (TransformException, ValueError) as error:
                self._log_throttled(
                    "missing_dynamic_tf",
                    "error",
                    f"drop cloud: cannot transform {sensor_frame} to {target_frame}: {error}",
                )
                return None

        sample = self.pose_buffer.closest(
            message_stamp_ns(message),
            int(self.maximum_odom_delta_sec * 1_000_000_000),
        )
        if sample is None:
            self._log_throttled(
                "missing_odom",
                "error",
                "drop cloud: no odometry sample within %.3f s of lidar stamp"
                % self.maximum_odom_delta_sec,
            )
            return None
        if sample.parent_frame != target_frame or sample.child_frame != self.base_frame:
            self._log_throttled(
                "odom_frames",
                "error",
                "drop cloud: odometry frames are %s -> %s; expected %s -> %s"
                % (
                    sample.parent_frame,
                    sample.child_frame,
                    target_frame,
                    self.base_frame,
                ),
            )
            return None
        try:
            base_from_sensor_message = self.tf_buffer.lookup_transform(
                self.base_frame,
                sensor_frame,
                Time(),
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
            base_from_sensor = _rigid_from_transform(
                base_from_sensor_message.transform
            )
        except (TransformException, ValueError) as error:
            self._log_throttled(
                "missing_static_tf",
                "error",
                f"drop cloud: cannot resolve static {self.base_frame} <- {sensor_frame}: {error}",
            )
            return None
        return base_from_sensor.then(sample.parent_from_child)

    def _cloud_callback(self, message: PointCloud2) -> None:
        self.frames_received += 1
        if not self.enabled:
            self.cloud_publisher.publish(message)
            self.frames_published += 1
            return

        transform = self._target_from_sensor(message)
        if transform is None:
            self.frames_dropped += 1
            return
        try:
            layout = PointCloudLayout.from_message(message)
            sensor_points = layout.read_xyz(message.data)
            target_points = transform.apply(sensor_points)
            sensor_origin = transform.translation
            result = filter_points_behind_planes(
                sensor_origin,
                target_points,
                self.configuration.planes,
                minimum_behind_distance=self.minimum_behind_distance,
                boundary_tolerance=self.boundary_tolerance,
            )
            if self.filtered_point_action == "glass_surface":
                replacement = sensor_points.copy()
                if np.any(result.mask):
                    replacement[result.mask] = transform.inverse().apply(
                        result.intersections[result.mask]
                    )
            else:
                replacement = sensor_points.copy()
                replacement[result.mask] = np.nan
            output_data = layout.replace_xyz(message.data, result.mask, replacement)
        except ValueError as error:
            self.frames_dropped += 1
            self._log_throttled(
                "invalid_cloud", "error", f"drop unsupported point cloud: {error}"
            )
            return

        output = PointCloud2()
        output.header = message.header
        output.height = message.height
        output.width = message.width
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = message.row_step
        output.data = output_data
        output.is_dense = message.is_dense and not (
            self.filtered_point_action == "nan" and np.any(result.mask)
        )
        self.cloud_publisher.publish(output)
        self.frames_published += 1
        replaced = int(np.count_nonzero(result.mask))
        self.points_replaced += replaced
        self._log_throttled(
            "stats",
            "info",
            "glass filter frames received/published/dropped=%d/%d/%d; "
            "last replaced=%d/%d, total replaced=%d"
            % (
                self.frames_received,
                self.frames_published,
                self.frames_dropped,
                replaced,
                layout.point_count,
                self.points_replaced,
            ),
        )

    def _publish_obstacle_reference(self) -> None:
        assert self.configuration is not None
        samples = [
            sample_plane_surface(plane, self.surface_point_spacing)
            for plane in self.configuration.planes
        ]
        points = np.concatenate(samples, axis=0) if samples else np.empty((0, 3))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.configuration.frame_id
        message = point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
        self.obstacle_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GlassPlaneFilter()
        rclpy.spin(node)
    except (ValueError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"glass_plane_filter: {error}")
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
