#!/usr/bin/env python3
"""Interactively record a fixed initial pose and SCAN mission waypoints."""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from scan_mission_io import (
    IDENTIFIER_PATTERN,
    MAX_DWELL_SEC,
    MissionValidationError,
    Waypoint,
    atomic_write_json,
    initial_pose_document,
    load_mission,
    mission_document,
    relative_initial_pose_reference,
    yaw_from_quaternion,
)


STATUS_FRESHNESS_SEC = 0.75
BODY_HEIGHT = 0.32


class ScanWaypointRecorder(Node):
    def __init__(self, mission_path: str, initial_pose_path: str) -> None:
        super().__init__("scan_waypoint_recorder")
        self.mission_path = Path(mission_path).expanduser().resolve()
        self.initial_pose_path = Path(initial_pose_path).expanduser().resolve()
        self.mission_id = self.mission_path.stem
        if not IDENTIFIER_PATTERN.fullmatch(self.mission_id):
            raise MissionValidationError(
                "mission filename stem must be a valid mission identifier"
            )
        self.waypoints: List[Waypoint] = []
        self.latest_body: Optional[Odometry] = None
        self.latest_mcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.localization_status: Optional[str] = None
        self.localization_health: Optional[str] = None
        self.body_receipt = 0.0
        self.mcl_receipt = 0.0
        self.status_receipt = 0.0
        self.health_receipt = 0.0

        if self.mission_path.exists():
            mission = load_mission(self.mission_path)
            if mission.initial_pose_path != self.initial_pose_path:
                raise MissionValidationError(
                    "existing mission references a different initial-pose file: "
                    f"{mission.initial_pose_path}"
                )
            self.mission_id = mission.mission_id
            self.waypoints = list(mission.waypoints)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Odometry,
            "/scan_planner/body_pose",
            self.body_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/mcl_pose", self.mcl_callback, 10
        )
        self.create_subscription(
            String, "/localization_status", self.status_callback, transient_qos
        )
        self.create_subscription(
            String, "/localization_health", self.health_callback, transient_qos
        )

    def body_callback(self, message: Odometry) -> None:
        if message.header.frame_id == "map":
            self.latest_body = message
            self.body_receipt = time.monotonic()

    def mcl_callback(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id == "map":
            self.latest_mcl_pose = message
            self.mcl_receipt = time.monotonic()

    def status_callback(self, message: String) -> None:
        self.localization_status = message.data
        self.status_receipt = time.monotonic()

    def health_callback(self, message: String) -> None:
        self.localization_health = message.data
        self.health_receipt = time.monotonic()

    def localization_ready(self, require_mcl: bool = False) -> bool:
        now = time.monotonic()
        times = [self.body_receipt, self.status_receipt, self.health_receipt]
        if require_mcl:
            times.append(self.mcl_receipt)
        return (
            self.localization_status == "TRACKING"
            and self.localization_health == "HEALTHY"
            and all(receipt > 0.0 and now - receipt <= STATUS_FRESHNESS_SEC for receipt in times)
        )

    def record_initial_pose(self, settings) -> None:
        if not self.localization_ready(require_mcl=True) or self.latest_mcl_pose is None:
            self.get_logger().warning(
                "Initial pose not saved: require fresh TRACKING, HEALTHY, body pose and mcl_pose"
            )
            return
        if self.initial_pose_path.exists():
            answer = self.prompt_text(
                settings,
                f"Overwrite fixed initial pose {self.initial_pose_path}? Type OVERWRITE: ",
            )
            if answer != "OVERWRITE":
                self.get_logger().warning("Initial-pose overwrite cancelled")
                return
        pose = self.latest_mcl_pose.pose
        document = initial_pose_document(
            "map",
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ),
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ),
            pose.covariance,
        )
        atomic_write_json(self.initial_pose_path, document)
        self.get_logger().info(f"Saved fixed initial pose: {self.initial_pose_path}")

    def current_waypoint(self, dwell_sec: float, waypoint_id: str) -> Optional[Waypoint]:
        if not self.localization_ready() or self.latest_body is None:
            self.get_logger().warning(
                "Waypoint not recorded: require fresh TRACKING, HEALTHY and map body pose"
            )
            return None
        pose = self.latest_body.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning("Waypoint not recorded: body pose is non-finite")
            return None
        # /goal_pose_3d uses a ground point. SCAN raises the returned reference
        # path by grid_map.body_height before tracking it.
        ground_z = float(pose.position.z) - BODY_HEIGHT
        return Waypoint(
            waypoint_id=waypoint_id,
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=ground_z,
            yaw=yaw_from_quaternion(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            ),
            dwell_sec=dwell_sec,
        )

    def prompt_dwell(self, settings) -> Optional[float]:
        raw = self.prompt_text(
            settings, f"dwell_sec [0..{MAX_DWELL_SEC:g}] (required): "
        )
        try:
            dwell = float(raw)
        except ValueError:
            self.get_logger().warning("Waypoint unchanged: dwell_sec is not numeric")
            return None
        if not math.isfinite(dwell) or dwell < 0.0 or dwell > MAX_DWELL_SEC:
            self.get_logger().warning(
                f"Waypoint unchanged: dwell_sec must be within [0, {MAX_DWELL_SEC:g}]"
            )
            return None
        return dwell

    def add_waypoint(self, settings) -> None:
        dwell = self.prompt_dwell(settings)
        if dwell is None:
            return
        existing = {waypoint.waypoint_id for waypoint in self.waypoints}
        sequence = 1
        while f"wp_{sequence:03d}" in existing:
            sequence += 1
        waypoint = self.current_waypoint(dwell, f"wp_{sequence:03d}")
        if waypoint is not None:
            self.waypoints.append(waypoint)
            self.get_logger().info(
                "Recorded %s: x=%.3f y=%.3f ground_z=%.3f yaw=%.3f dwell=%.1fs"
                % (
                    waypoint.waypoint_id,
                    waypoint.x,
                    waypoint.y,
                    waypoint.z,
                    waypoint.yaw,
                    waypoint.dwell_sec,
                )
            )

    def replace_waypoint(self, settings) -> None:
        index = self.prompt_index(settings, "Replace")
        if index is None:
            return
        dwell = self.prompt_dwell(settings)
        if dwell is None:
            return
        current = self.waypoints[index]
        replacement = self.current_waypoint(dwell, current.waypoint_id)
        if replacement is not None:
            self.waypoints[index] = replacement
            self.get_logger().info(f"Replaced {current.waypoint_id}")

    def delete_waypoint(self, settings) -> None:
        index = self.prompt_index(settings, "Delete")
        if index is not None:
            removed = self.waypoints.pop(index)
            self.get_logger().info(f"Deleted {removed.waypoint_id}")

    def list_waypoints(self) -> None:
        if not self.waypoints:
            print("(no waypoints)")
            return
        for index, waypoint in enumerate(self.waypoints, 1):
            print(
                "%d: %s x=%.3f y=%.3f ground_z=%.3f yaw=%.3f dwell=%.1fs"
                % (
                    index,
                    waypoint.waypoint_id,
                    waypoint.x,
                    waypoint.y,
                    waypoint.z,
                    waypoint.yaw,
                    waypoint.dwell_sec,
                )
            )

    def save(self) -> bool:
        if not self.initial_pose_path.is_file():
            self.get_logger().error(
                f"Cannot save mission before fixed initial pose exists: {self.initial_pose_path}"
            )
            return False
        if not self.waypoints:
            self.get_logger().error("Cannot save an empty mission")
            return False
        reference = relative_initial_pose_reference(
            self.mission_path, self.initial_pose_path
        )
        document = mission_document(self.mission_id, reference, self.waypoints)
        atomic_write_json(self.mission_path, document)
        load_mission(self.mission_path)
        self.get_logger().info(
            f"Saved mission {self.mission_id} with {len(self.waypoints)} waypoint(s): "
            f"{self.mission_path}"
        )
        return True

    def prompt_index(self, settings, action: str) -> Optional[int]:
        if not self.waypoints:
            self.get_logger().warning("No waypoint is available")
            return None
        raw = self.prompt_text(
            settings, f"{action} waypoint index (1-{len(self.waypoints)}): "
        )
        try:
            index = int(raw) - 1
        except ValueError:
            index = -1
        if index < 0 or index >= len(self.waypoints):
            self.get_logger().warning("Invalid waypoint index")
            return None
        return index

    @staticmethod
    def prompt_text(settings, prompt: str) -> str:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            return input(prompt).strip()
        finally:
            tty.setcbreak(sys.stdin.fileno())

    def run(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("waypoint recording requires an interactive TTY")
        print(
            "i: save fixed initial pose | Enter/Space/a: add waypoint | "
            "r: replace | d: delete | u: undo | l: list | s: save | q: save+quit"
        )
        print(
            "Set a correct RViz 3D Pose Estimate first. Each waypoint records the "
            "current localized base pose and requires dwell_sec."
        )
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    continue
                key = sys.stdin.read(1)
                lowered = key.lower()
                if lowered == "i":
                    self.record_initial_pose(settings)
                elif key in ("\n", "\r", " ") or lowered == "a":
                    self.add_waypoint(settings)
                elif lowered == "r":
                    self.replace_waypoint(settings)
                elif lowered == "d":
                    self.delete_waypoint(settings)
                elif lowered == "u" and self.waypoints:
                    removed = self.waypoints.pop()
                    self.get_logger().info(f"Undid {removed.waypoint_id}")
                elif lowered == "l":
                    self.list_waypoints()
                elif lowered == "s":
                    self.save()
                elif lowered in ("q", "\x03"):
                    if self.save():
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission-file", required=True)
    parser.add_argument("--initial-pose-file", required=True)
    arguments = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    result = 0
    try:
        recorder = ScanWaypointRecorder(
            arguments.mission_file, arguments.initial_pose_file
        )
        recorder.run()
    except (KeyboardInterrupt, RuntimeError, MissionValidationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        result = 1
    finally:
        if "recorder" in locals():
            recorder.destroy_node()
        rclpy.try_shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
