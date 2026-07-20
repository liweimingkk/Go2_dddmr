#!/usr/bin/env python3
"""Run an isolated, no-motion wall-rejection integration check."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

try:
    from dddmr_sys_core.action import GetPlan
except ModuleNotFoundError as exc:
    raise SystemExit(
        "dddmr_sys_core is unavailable; source the build/install setup before running this check"
    ) from exc


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"
)


def make_cloud(
    node: Node,
    points: list[tuple[float, float, float, float]],
    frame_id: str = "map",
) -> PointCloud2:
    header = Header()
    header.frame_id = frame_id
    header.stamp = node.get_clock().now().to_msg()
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, points)


def frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    value = start
    while value <= stop + step * 0.5:
        values.append(round(value, 6))
        value += step
    return values


def synthetic_map(
    node: Node, wall_gap_half_width: float = 0.0
) -> tuple[PointCloud2, PointCloud2]:
    ground = [
        (x, y, 0.0, 0.0)
        # A 0.4 m grid yields fewer than eight neighbors inside the planner's
        # 0.5 m radius and exercises the expanded-radius A* branch.
        for x in frange(-2.0, 2.0, 0.4)
        for y in frange(-1.2, 1.2, 0.4)
    ]
    wall = [
        (0.0, y, z, 1.0)
        for y in frange(-2.0, 2.0, 0.1)
        for z in frange(0.2, 0.8, 0.1)
        if wall_gap_half_width <= 0.0 or abs(y) >= wall_gap_half_width
    ]
    return make_cloud(node, wall), make_cloud(node, ground)


def send_static_pose(
    node: Node, broadcaster: StaticTransformBroadcaster, x: float, y: float
) -> None:
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)


def request_plan(
    node: Node,
    client: ActionClient,
    goal_x: float,
    goal_y: float,
    timeout: float,
) -> tuple[int, int]:
    request = GetPlan.Goal()
    request.goal.header.frame_id = "map"
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x = goal_x
    request.goal.pose.position.y = goal_y
    request.goal.pose.orientation.w = 1.0
    request.activate_threading = True

    send_future = client.send_goal_async(request)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout)
    if not send_future.done() or send_future.result() is None:
        raise RuntimeError("timed out sending /get_plan goal")
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        raise RuntimeError("/get_plan rejected the test goal")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    if not result_future.done() or result_future.result() is None:
        raise RuntimeError("timed out waiting for /get_plan result")
    wrapped = result_future.result()
    return wrapped.status, len(wrapped.result.path.poses)


def stop_process(process: subprocess.Popen[str]) -> str:
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)
    if process.stdout is None:
        return ""
    process.stdout.seek(0)
    return process.stdout.read()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="No-motion synthetic test; uses an isolated ROS domain and publishes no velocity."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--domain-id", type=int, default=231)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--wall-gap-half-width",
        type=float,
        default=0.0,
        help="Create a centered doorway and require the cross-wall plan to pass.",
    )
    args = parser.parse_args()

    ros2 = shutil.which("ros2")
    if ros2 is None:
        print("SYNTHETIC_WALL_REJECTION_STATUS=FAIL: ros2 executable not found")
        return 1
    if not args.config.is_file():
        print(f"SYNTHETIC_WALL_REJECTION_STATUS=FAIL: missing config {args.config}")
        return 1

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.pop("CYCLONEDDS_URI", None)
    ros_log_dir = Path(tempfile.gettempdir()) / "go2_xt16_synthetic_wall_ros_logs"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    child_env = os.environ.copy()
    child_env["RCUTILS_LOGGING_SEVERITY"] = "WARN"

    output = tempfile.TemporaryFile(mode="w+", encoding="utf-8")
    process = subprocess.Popen(
        [
            ros2,
            "run",
            "global_planner",
            "global_planner_node",
            "--ros-args",
            "--params-file",
            str(args.config),
            "-p",
            "map.traversed_stair_clearance.enabled:=false",
        ],
        env=child_env,
        stdout=output,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    node = Node("go2_xt16_synthetic_wall_rejection")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    map_pub = node.create_publisher(PointCloud2, "map1/mapcloud", qos)
    ground_pub = node.create_publisher(PointCloud2, "map1/planning_ground", qos)
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    sensor_pub = node.create_publisher(PointCloud2, "segmented_cloud_pure", sensor_qos)
    client = ActionClient(node, GetPlan, "/get_plan")
    broadcaster = StaticTransformBroadcaster(node)

    status = 1
    try:
        send_static_pose(node, broadcaster, -1.2, 0.0)
        wall, ground = synthetic_map(node, args.wall_gap_half_width)
        for _ in range(5):
            map_pub.publish(wall)
            ground_pub.publish(ground)
            sensor_pub.publish(make_cloud(node, [(10.0, 10.0, 0.0, 0.0)], "base_link"))
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.1)

        for _ in range(5):
            sensor_pub.publish(make_cloud(node, [(10.0, 10.0, 0.0, 0.0)], "base_link"))
            rclpy.spin_once(node, timeout_sec=0.2)

        if not client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError("/get_plan action server did not start")

        # Keep the control goal on the synthetic 0.4 m ground grid.  Endpoint
        # projection is deliberately limited to 0.10 m in the live config, so
        # a midpoint goal would test endpoint rejection instead of wall safety.
        same_status, same_poses = request_plan(node, client, -1.2, 0.8, args.timeout)
        if same_status != GoalStatus.STATUS_SUCCEEDED or same_poses == 0:
            raise RuntimeError(
                f"same-side control plan failed: status={same_status} poses={same_poses}"
            )

        cross_status, cross_poses = request_plan(node, client, 1.2, 0.0, args.timeout)
        if args.wall_gap_half_width > 0.0:
            if cross_status != GoalStatus.STATUS_SUCCEEDED or cross_poses == 0:
                raise RuntimeError(
                    "doorway plan was rejected: "
                    f"status={cross_status} poses={cross_poses}"
                )
        elif cross_poses != 0 or cross_status == GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"cross-wall plan was admitted: status={cross_status} poses={cross_poses}"
            )

        print(f"same_side_status={same_status} same_side_poses={same_poses}")
        print(f"cross_wall_status={cross_status} cross_wall_poses={cross_poses}")
        if args.wall_gap_half_width > 0.0:
            print(
                "SYNTHETIC_DOORWAY_PASSAGE_STATUS=PASS "
                f"gap_width={2.0 * args.wall_gap_half_width:.2f}"
            )
        else:
            print("SYNTHETIC_WALL_REJECTION_STATUS=PASS")
        status = 0
    except Exception as exc:  # The node log is printed below for diagnosis.
        print(f"SYNTHETIC_WALL_REJECTION_STATUS=FAIL: {exc}")
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        process.stdout = output
        node_log = stop_process(process)
        if status != 0 and node_log:
            print("GLOBAL_PLANNER_LOG_BEGIN")
            print(node_log[-12000:])
            print("GLOBAL_PLANNER_LOG_END")
        output.close()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
