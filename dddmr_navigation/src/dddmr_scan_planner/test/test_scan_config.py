#!/usr/bin/env python3

import math
import pathlib
import sys
import xml.etree.ElementTree as ET

import yaml


def require_mapping(value, path):
    if not isinstance(value, dict):
        raise AssertionError(f"{path} must be a mapping")
    return value


def require_finite(params, name):
    value = params.get(name)
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
    ):
        raise AssertionError(f"{name} must be finite, got {value!r}")
    return float(value)


def child_attributes(element, tag, key):
    return {
        child.attrib[key]: child.attrib
        for child in element.findall(tag)
        if key in child.attrib
    }


def child_elements(element, tag, key):
    return {
        child.attrib[key]: child
        for child in element.findall(tag)
        if key in child.attrib
    }


def check_launch(launch_path):
    root = ET.parse(launch_path).getroot()
    launch_args = child_attributes(root, "arg", "name")
    assert launch_args["scan_max_vel_y"]["default"] == "0.0"
    assert launch_args["scan_max_plan_vel"]["default"] == "0.50"
    assert launch_args["start_sport_dry_run_adapter"]["default"] == "true"

    includes = root.findall("include")
    assert len(includes) == 1
    include_args = child_attributes(includes[0], "arg", "name")
    assert include_args["start_move_base"]["value"] == "false"
    assert include_args["start_global_planner"]["value"] == "true"
    assert include_args["start_go2_nav_cmd_gate"]["value"] == "true"
    assert include_args["start_go2_sport_adapter"]["value"] == "false"
    assert include_args["go2_sport_enable_output"]["value"] == "false"
    assert include_args["go2_sport_allow_real_request_topic"]["value"] == "false"

    nodes = child_elements(root, "node", "name")
    guard_remaps = child_attributes(nodes["scan_command_guard"], "remap", "from")
    assert guard_remaps["guarded_cmd"]["to"] == "/dddmr_go2/dry_run_cmd_vel"
    assert (
        guard_remaps["planner_heartbeat"]["to"]
        == "/scan_planner/planning/data_display"
    )
    controller_remaps = child_attributes(
        nodes["closed_loop_controller"], "remap", "from"
    )
    assert controller_remaps["cmd_vel"]["to"] == "/scan_planner/raw_cmd_vel"


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: test_scan_config.py CONFIG.yaml LAUNCH.xml")
    config_path = pathlib.Path(sys.argv[1])
    launch_path = pathlib.Path(sys.argv[2])
    config = require_mapping(
        yaml.safe_load(config_path.read_text(encoding="utf-8")), "config"
    )

    planner = require_mapping(config["scan_planner_node"], "scan_planner_node")
    planner_params = require_mapping(
        planner["ros__parameters"], "scan_planner_node.ros__parameters"
    )
    controller = require_mapping(
        config["closed_loop_controller"]["ros__parameters"],
        "closed_loop_controller.ros__parameters",
    )
    guard = require_mapping(
        config["scan_command_guard"]["ros__parameters"],
        "scan_command_guard.ros__parameters",
    )
    adapter = require_mapping(
        config["scan_input_adapter"]["ros__parameters"],
        "scan_input_adapter.ros__parameters",
    )
    route = require_mapping(
        config["scan_route_bridge"]["ros__parameters"],
        "scan_route_bridge.ros__parameters",
    )

    assert planner_params["fsm.navi_mode"] == 3
    assert planner_params["grid_map.frame_id"] == "map"
    assert planner_params["grid_map.cloud_is_world"] is True
    assert planner_params["grid_map.need_extrinsic"] is False
    assert adapter["prefer_latest_transform"] is True
    assert require_finite(adapter, "transform_max_age_sec") <= 0.25
    assert require_finite(adapter, "transform_max_future_skew_sec") <= 0.05
    assert require_finite(route, "body_pose_timeout") <= 0.25
    assert require_finite(route, "start_exclusion_xy") >= 0.10
    assert require_finite(route, "min_path_point_separation") > 0.0
    assert require_finite(planner_params, "grid_map.double_cylinder_radius") >= 0.26
    assert math.isclose(
        require_finite(planner_params, "grid_map.body_height"),
        0.32,
        abs_tol=1e-9,
    )
    assert math.isclose(
        require_finite(planner_params, "manager.max_vel"),
        require_finite(planner_params, "optimization.max_vel"),
        abs_tol=1e-9,
    )
    assert math.isclose(
        require_finite(controller, "max_vx"),
        require_finite(guard, "max_x"),
        abs_tol=1e-9,
    )
    assert math.isclose(
        require_finite(controller, "max_vy"),
        require_finite(guard, "max_y"),
        abs_tol=1e-9,
    )
    assert math.isclose(
        require_finite(controller, "max_vyaw"),
        require_finite(guard, "max_yaw"),
        abs_tol=1e-9,
    )
    assert require_finite(guard, "max_y") <= 0.20
    assert require_finite(guard, "max_yaw") <= 0.50
    assert require_finite(guard, "cloud_timeout") <= 0.35
    assert require_finite(guard, "odom_timeout") <= 0.25
    assert require_finite(guard, "raw_command_timeout") <= 0.15
    assert require_finite(guard, "planner_timeout") <= 0.50
    check_launch(launch_path)


if __name__ == "__main__":
    main()
