#!/usr/bin/env python3

import pathlib
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LAUNCH_DIRECTORY = pathlib.Path(__file__).resolve().parent
if str(LAUNCH_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(LAUNCH_DIRECTORY))

from go2_xt16_p2p_runtime_parameters import (  # noqa: E402
    build_exact_runtime_parameters,
    write_exact_runtime_parameters,
)


def _launch_p2p_move_base(context):
    freshness = LaunchConfiguration(
        "local_lidar_expected_sensor_time_sec"
    ).perform(context)
    minimum_y = LaunchConfiguration("omni_min_vel_y").perform(context)
    maximum_y = LaunchConfiguration("omni_max_vel_y").perform(context)
    goals_enabled = LaunchConfiguration("p2p_goals_enabled").perform(context)

    exact_parameters = build_exact_runtime_parameters(
        freshness,
        minimum_y,
        maximum_y,
        goals_enabled,
    )
    runtime_parameter_file = write_exact_runtime_parameters(
        freshness,
        minimum_y,
        maximum_y,
        goals_enabled,
    )
    local_parameters = exact_parameters["/perception_3d_local"][
        "ros__parameters"
    ]
    planner_parameters = exact_parameters["/trajectory_generators"][
        "ros__parameters"
    ]
    get_logger("go2_xt16_p2p_move_base").info(
        "Generated exact Foxy P2P overrides: "
        "/perception_3d_local lidar.expected_sensor_time=%.6f, "
        "/trajectory_generators omni_drive_simple.min_vel_y=%.6f "
        "omni_drive_simple.max_vel_y=%.6f"
        % (
            local_parameters["lidar.expected_sensor_time"],
            planner_parameters["omni_drive_simple.min_vel_y"],
            planner_parameters["omni_drive_simple.max_vel_y"],
        )
    )

    return [
        Node(
            package="p2p_move_base",
            executable="p2p_move_base_node",
            output="screen",
            respawn=False,
            parameters=[
                LaunchConfiguration("config_file"),
                runtime_parameter_file,
                {"use_sim_time": False},
            ],
            remappings=[
                ("/cmd_vel", "/dddmr_go2/dry_run_cmd_vel"),
                (
                    "/cmd_vel_stamped",
                    "/dddmr_go2/dry_run_cmd_vel_stamped",
                ),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file"),
            DeclareLaunchArgument(
                "local_lidar_expected_sensor_time_sec",
                default_value="0.20",
            ),
            DeclareLaunchArgument("omni_min_vel_y", default_value="0.0"),
            DeclareLaunchArgument("omni_max_vel_y", default_value="0.0"),
            DeclareLaunchArgument("p2p_goals_enabled", default_value="true"),
            OpaqueFunction(function=_launch_p2p_move_base),
        ]
    )
