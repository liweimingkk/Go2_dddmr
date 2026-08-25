"""Launch the DDDMR browser viewer and its guarded ROS bridge."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the standalone web-viewer launch description."""
    bind_address = LaunchConfiguration("bind_address")
    http_port = LaunchConfiguration("http_port")
    websocket_port = LaunchConfiguration("websocket_port")
    launch_rosbridge = LaunchConfiguration("launch_rosbridge")
    rosbridge_parameters = {
        "address": bind_address,
        "port": ParameterValue(websocket_port, value_type=int),
    }
    if os.environ.get("ROS_DISTRO") == "humble":
        rosbridge_parameters.update(
            {
                "topics_pub_glob": "['/dddmr_web/*']",
                "topics_sub_glob": (
                    "['/dddmr_web/*','/tf','/tf_static','/global_path','/prune_plan']"
                ),
                "services_glob": "[]",
                "actions_glob": "[]",
                "default_call_service_timeout": 5.0,
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
            }
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bind_address",
                default_value="127.0.0.1",
                description=(
                    "HTTP and WebSocket bind address. Keep localhost unless access "
                    "is protected by an SSH tunnel or trusted network boundary."
                ),
            ),
            DeclareLaunchArgument("http_port", default_value="8080"),
            DeclareLaunchArgument("websocket_port", default_value="9090"),
            DeclareLaunchArgument("launch_rosbridge", default_value="true"),
            DeclareLaunchArgument(
                "allow_navigation_execution",
                default_value="false",
                description=(
                    "Allow an explicitly confirmed preview to start /p2p_move_base."
                ),
            ),
            DeclareLaunchArgument("map_topic", default_value="/map1/mapcloud"),
            DeclareLaunchArgument("ground_topic", default_value="/map1/mapground"),
            DeclareLaunchArgument("preview_action_name", default_value="/get_plan"),
            DeclareLaunchArgument(
                "navigation_action_name", default_value="/p2p_move_base"
            ),
            DeclareLaunchArgument(
                "initial_pose_topic", default_value="/initial_3d_pose"
            ),
            DeclareLaunchArgument("preview_max_age_sec", default_value="30.0"),
            Node(
                package="dddmr_web_viewer",
                executable="web_navigation_bridge",
                name="dddmr_web_navigation_bridge",
                output="screen",
                parameters=[
                    {
                        "map_topic": LaunchConfiguration("map_topic"),
                        "ground_topic": LaunchConfiguration("ground_topic"),
                        "preview_action_name": LaunchConfiguration(
                            "preview_action_name"
                        ),
                        "navigation_action_name": LaunchConfiguration(
                            "navigation_action_name"
                        ),
                        "initial_pose_topic": LaunchConfiguration(
                            "initial_pose_topic"
                        ),
                        "allow_navigation_execution": ParameterValue(
                            LaunchConfiguration("allow_navigation_execution"),
                            value_type=bool,
                        ),
                        "preview_max_age_sec": ParameterValue(
                            LaunchConfiguration("preview_max_age_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                condition=IfCondition(launch_rosbridge),
                parameters=[rosbridge_parameters],
            ),
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="python3"),
                    "-m",
                    "dddmr_web_viewer.web_server",
                    "--host",
                    bind_address,
                    "--port",
                    http_port,
                ],
                output="screen",
            ),
        ]
    )
