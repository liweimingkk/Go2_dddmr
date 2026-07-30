from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("input_topic", default_value="/lidar_points"),
        DeclareLaunchArgument(
            "output_topic", default_value="/lidar_points_glass_filtered"
        ),
        DeclareLaunchArgument(
            "obstacle_topic", default_value="/glass_plane_obstacles"
        ),
        DeclareLaunchArgument("plane_config_file", default_value=""),
        DeclareLaunchArgument("pose_source", default_value="odometry"),
        DeclareLaunchArgument(
            "odometry_topic", default_value="/dddmr_go2/robot_odom_standard"
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "filtered_point_action", default_value="glass_surface"
        ),
        DeclareLaunchArgument("minimum_behind_distance", default_value="0.15"),
        DeclareLaunchArgument("boundary_tolerance", default_value="0.05"),
        DeclareLaunchArgument("maximum_odom_delta_sec", default_value="0.05"),
        DeclareLaunchArgument("transform_timeout_sec", default_value="0.10"),
        DeclareLaunchArgument("surface_point_spacing", default_value="0.10"),
    ]
    filter_node = Node(
        package="dddmr_glass_filter",
        executable="glass_plane_filter",
        name="glass_plane_filter",
        output="screen",
        parameters=[
            {
                "enabled": LaunchConfiguration("enabled"),
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "obstacle_topic": LaunchConfiguration("obstacle_topic"),
                "plane_config_file": LaunchConfiguration("plane_config_file"),
                "pose_source": LaunchConfiguration("pose_source"),
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "filtered_point_action": LaunchConfiguration(
                    "filtered_point_action"
                ),
                "minimum_behind_distance": LaunchConfiguration(
                    "minimum_behind_distance"
                ),
                "boundary_tolerance": LaunchConfiguration("boundary_tolerance"),
                "maximum_odom_delta_sec": LaunchConfiguration(
                    "maximum_odom_delta_sec"
                ),
                "transform_timeout_sec": LaunchConfiguration(
                    "transform_timeout_sec"
                ),
                "surface_point_spacing": LaunchConfiguration(
                    "surface_point_spacing"
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [filter_node])
