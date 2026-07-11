#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/record_go2_xt16_nav_debug_bag.sh [--duration SEC]

Record the Go2 XT16 navigation debug topics recommended for heading-alignment
diagnosis. This script only records ROS topics; it does not publish commands.

Environment:
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh
  GO2_NAV_DEBUG_BAG_DIR=/tmp/go2_xt16_nav_debug_bags
  GO2_NAV_DEBUG_BAG_PREFIX=go2_xt16_nav_debug

Topics:
  /dddmr_go2/dry_run_cmd_vel
  /dddmr_go2/safe_cmd_vel
  /dddmr_go2/p2p_decision
  /dddmr_go2/gait_unchanged
  /dddmr_go2/gait_monitor_reason
  /api/sport/request
  /lidar_points
  /ground_cloud
  /utlidar/robot_odom
  /dddmr_go2/robot_odom_standard
  /localization_status
  /lowstate
  /sportmodestate
  /mcl_pose
  /tf
  /tf_static
  /global_path
  /awared_global_path
  /map1/mapcloud
  /map1/mapsurface
  /map1/mapground
  /map1/terrain_ground
  /perception_3d_ros/dGraph
  /perception_3d_global/map/dGraph
  /perception_3d_global/lidar/global_marking
  /prune_plan
  /trajectory
  /accepted_trajectory
  /best_trajectory
  /segmented_cloud_pure
  /laser_cloud_less_sharp
  /laser_cloud_less_flat
  /utlidar/cloud_base
  /dddmr_terrain/traversability_cloud
  /dddmr_terrain/support_cloud
  /dddmr_terrain/unknown_cloud
  /dddmr_terrain/drop_cloud
  /dddmr_terrain/stair_markers
  /dddmr_terrain/status
  /dddmr_terrain/supervised_status
  /dddmr_terrain/traversal_state
  /dddmr_terrain/rejection_reason
  /dddmr_terrain/speed_limit
EOF
}

duration=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --duration requires seconds" >&2
        exit 2
      }
      duration="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
BAG_DIR="${GO2_NAV_DEBUG_BAG_DIR:-/tmp/go2_xt16_nav_debug_bags}"
BAG_PREFIX="${GO2_NAV_DEBUG_BAG_PREFIX:-go2_xt16_nav_debug}"
stamp="$(date +%Y%m%d_%H%M%S)"
output="${BAG_DIR}/${BAG_PREFIX}_${stamp}"

[[ -f "${GO2_SETUP}" ]] || {
  echo "ERROR: missing Go2 ROS setup: ${GO2_SETUP}" >&2
  exit 1
}
[[ -f "${WS_ROOT}/scripts/setup_go2_dds_env.sh" ]] || {
  echo "ERROR: missing DDS env script" >&2
  exit 1
}
mkdir -p "${BAG_DIR}"

unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
set +u
# shellcheck disable=SC1090
source "${GO2_SETUP}"
# shellcheck disable=SC1090
source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
set -u

topics=(
  /dddmr_go2/dry_run_cmd_vel
  /dddmr_go2/safe_cmd_vel
  /dddmr_go2/p2p_decision
  /dddmr_go2/gait_unchanged
  /dddmr_go2/gait_monitor_reason
  /api/sport/request
  /lidar_points
  /ground_cloud
  /utlidar/robot_odom
  /dddmr_go2/robot_odom_standard
  /localization_status
  /lowstate
  /sportmodestate
  /mcl_pose
  /tf
  /tf_static
  /global_path
  /awared_global_path
  /map1/mapcloud
  /map1/mapsurface
  /map1/mapground
  /map1/terrain_ground
  /perception_3d_ros/dGraph
  /perception_3d_global/map/dGraph
  /perception_3d_global/lidar/global_marking
  /prune_plan
  /trajectory
  /accepted_trajectory
  /best_trajectory
  /segmented_cloud_pure
  /laser_cloud_less_sharp
  /laser_cloud_less_flat
  /utlidar/cloud_base
  /dddmr_terrain/traversability_cloud
  /dddmr_terrain/support_cloud
  /dddmr_terrain/unknown_cloud
  /dddmr_terrain/drop_cloud
  /dddmr_terrain/stair_markers
  /dddmr_terrain/status
  /dddmr_terrain/supervised_status
  /dddmr_terrain/traversal_state
  /dddmr_terrain/rejection_reason
  /dddmr_terrain/speed_limit
)

echo "OUTPUT=${output}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"

cmd=(ros2 bag record -o "${output}" "${topics[@]}")
if [[ -n "${duration}" ]]; then
  timeout -s INT -k 5s "${duration}s" "${cmd[@]}"
else
  "${cmd[@]}"
fi
