#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/capture_go2_xt16_glass_calibration.sh \
    --label normal|angle30|angle60 [options]

Read-only capture for known-glass-plane calibration. This script records ROS
topics only. It does not launch mapping/navigation and never publishes a motion
command.

Options:
  --label NAME          Required capture label (letters, digits, dot, dash, underscore)
  --duration SEC        Bounded recording duration (default: 30)
  --return-mode TEXT    Operator-observed XT16 return mode (default: unknown)
  --expect-width N      Expected PointCloud2 width (default: 32000)
  --expect-per-ring N   Expected points per ring (default: 2000)
  --output-root DIR     Bag parent (default: repository bags directory)
  --dry-run             Validate discovery and print the bag command only
  -h, --help            Show this help

Environment:
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh

Required topics:
  /lidar_points
  /utlidar/robot_odom

Recorded when present:
  /lidar_packets
  /utlidar/cloud_base
  /tf
  /tf_static
EOF
}

label=""
duration="30"
return_mode="unknown"
expect_width="32000"
expect_per_ring="2000"
output_root=""
dry_run=false

while (( $# > 0 )); do
  case "$1" in
    --label)
      (( $# >= 2 )) || { echo "ERROR: --label requires a value" >&2; exit 2; }
      label="$2"
      shift 2
      ;;
    --duration)
      (( $# >= 2 )) || { echo "ERROR: --duration requires seconds" >&2; exit 2; }
      duration="$2"
      shift 2
      ;;
    --return-mode)
      (( $# >= 2 )) || { echo "ERROR: --return-mode requires text" >&2; exit 2; }
      return_mode="$2"
      shift 2
      ;;
    --expect-width)
      (( $# >= 2 )) || { echo "ERROR: --expect-width requires a value" >&2; exit 2; }
      expect_width="$2"
      shift 2
      ;;
    --expect-per-ring)
      (( $# >= 2 )) || { echo "ERROR: --expect-per-ring requires a value" >&2; exit 2; }
      expect_per_ring="$2"
      shift 2
      ;;
    --output-root)
      (( $# >= 2 )) || { echo "ERROR: --output-root requires a directory" >&2; exit 2; }
      output_root="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "${label}" ]] || { echo "ERROR: --label is required" >&2; exit 2; }
[[ "${label}" =~ ^[A-Za-z0-9._-]+$ ]] || {
  echo "ERROR: --label contains unsupported characters: ${label}" >&2
  exit 2
}
[[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "ERROR: --duration must be a positive number" >&2
  exit 2
}
awk -v value="${duration}" 'BEGIN { exit !(value > 0 && value <= 120) }' || {
  echo "ERROR: --duration must be in (0, 120] seconds" >&2
  exit 2
}
[[ "${return_mode}" != *$'\n'* ]] || {
  echo "ERROR: --return-mode must be a single line" >&2
  exit 2
}
[[ "${expect_width}" =~ ^[1-9][0-9]*$ ]] || {
  echo "ERROR: --expect-width must be a positive integer" >&2
  exit 2
}
[[ "${expect_per_ring}" =~ ^[1-9][0-9]*$ ]] || {
  echo "ERROR: --expect-per-ring must be a positive integer" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
output_root="${output_root:-${REPO_ROOT}/bags}"

[[ -f "${GO2_SETUP}" ]] || {
  echo "ERROR: missing Go2 ROS setup: ${GO2_SETUP}" >&2
  exit 1
}
[[ -f "${WS_ROOT}/scripts/setup_go2_dds_env.sh" ]] || {
  echo "ERROR: missing DDS setup helper" >&2
  exit 1
}

unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
set +u
# shellcheck disable=SC1090
source "${GO2_SETUP}"
# shellcheck disable=SC1090
source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
set -u

echo "MODE=READ_ONLY_GLASS_CALIBRATION_CAPTURE"
echo "No mapping, navigation, /cmd_vel, /api/sport/request, or /lowcmd publisher is started."

lidar_type="$(timeout 10s ros2 topic type /lidar_points 2>/dev/null || true)"
odom_type="$(timeout 10s ros2 topic type /utlidar/robot_odom 2>/dev/null || true)"
[[ "${lidar_type}" == "sensor_msgs/msg/PointCloud2" ]] || {
  echo "ERROR: /lidar_points type is '${lidar_type:-missing}', expected sensor_msgs/msg/PointCloud2" >&2
  exit 1
}
[[ "${odom_type}" == "nav_msgs/msg/Odometry" ]] || {
  echo "ERROR: /utlidar/robot_odom type is '${odom_type:-missing}', expected nav_msgs/msg/Odometry" >&2
  exit 1
}

python3 "${WS_ROOT}/scripts/go2_xt16_lidar_preflight.py" \
  --topic /lidar_points --samples 3 --timeout 10 \
  --expect-width "${expect_width}" --expect-points-per-ring "${expect_per_ring}"

mapfile -t discovered_topics < <(ros2 topic list 2>/dev/null)
has_topic() {
  local candidate="$1"
  local discovered
  for discovered in "${discovered_topics[@]}"; do
    [[ "${discovered}" == "${candidate}" ]] && return 0
  done
  return 1
}

topics=(/lidar_points /utlidar/robot_odom)
for optional_topic in /lidar_packets /utlidar/cloud_base /tf /tf_static; do
  if has_topic "${optional_topic}"; then
    topics+=("${optional_topic}")
  else
    echo "OPTIONAL_TOPIC_MISSING=${optional_topic}"
  fi
done

stamp="$(date +%Y%m%d_%H%M%S)"
mkdir -p -- "${output_root}"
output="${output_root}/go2_xt16_glass_${label}_${stamp}"
manifest="${output}_capture_manifest.txt"
cmd=(ros2 bag record -o "${output}" "${topics[@]}")

echo "LABEL=${label}"
echo "DURATION_SEC=${duration}"
echo "RETURN_MODE=${return_mode}"
echo "EXPECTED_WIDTH=${expect_width}"
echo "EXPECTED_POINTS_PER_RING=${expect_per_ring}"
echo "OUTPUT=${output}"
echo "TOPICS=${topics[*]}"
printf 'COMMAND='
printf '%q ' "${cmd[@]}"
printf '\n'

if [[ "${dry_run}" == "true" ]]; then
  echo "RESULT=DRY_RUN_PASS"
  exit 0
fi

{
  echo "capture_kind=known_glass_plane_calibration"
  echo "capture_label=${label}"
  echo "capture_started=$(date --iso-8601=seconds)"
  echo "duration_sec=${duration}"
  echo "xt16_return_mode=${return_mode}"
  echo "expected_width=${expect_width}"
  echo "expected_points_per_ring=${expect_per_ring}"
  echo "lidar_topic_type=${lidar_type}"
  echo "odom_topic_type=${odom_type}"
  echo "rmw_implementation=${RMW_IMPLEMENTATION:-}"
  echo "cyclonedds_uri=${CYCLONEDDS_URI:-}"
  printf 'topics='
  printf '%s ' "${topics[@]}"
  printf '\n'
} >"${manifest}"

set +e
timeout -s INT -k 10s "${duration}s" "${cmd[@]}"
record_status=$?
set -e

if [[ ! -f "${output}/metadata.yaml" ]]; then
  echo "ERROR: recording produced no metadata: ${output}" >&2
  exit 1
fi
if (( record_status != 0 && record_status != 124 && record_status != 130 )); then
  echo "ERROR: ros2 bag record exited with status ${record_status}" >&2
  exit "${record_status}"
fi

echo "capture_finished=$(date --iso-8601=seconds)" >>"${manifest}"
echo "record_status=${record_status}" >>"${manifest}"
echo "MANIFEST=${manifest}"
echo "RESULT=PASS"
