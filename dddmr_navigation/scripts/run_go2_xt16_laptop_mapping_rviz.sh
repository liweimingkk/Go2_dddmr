#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP_FILE="${GO2_LAPTOP_ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
ORIN_OPERATOR_IP="${GO2_ORIN_OPERATOR_IP:-192.168.50.1}"
RVIZ_CONFIG="${GO2_MAPPING_RVIZ_CONFIG:-${WS_ROOT}/src/dddmr_lego_loam/lego_loam_bor/rviz/go2_xt16_mapping_laptop.rviz}"
WAIT_SEC="${GO2_MAPPING_RVIZ_WAIT_SEC:-45}"
SAMPLE_TIMEOUT_SEC="${GO2_MAPPING_RVIZ_SAMPLE_TIMEOUT_SEC:-8}"
CHECK_ONLY=false
REQUIRED_TOPICS=(/lego_loam_map /lego_loam_ground)

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_laptop_mapping_rviz.sh
  scripts/run_go2_xt16_laptop_mapping_rviz.sh --check-only

Starts a read-only laptop RViz2 view of the mapping process running on the
Orin. It does not launch mapping, navigation, or any Go2 command publisher.

Robot-side mapping must expose the operator Wi-Fi DDS interface, for example:
  GO2_DDS_EXTRA_IFACES=wlan0

Environment overrides:
  GO2_ORIN_OPERATOR_IP=192.168.50.1
  GO2_NET_IFACE=<laptop Wi-Fi interface>
  ROS_DOMAIN_ID=0
  GO2_MAPPING_RVIZ_WAIT_SEC=45
  GO2_MAPPING_RVIZ_SAMPLE_TIMEOUT_SEC=8
  GO2_MAPPING_RVIZ_CONFIG=/absolute/path/to/config.rviz
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

is_nonnegative_integer() {
  [[ "$1" =~ ^[0-9]+$ ]]
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

detect_operator_iface() {
  local route_iface
  route_iface="$(ip route get "${ORIN_OPERATOR_IP}" 2>/dev/null | awk '
    {
      for (i = 1; i <= NF; i++) {
        if ($i == "dev" && (i + 1) <= NF) {
          print $(i + 1)
          exit
        }
      }
    }
  ')"
  [[ -n "${route_iface}" ]] || \
    die "Could not find a route to Orin ${ORIN_OPERATOR_IP}; connect the laptop to the Orin operator Wi-Fi."
  printf '%s\n' "${route_iface}"
}

topic_has_publisher() {
  local topic="$1"
  local report
  report="$(
    ros2 topic info "${topic}" --no-daemon --spin-time 0.5 2>/dev/null ||
      true
  )"
  awk '
    $1 == "Publisher" && $2 == "count:" && ($3 + 0) > 0 {
      found = 1
    }
    END {
      exit !found
    }
  ' <<<"${report}"
}

wait_for_mapping_publishers() {
  local deadline=$((SECONDS + WAIT_SEC))
  local topic
  local missing=()

  printf 'Waiting up to %ss for robot-side mapping topics' "${WAIT_SEC}"
  while true; do
    missing=()
    for topic in "${REQUIRED_TOPICS[@]}"; do
      if ! topic_has_publisher "${topic}"; then
        missing+=("${topic}")
      fi
    done
    if [[ "${#missing[@]}" -eq 0 ]]; then
      printf ': ready.\n'
      return 0
    fi
    if (( SECONDS >= deadline )); then
      printf ': timed out.\n' >&2
      printf 'Missing publisher(s): %s\n' "${missing[*]}" >&2
      printf '%s\n' \
        'Restart robot-side mapping with GO2_DDS_EXTRA_IFACES=wlan0 and use the same ROS_DOMAIN_ID.' >&2
      return 1
    fi
    printf '.'
    sleep 1
  done
}

sample_cloud_width() {
  local topic="$1"
  local sample
  sample="$(
    timeout "${SAMPLE_TIMEOUT_SEC}" \
      ros2 topic echo "${topic}" sensor_msgs/msg/PointCloud2 \
      --no-daemon \
      --spin-time 1 \
      --qos-reliability reliable \
      --qos-durability volatile \
      --field width \
      --once 2>/dev/null || true
  )"
  awk '/^[[:space:]]*[0-9]+[[:space:]]*$/ {print $1; exit}' <<<"${sample}"
}

case "${1:-}" in
  "")
    (( $# == 0 )) || {
      usage >&2
      exit 2
    }
    ;;
  --check-only)
    (( $# == 1 )) || {
      usage >&2
      exit 2
    }
    CHECK_ONLY=true
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

is_nonnegative_integer "${WAIT_SEC}" || \
  die "GO2_MAPPING_RVIZ_WAIT_SEC must be a nonnegative integer."
is_positive_integer "${SAMPLE_TIMEOUT_SEC}" || \
  die "GO2_MAPPING_RVIZ_SAMPLE_TIMEOUT_SEC must be a positive integer."
[[ -f "${ROS_SETUP_FILE}" ]] || die "ROS setup file not found: ${ROS_SETUP_FILE}"
[[ -f "${RVIZ_CONFIG}" ]] || die "Laptop mapping RViz config not found: ${RVIZ_CONFIG}"
if [[ "${CHECK_ONLY}" != "true" ]]; then
  [[ -n "${DISPLAY:-}" ]] || \
    die "DISPLAY is empty; run this command from the laptop graphical session."
fi

export GO2_NET_IFACE="${GO2_NET_IFACE:-$(detect_operator_iface)}"
ip link show dev "${GO2_NET_IFACE}" >/dev/null 2>&1 || \
  die "Laptop DDS interface does not exist: ${GO2_NET_IFACE}"
ip -o -4 address show dev "${GO2_NET_IFACE}" | grep -q ' inet ' || \
  die "Laptop DDS interface has no IPv4 address: ${GO2_NET_IFACE}"

set +u
source "${ROS_SETUP_FILE}"
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 was not found after sourcing ${ROS_SETUP_FILE}."
if [[ "${CHECK_ONLY}" != "true" ]]; then
  command -v rviz2 >/dev/null 2>&1 || die "rviz2 is not installed on the laptop."
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export GO2_DDS_EXTRA_IFACES="${GO2_LAPTOP_DDS_EXTRA_IFACES:-}"
export GO2_DDS_RCVBUF_MIN="${GO2_LAPTOP_DDS_RCVBUF_MIN:-default}"
export GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX:-120}"
unset ROS_LOCALHOST_ONLY

source "${SCRIPT_DIR}/setup_go2_dds_env.sh"

wait_for_mapping_publishers || exit 1

map_width="$(sample_cloud_width /lego_loam_map)"
ground_width="$(sample_cloud_width /lego_loam_ground)"
[[ -n "${map_width}" ]] || \
  die "A /lego_loam_map publisher was discovered, but no fresh PointCloud2 sample arrived."
[[ -n "${ground_width}" ]] || \
  die "A /lego_loam_ground publisher was discovered, but no fresh PointCloud2 sample arrived."

printf 'Laptop mapping stream ready on %s (ROS_DOMAIN_ID=%s): map=%s points, ground=%s points.\n' \
  "${GO2_NET_IFACE}" "${ROS_DOMAIN_ID}" "${map_width}" "${ground_width}"
printf '%s\n' \
  'Read-only view: no mapping, navigation, goal, cmd_vel, or Unitree request publisher is started.' \
  'Raw /lidar_points is intentionally not displayed to keep operator Wi-Fi load bounded.'

if [[ "${CHECK_ONLY}" == "true" ]]; then
  exit 0
fi

exec ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" "$@"
