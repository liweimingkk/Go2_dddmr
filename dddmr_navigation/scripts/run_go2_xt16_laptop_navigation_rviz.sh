#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP_FILE="${GO2_LAPTOP_ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
ORIN_OPERATOR_IP="${GO2_ORIN_OPERATOR_IP:-192.168.50.1}"
RVIZ_CONFIG="${GO2_NAV_RVIZ_CONFIG:-${WS_ROOT}/src/dddmr_beginner_guide/rviz/go2_xt16_navigation_laptop.rviz}"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
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

[[ -f "${ROS_SETUP_FILE}" ]] || die "ROS setup file not found: ${ROS_SETUP_FILE}"
[[ -f "${RVIZ_CONFIG}" ]] || die "Laptop RViz config not found: ${RVIZ_CONFIG}"

export GO2_NET_IFACE="${GO2_NET_IFACE:-$(detect_operator_iface)}"
ip link show dev "${GO2_NET_IFACE}" >/dev/null 2>&1 || \
  die "Laptop DDS interface does not exist: ${GO2_NET_IFACE}"
ip -o -4 address show dev "${GO2_NET_IFACE}" | grep -q ' inet ' || \
  die "Laptop DDS interface has no IPv4 address: ${GO2_NET_IFACE}"

set +u
source "${ROS_SETUP_FILE}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export GO2_DDS_EXTRA_IFACES="${GO2_LAPTOP_DDS_EXTRA_IFACES:-}"
export GO2_DDS_RCVBUF_MIN="${GO2_LAPTOP_DDS_RCVBUF_MIN:-default}"
export GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX:-120}"
unset ROS_LOCALHOST_ONLY

source "${SCRIPT_DIR}/setup_go2_dds_env.sh"

printf 'Starting laptop navigation RViz on %s (ROS_DOMAIN_ID=%s).\n' \
  "${GO2_NET_IFACE}" "${ROS_DOMAIN_ID}"
printf 'Goal tool: 2D Goal Pose -> /goal_pose_3d\n'
printf 'WARNING: clicking a goal can move the Go2 when the supervised live adapter is active.\n'

exec ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" "$@"
