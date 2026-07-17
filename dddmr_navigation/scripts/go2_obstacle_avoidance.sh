#!/usr/bin/env bash
set -eo pipefail

# One-command launcher for the Go2 obstacle-avoidance control utility.
# This launcher does not publish any motion command.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAV_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
UNITREE_SETUP="${NAV_ROOT}/.unitree_msg_ws/install/setup.bash"

if [[ ! -r "${ROS_SETUP}" ]]; then
  printf 'ERROR: ROS 2 setup not found: %s\n' "${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -r "${UNITREE_SETUP}" ]]; then
  printf 'ERROR: Unitree message setup not found: %s\n' "${UNITREE_SETUP}" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${UNITREE_SETUP}"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

exec python3 "${SCRIPT_DIR}/go2_obstacle_avoidance.py" "$@"
