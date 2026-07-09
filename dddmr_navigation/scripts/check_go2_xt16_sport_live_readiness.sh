#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/check_go2_xt16_sport_live_readiness.sh

Read-only readiness gate before the supervised Go2 Sport live probe/navigation.

Checks:
  - no stale Go2/DDDMR Docker container or local nav/adapter/RViz process
  - Go2 ROS setup can be sourced on the host
  - /api/sport/request, /sportmodestate, and /lowstate are visible with expected types
  - live probe and live navigation scripts reject missing confirmations
  - live navigation rejects a missing probe summary even with navigation confirmation

This script does not publish /cmd_vel, /dddmr_go2/dry_run_cmd_vel, or /api/sport/request.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "Unknown argument: $1" >&2
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
SPORT_PROBE="${WS_ROOT}/scripts/run_go2_sport_adapter_supervised_probe.sh"
NAV_LIVE="${WS_ROOT}/scripts/run_go2_xt16_navigation_supervised_live.sh"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

expect_command_failure() {
  local expected="$1"
  shift
  local out
  local status

  set +e
  out="$("$@" 2>&1)"
  status=$?
  set -e

  if [[ "${status}" -eq 0 ]]; then
    echo "${out}" >&2
    die "expected command to fail: $*"
  fi
  if [[ "${out}" != *"${expected}"* ]]; then
    echo "${out}" >&2
    die "expected failure text not found: ${expected}"
  fi
  printf '%s\n' "${out}"
}

check_no_conflicts() {
  local docker_matches
  docker_matches="$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    rg 'go2_xt16|dddmr_go2_xt16|dddmr_navigation' || true)"
  if [[ -n "${docker_matches}" ]]; then
    echo "${docker_matches}" >&2
    die "stale Go2/DDDMR Docker container is still running"
  fi

  local proc_matches
  proc_matches="$(ps -eo pid,args | \
    rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | \
    rg -v 'check_go2_xt16_sport_live_readiness|bash -lc|py_compile|sed -n|nl -ba|rg |ps -eo' || true)"
  if [[ -n "${proc_matches}" ]]; then
    echo "${proc_matches}" >&2
    die "stale nav/adapter/RViz/ros2 pub/echo process is still running"
  fi
}

source_go2_ros() {
  [[ -f "${GO2_SETUP}" ]] || die "missing Go2 ROS setup: ${GO2_SETUP}"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
  set +u
  # shellcheck disable=SC1090
  source "${GO2_SETUP}"
  # shellcheck disable=SC1090
  source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
  set -u
}

check_topic_type() {
  local topic="$1"
  local expected="$2"
  local info
  info="$(timeout 10 ros2 topic info "${topic}" 2>&1)" || {
    echo "${info}" >&2
    die "topic ${topic} is not visible"
  }
  echo "=== ${topic}"
  echo "${info}"
  [[ "${info}" == *"Type: ${expected}"* ]] || die "${topic} type is not ${expected}"
}

echo "=== runtime cleanup check"
check_no_conflicts
echo "OK no stale Go2/DDDMR container or nav/adapter/RViz process"

echo "=== script presence"
[[ -x "${SPORT_PROBE}" ]] || die "missing executable: ${SPORT_PROBE}"
[[ -x "${NAV_LIVE}" ]] || die "missing executable: ${NAV_LIVE}"
echo "OK supervised scripts are executable"

echo "=== Go2 ROS read-only topics"
source_go2_ros
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
check_topic_type "/api/sport/request" "unitree_api/msg/Request"
check_topic_type "/sportmodestate" "unitree_go/msg/SportModeState"
check_topic_type "/lowstate" "unitree_go/msg/LowState"

echo "=== confirmation gates"
expect_command_failure \
  "live mode requires GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2" \
  "${SPORT_PROBE}" --live >/dev/null
echo "OK sport live probe rejects missing confirmation"

expect_command_failure \
  "live navigation requires GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV" \
  "${NAV_LIVE}" --live >/dev/null
echo "OK navigation live rejects missing confirmation"

expect_command_failure \
  "live navigation requires GO2_SPORT_PROBE_SUMMARY from the live adapter probe" \
  env GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV "${NAV_LIVE}" --live >/dev/null
echo "OK navigation live rejects missing live probe summary"

echo "RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS"
