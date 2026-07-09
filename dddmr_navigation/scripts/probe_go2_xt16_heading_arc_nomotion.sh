#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/probe_go2_xt16_heading_arc_nomotion.sh

Starts the Docker navigation dry-run, publishes one /clicked_point goal, and
checks that the planner-side Go2 heading arc emits a nonzero forward component
on /dddmr_go2/dry_run_cmd_vel. This never starts the live Sport adapter and
does not publish /api/sport/request.

Environment:
  GO2_HEADING_ARC_PROBE_LOG_DIR=/tmp
  GO2_HEADING_ARC_GOAL_X=-1.33
  GO2_HEADING_ARC_GOAL_Y=-0.84
  GO2_HEADING_ARC_GOAL_Z=0.0
  GO2_HEADING_ARC_RUN_SECONDS=35
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -gt 0 ]]; then
  echo "Unknown argument: $1" >&2
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKER_WRAPPER="${WS_ROOT}/scripts/dddmr_docker_go2_xt16.sh"
LOG_DIR="${GO2_HEADING_ARC_PROBE_LOG_DIR:-/tmp}"
GOAL_X="${GO2_HEADING_ARC_GOAL_X:--1.33}"
GOAL_Y="${GO2_HEADING_ARC_GOAL_Y:--0.84}"
GOAL_Z="${GO2_HEADING_ARC_GOAL_Z:-0.0}"
RUN_SECONDS="${GO2_HEADING_ARC_RUN_SECONDS:-35}"
STAMP="$(date +%Y%m%d_%H%M%S)"
DOCKER_NAME="go2_heading_arc_nomotion_${STAMP}"
DOCKER_LOG="${LOG_DIR}/go2_heading_arc_nomotion_${STAMP}_docker.log"
CMD_LOG="${LOG_DIR}/go2_heading_arc_nomotion_${STAMP}_cmd_vel.log"
DECISION_LOG="${LOG_DIR}/go2_heading_arc_nomotion_${STAMP}_decision.log"
PATH_LOG="${LOG_DIR}/go2_heading_arc_nomotion_${STAMP}_global_path.log"
PUB_LOG="${LOG_DIR}/go2_heading_arc_nomotion_${STAMP}_publish_goal.log"
SUMMARY_LOG="${LOG_DIR}/go2_heading_arc_nomotion_${STAMP}.env"

docker_pid=""
cmd_echo_pid=""
decision_echo_pid=""
path_echo_pid=""
cleanup_started="false"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

cleanup() {
  local status=$?
  if [[ "${cleanup_started}" == "true" ]]; then
    exit "${status}"
  fi
  cleanup_started="true"

  for pid in "${cmd_echo_pid}" "${decision_echo_pid}" "${path_echo_pid}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      wait "${pid}" >/dev/null 2>&1 || true
    fi
  done

  if docker ps --format '{{.Names}}' 2>/dev/null | rg -qx "${DOCKER_NAME}"; then
    docker stop "${DOCKER_NAME}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${docker_pid}" ]]; then
    wait "${docker_pid}" >/dev/null 2>&1 || true
  fi

  if [[ -f "${SUMMARY_LOG}" ]]; then
    echo "SUMMARY_LOG=${SUMMARY_LOG}"
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

assert_no_conflicting_runtime() {
  local docker_matches
  docker_matches="$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    rg 'go2_xt16|dddmr_go2_xt16|dddmr_navigation|go2_heading_arc_nomotion' || true)"
  if [[ -n "${docker_matches}" ]]; then
    echo "${docker_matches}" >&2
    die "stop existing Go2/DDDMR Docker containers before running no-motion heading arc probe"
  fi

  local proc_matches
  proc_matches="$(ps -eo pid,args | \
    rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | \
    rg -v 'probe_go2_xt16_heading_arc_nomotion|dddmr_docker_go2_xt16|bash -lc|py_compile|sed -n|nl -ba|rg |ps -eo' || true)"
  if [[ -n "${proc_matches}" ]]; then
    echo "${proc_matches}" >&2
    die "stop existing navigation/RViz/Sport adapter/ros2 pub/echo processes first"
  fi
}

docker_ros() {
  docker exec "${DOCKER_NAME}" bash -lc "
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
cd /root/dddmr_navigation
source \"\${DDDMR_INSTALL_BASE}/setup.bash\"
set -u
$*"
}

wait_for_container() {
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if docker ps --format '{{.Names}}' 2>/dev/null | rg -qx "${DOCKER_NAME}"; then
      return 0
    fi
    sleep 0.5
  done
  die "timed out waiting for Docker container ${DOCKER_NAME}"
}

wait_for_navigation() {
  local deadline=$((SECONDS + 25))
  while (( SECONDS < deadline )); do
    if docker_ros "ros2 action list 2>/dev/null | grep -Fxq '/p2p_move_base'" && \
       docker_ros "ros2 topic info /dddmr_go2/dry_run_cmd_vel 2>/dev/null | grep -Fq 'Type: geometry_msgs/msg/Twist'"; then
      return 0
    fi
    sleep 1
  done
  die "timed out waiting for dry-run navigation action/topic"
}

start_echoes() {
  docker_ros "timeout -s INT -k 2s 14s ros2 topic echo /dddmr_go2/dry_run_cmd_vel geometry_msgs/msg/Twist" \
    >"${CMD_LOG}" 2>&1 &
  cmd_echo_pid="$!"

  docker_ros "timeout -s INT -k 2s 14s ros2 topic echo /dddmr_go2/p2p_decision std_msgs/msg/String" \
    >"${DECISION_LOG}" 2>&1 &
  decision_echo_pid="$!"

  docker_ros "timeout -s INT -k 2s 14s ros2 topic echo /global_path nav_msgs/msg/Path" \
    >"${PATH_LOG}" 2>&1 &
  path_echo_pid="$!"

  sleep 1.0
}

publish_goal() {
  docker_ros "ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
\"{header: {frame_id: 'map'}, point: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}}\"" \
    >"${PUB_LOG}" 2>&1
}

validate_outputs() {
  rg -q --fixed-strings 'Use theory name: differential_drive_go2_heading_arc' "${DOCKER_LOG}" || \
    die "Docker log does not show differential_drive_go2_heading_arc loaded"
  rg -q --fixed-strings 'rotation_forward_x: 0.030' "${DOCKER_LOG}" || \
    die "Docker log does not show rotation_forward_x=0.030"
  rg -q --fixed-strings 'initial_heading_trajectory_generator: differential_drive_go2_heading_arc' "${DOCKER_LOG}" || \
    die "Docker log does not show p2p using Go2 heading arc for initial alignment"

  /usr/bin/python3 - "${CMD_LOG}" <<'PY'
import re
import sys

path = sys.argv[1]
text = open(path, encoding="utf-8", errors="replace").read()
blocks = re.split(r"\n---\s*\n", text)
matches = []
for block in blocks:
    x_match = re.search(r"linear:\s*\n\s*x:\s*([-+0-9.eE]+)", block)
    z_match = re.search(r"angular:\s*\n\s*x:\s*[-+0-9.eE]+\s*\n\s*y:\s*[-+0-9.eE]+\s*\n\s*z:\s*([-+0-9.eE]+)", block)
    if not x_match or not z_match:
        continue
    x = float(x_match.group(1))
    z = float(z_match.group(1))
    if x >= 0.025 and abs(z) >= 0.15:
        matches.append((x, z))

if not matches:
    raise SystemExit("no dry-run cmd_vel sample had x>=0.025 and |z|>=0.15")
print(f"heading arc cmd_vel samples: {matches[:5]}")
PY

  rg -q --fixed-strings 'd_align_heading' "${DECISION_LOG}" || \
    die "decision log does not show d_align_heading"
  rg -q --fixed-strings 'poses:' "${PATH_LOG}" || \
    die "global path log does not show a path message"
}

assert_no_conflicting_runtime
mkdir -p "${LOG_DIR}"

echo "DOCKER_NAME=${DOCKER_NAME}"
echo "DOCKER_LOG=${DOCKER_LOG}"
DDDMR_DOCKER_NAME="${DOCKER_NAME}" \
RUN_SECONDS="${RUN_SECONDS}" \
RVIZ=false \
  "${DOCKER_WRAPPER}" navigation-dry-run >"${DOCKER_LOG}" 2>&1 &
docker_pid="$!"

wait_for_container
wait_for_navigation
start_echoes
publish_goal

wait "${cmd_echo_pid}" || true
cmd_echo_pid=""
wait "${decision_echo_pid}" || true
decision_echo_pid=""
wait "${path_echo_pid}" || true
path_echo_pid=""

validate_outputs

cat >"${SUMMARY_LOG}" <<EOF
RESULT=GO2_HEADING_ARC_NOMOTION_PASS
GOAL_X=${GOAL_X}
GOAL_Y=${GOAL_Y}
GOAL_Z=${GOAL_Z}
DOCKER_NAME=${DOCKER_NAME}
DOCKER_LOG=${DOCKER_LOG}
CMD_LOG=${CMD_LOG}
DECISION_LOG=${DECISION_LOG}
PATH_LOG=${PATH_LOG}
PUB_LOG=${PUB_LOG}
EOF

echo "SUMMARY_LOG=${SUMMARY_LOG}"
echo "RESULT: GO2_HEADING_ARC_NOMOTION_PASS"
