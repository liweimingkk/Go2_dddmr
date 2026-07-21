#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_go2_xt16_navigation_supervised_live.sh [--dry-run|--live]

Mixed-runtime launcher for Go2 XT16 DDDMR navigation.

Modes:
  --dry-run  Launch Docker navigation/RViz with the dry-run Sport logger only.
  --live     Launch Docker navigation/RViz as a velocity source and run the
             host Go2 Sport adapter. Can publish /api/sport/request after RViz
             click goals. Requires:
             GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV
             GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_..._summary.env

Environment:
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV
  GO2_SPORT_PROBE_SUMMARY=<summary from run_go2_sport_adapter_supervised_probe.sh --live>
  GO2_NAV_LOG_DIR=/tmp
  RVIZ=true
  PUBLISH_STATIC_TF=true
  GO2_NAV_CMD_TOPIC=/dddmr_go2/safe_cmd_vel
  GO2_SPORT_PUBLISH_RATE_HZ=50.0
  GO2_SPORT_CMD_TIMEOUT_SEC=0.20
  GO2_SPORT_MAX_X=0.30
  GO2_SPORT_MAX_Y=0.0
  GO2_SPORT_MAX_YAW=0.25
  GO2_SPORT_ZERO_EPSILON=0.001
  GO2_SPORT_STOP_KEEPALIVE_HZ=2.0
  GO2_ENABLE_YAW_ARC_SHIM=false
  GO2_YAW_ARC_SHIM_MODE=off        # off | preview | live
  GO2_YAW_ARC_SHIM_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_SHIM
  GO2_YAW_ARC_FORWARD_X=0.03
  GO2_YAW_ARC_MIN_ABS_YAW=0.20
  GO2_YAW_ARC_TRIGGER_ABS_YAW=0.03
  GO2_YAW_ARC_ALLOWED_DECISIONS=d_align_heading
  GO2_YAW_ARC_NOMOTION_REPORT=<required when GO2_YAW_ARC_SHIM_MODE=live>
  GO2_DECISION_TOPIC=/dddmr_go2/p2p_decision
  GO2_DECISION_TIMEOUT_SEC=0.30
  GO2_REQUIRE_MOTION_DECISION=true
  GO2_MOTION_ALLOWED_DECISIONS=d_controlling,d_align_heading,d_align_goal_heading,d_recovery_waitdone
  GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED=true
  GO2_MAX_CONTINUOUS_YAW_ARC_SEC=4.0
  GO2_NAV_DRY_RUN_COMMAND=navigation-dry-run
  GO2_NAV_DOCKER_COMMAND=navigation-live-source

Live runtime shape:
  Docker: DDDMR navigation/RViz publishes /dddmr_go2/dry_run_cmd_vel,
          then go2_nav_cmd_gate republishes /dddmr_go2/safe_cmd_vel.
  Host:   timer-driven go2_sport_cmd_vel_adapter.py consumes safe_cmd_vel
          and publishes /api/sport/request.
EOF
}

mode="dry-run"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      mode="dry-run"
      shift
      ;;
    --live)
      mode="live"
      shift
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
DOCKER_WRAPPER="${WS_ROOT}/scripts/dddmr_docker_go2_xt16.sh"
ADAPTER="${WS_ROOT}/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py"
CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_NAV"
YAW_ARC_CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_YAW_ARC_SHIM"
REAL_REQUEST_TOPIC="/api/sport/request"
CMD_TOPIC="${GO2_NAV_CMD_TOPIC:-/dddmr_go2/safe_cmd_vel}"
sport_publish_rate_hz="${GO2_SPORT_PUBLISH_RATE_HZ:-50.0}"
sport_cmd_timeout_sec="${GO2_SPORT_CMD_TIMEOUT_SEC:-0.20}"
sport_max_x="${GO2_SPORT_MAX_X:-0.30}"
sport_max_y="${GO2_SPORT_MAX_Y:-0.0}"
sport_max_yaw="${GO2_SPORT_MAX_YAW:-0.25}"
sport_zero_epsilon="${GO2_SPORT_ZERO_EPSILON:-0.001}"
sport_stop_keepalive_hz="${GO2_SPORT_STOP_KEEPALIVE_HZ:-2.0}"
enable_yaw_arc_shim="${GO2_ENABLE_YAW_ARC_SHIM:-false}"
yaw_arc_shim_mode="${GO2_YAW_ARC_SHIM_MODE:-off}"
yaw_arc_forward_x="${GO2_YAW_ARC_FORWARD_X:-0.03}"
yaw_arc_min_abs_yaw="${GO2_YAW_ARC_MIN_ABS_YAW:-0.20}"
yaw_arc_trigger_abs_yaw="${GO2_YAW_ARC_TRIGGER_ABS_YAW:-0.03}"
yaw_arc_allowed_decisions="${GO2_YAW_ARC_ALLOWED_DECISIONS:-d_align_heading}"
yaw_arc_nomotion_report="${GO2_YAW_ARC_NOMOTION_REPORT:-}"
decision_topic="${GO2_DECISION_TOPIC:-/dddmr_go2/p2p_decision}"
decision_timeout_sec="${GO2_DECISION_TIMEOUT_SEC:-0.30}"
require_motion_decision="${GO2_REQUIRE_MOTION_DECISION:-true}"
motion_allowed_decisions="${GO2_MOTION_ALLOWED_DECISIONS:-d_controlling,d_align_heading,d_align_goal_heading,d_recovery_waitdone}"
zero_yaw_only_when_shim_disallowed="${GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED:-true}"
max_continuous_yaw_arc_sec="${GO2_MAX_CONTINUOUS_YAW_ARC_SEC:-4.0}"
rviz="${RVIZ:-true}"
publish_static_tf="${PUBLISH_STATIC_TF:-true}"
log_dir="${GO2_NAV_LOG_DIR:-/tmp}"
stamp="$(date +%Y%m%d_%H%M%S)"
nav_request_id_base="$(date +%s)"
docker_name="go2_xt16_nav_live_${stamp}"
docker_dry_run_command="${GO2_NAV_DRY_RUN_COMMAND:-navigation-dry-run}"
docker_live_command="${GO2_NAV_DOCKER_COMMAND:-navigation-live-source}"
docker_log="${log_dir}/go2_xt16_nav_live_${stamp}_docker.log"
adapter_log="${log_dir}/go2_xt16_nav_live_${stamp}_adapter.log"
request_echo_log="${log_dir}/go2_xt16_nav_live_${stamp}_request_echo.log"
summary_log="${log_dir}/go2_xt16_nav_live_${stamp}_summary.env"
docker_pid=""
adapter_pid=""
echo_pid=""
cleanup_started="false"
live_runtime_started="false"

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

  if [[ "${live_runtime_started}" == "true" ]]; then
    publish_stopmove_burst "supervisor cleanup" || true
  fi

  if [[ -n "${adapter_pid}" ]]; then
    kill "${adapter_pid}" >/dev/null 2>&1 || true
    wait "${adapter_pid}" >/dev/null 2>&1 || true
  fi
  if docker ps --format '{{.Names}}' 2>/dev/null | rg -qx "${docker_name}"; then
    docker stop "${docker_name}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${docker_pid}" ]]; then
    wait "${docker_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${echo_pid}" ]]; then
    kill "${echo_pid}" >/dev/null 2>&1 || true
    wait "${echo_pid}" >/dev/null 2>&1 || true
  fi

  if [[ "${mode}" == "live" ]]; then
    cat >"${summary_log}" <<EOF
MODE=${mode}
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_ID_BASE=${nav_request_id_base}
DOCKER_NAME=${docker_name}
DOCKER_LOG=${docker_log}
ADAPTER_LOG=${adapter_log}
REQUEST_ECHO_LOG=${request_echo_log}
CMD_TOPIC=${CMD_TOPIC}
REQUEST_TOPIC=${REAL_REQUEST_TOPIC}
SPORT_PUBLISH_RATE_HZ=${sport_publish_rate_hz}
SPORT_CMD_TIMEOUT_SEC=${sport_cmd_timeout_sec}
SPORT_MAX_X=${sport_max_x}
SPORT_MAX_Y=${sport_max_y}
SPORT_MAX_YAW=${sport_max_yaw}
SPORT_ZERO_EPSILON=${sport_zero_epsilon}
SPORT_STOP_KEEPALIVE_HZ=${sport_stop_keepalive_hz}
ENABLE_YAW_ARC_SHIM=${enable_yaw_arc_shim}
YAW_ARC_SHIM_MODE=${yaw_arc_shim_mode}
YAW_ARC_FORWARD_X=${yaw_arc_forward_x}
YAW_ARC_MIN_ABS_YAW=${yaw_arc_min_abs_yaw}
YAW_ARC_TRIGGER_ABS_YAW=${yaw_arc_trigger_abs_yaw}
YAW_ARC_ALLOWED_DECISIONS=${yaw_arc_allowed_decisions}
YAW_ARC_NOMOTION_REPORT=${yaw_arc_nomotion_report}
DECISION_TOPIC=${decision_topic}
DECISION_TIMEOUT_SEC=${decision_timeout_sec}
REQUIRE_MOTION_DECISION=${require_motion_decision}
MOTION_ALLOWED_DECISIONS=${motion_allowed_decisions}
ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED=${zero_yaw_only_when_shim_disallowed}
MAX_CONTINUOUS_YAW_ARC_SEC=${max_continuous_yaw_arc_sec}
EOF
    echo "SUMMARY_LOG=${summary_log}"
    echo "DOCKER_LOG=${docker_log}"
    echo "ADAPTER_LOG=${adapter_log}"
    echo "REQUEST_ECHO_LOG=${request_echo_log}"
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

assert_no_conflicting_runtime() {
  local docker_matches
  docker_matches="$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    rg 'go2_xt16|dddmr_go2_xt16|dddmr_navigation' || true)"
  if [[ -n "${docker_matches}" ]]; then
    echo "${docker_matches}" >&2
    die "stop existing Go2/DDDMR Docker containers before launching mixed navigation"
  fi

  local proc_matches
  proc_matches="$(ps -eo pid,args | \
    rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | \
    rg -v 'run_go2_xt16_navigation_supervised_live|dddmr_docker_go2_xt16|bash -lc|py_compile|sed -n|nl -ba|rg |ps -eo' || true)"
  if [[ -n "${proc_matches}" ]]; then
    echo "${proc_matches}" >&2
    die "stop existing navigation/RViz/Sport adapter/ros2 pub/echo processes first"
  fi
}

source_host_go2_ros() {
  [[ -f "${GO2_SETUP}" ]] || die "missing Go2 ROS setup: ${GO2_SETUP}"
  [[ -f "${WS_ROOT}/scripts/setup_go2_dds_env.sh" ]] || die "missing DDS env script"
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

wait_for_cmd_publisher() {
  local deadline=$((SECONDS + 20))
  local info
  while (( SECONDS < deadline )); do
    info="$(ros2 topic info "${CMD_TOPIC}" 2>&1 || true)"
    if [[ "${info}" == *"Type: geometry_msgs/msg/Twist"* ]] && \
       [[ "${info}" =~ Publisher\ count:\ [1-9][0-9]* ]]; then
      echo "=== ${CMD_TOPIC}"
      echo "${info}"
      return 0
    fi
    sleep 1
  done
  echo "${info:-}" >&2
  die "timed out waiting for Docker velocity source publisher on ${CMD_TOPIC}"
}

validate_probe_summary() {
  local summary="$1"
  [[ -f "${summary}" ]] || die "missing GO2_SPORT_PROBE_SUMMARY: ${summary}"

  # shellcheck disable=SC1090
  source "${summary}"
  [[ "${MODE:-}" == "live" ]] || die "probe summary MODE is not live"
  [[ "${RESULT:-}" == "GO2_SPORT_ADAPTER_live_COMPLETE" ]] || die "probe summary RESULT is not live complete"
  [[ -n "${REQUEST_ID_BASE:-}" ]] || die "probe summary missing REQUEST_ID_BASE"
  [[ -f "${REQUEST_ECHO_LOG:-}" ]] || die "probe summary missing REQUEST_ECHO_LOG file"

  /usr/bin/python3 - "${REQUEST_ECHO_LOG}" "${REQUEST_ID_BASE}" <<'PY'
import re
import sys

path = sys.argv[1]
base_id = int(sys.argv[2])
text = open(path, encoding="utf-8").read()

seen = []
for block in re.split(r"\n---\s*\n", text):
    id_match = re.search(r"^\s*id:\s*(-?\d+)\s*$", block, re.MULTILINE)
    api_match = re.search(r"^\s*api_id:\s*(-?\d+)\s*$", block, re.MULTILINE)
    if not id_match or not api_match:
        continue
    req_id = int(id_match.group(1))
    api_id = int(api_match.group(1))
    if req_id > base_id:
        seen.append((req_id, api_id))

if not any(api_id == 1008 for _, api_id in seen):
    raise SystemExit(f"live probe summary did not prove Move api_id=1008; seen={seen}")
if not any(api_id == 1003 for _, api_id in seen):
    raise SystemExit(f"live probe summary did not prove StopMove api_id=1003; seen={seen}")

print(f"validated live probe request ids: {seen}")
PY
}

validate_yaw_arc_settings() {
  case "${enable_yaw_arc_shim}" in
    true|false) ;;
    *) die "GO2_ENABLE_YAW_ARC_SHIM must be true or false" ;;
  esac
  case "${yaw_arc_shim_mode}" in
    off|preview|live) ;;
    *) die "GO2_YAW_ARC_SHIM_MODE must be off, preview, or live" ;;
  esac

  /usr/bin/python3 - \
    "${yaw_arc_forward_x}" \
    "${yaw_arc_min_abs_yaw}" \
    "${yaw_arc_trigger_abs_yaw}" \
    "${decision_timeout_sec}" \
    "${max_continuous_yaw_arc_sec}" <<'PY'
import math
import sys

names = [
    "GO2_YAW_ARC_FORWARD_X",
    "GO2_YAW_ARC_MIN_ABS_YAW",
    "GO2_YAW_ARC_TRIGGER_ABS_YAW",
    "GO2_DECISION_TIMEOUT_SEC",
    "GO2_MAX_CONTINUOUS_YAW_ARC_SEC",
]
for name, raw in zip(names, sys.argv[1:]):
    try:
        value = float(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be numeric: {exc}")
    if not math.isfinite(value):
        raise SystemExit(f"{name} must be finite")
if float(sys.argv[1]) < 0.0 or float(sys.argv[1]) > 0.05:
    raise SystemExit("GO2_YAW_ARC_FORWARD_X must be within [0.0, 0.05]")
if float(sys.argv[2]) < 0.0 or float(sys.argv[2]) > 0.30:
    raise SystemExit("GO2_YAW_ARC_MIN_ABS_YAW must be within [0.0, 0.30]")
if float(sys.argv[3]) < 0.0 or float(sys.argv[3]) > 0.30:
    raise SystemExit("GO2_YAW_ARC_TRIGGER_ABS_YAW must be within [0.0, 0.30]")
if float(sys.argv[4]) <= 0.0 or float(sys.argv[4]) > 2.0:
    raise SystemExit("GO2_DECISION_TIMEOUT_SEC must be within (0.0, 2.0]")
if float(sys.argv[5]) <= 0.0 or float(sys.argv[5]) > 5.0:
    raise SystemExit("GO2_MAX_CONTINUOUS_YAW_ARC_SEC must be within (0.0, 5.0]")
PY

  case "${zero_yaw_only_when_shim_disallowed}" in
    true|false) ;;
    *) die "GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED must be true or false" ;;
  esac
  case "${require_motion_decision}" in
    true|false) ;;
    *) die "GO2_REQUIRE_MOTION_DECISION must be true or false" ;;
  esac
  validate_allowed_decisions "${yaw_arc_allowed_decisions}"
  validate_allowed_decisions "${motion_allowed_decisions}"

  if [[ "${enable_yaw_arc_shim}" == "true" && "${yaw_arc_shim_mode}" == "live" ]]; then
    [[ "${GO2_YAW_ARC_SHIM_CONFIRM:-}" == "${YAW_ARC_CONFIRM_PHRASE}" ]] || \
      die "live yaw-arc shim requires GO2_YAW_ARC_SHIM_CONFIRM=${YAW_ARC_CONFIRM_PHRASE}"
    validate_yaw_arc_nomotion_report "${yaw_arc_nomotion_report}"
  fi
}

validate_allowed_decisions() {
  local raw="$1"
  [[ -n "${raw}" ]] || die "GO2_YAW_ARC_ALLOWED_DECISIONS must not be empty"
  local token
  IFS=',' read -r -a tokens <<<"${raw}"
  for token in "${tokens[@]}"; do
    [[ -n "${token}" ]] || die "GO2_YAW_ARC_ALLOWED_DECISIONS contains an empty entry"
    [[ "${token}" =~ ^[A-Za-z0-9_]+$ ]] || \
      die "GO2_YAW_ARC_ALLOWED_DECISIONS entry is invalid: ${token}"
  done
}

validate_yaw_arc_nomotion_report() {
  local report="$1"
  [[ -n "${report}" ]] || \
    die "live yaw-arc shim requires GO2_YAW_ARC_NOMOTION_REPORT from scripts/check_go2_yaw_arc_shim_nomotion.sh"
  [[ -f "${report}" ]] || die "missing GO2_YAW_ARC_NOMOTION_REPORT: ${report}"

  rg -q --fixed-strings 'RESULT=GO2_YAW_ARC_SHIM_NOMOTION_PASS' "${report}" || \
    die "GO2_YAW_ARC_NOMOTION_REPORT did not pass: ${report}"

  local allowed_log recovery_log stale_log report_forward_x report_min_abs_yaw report_allowed_decisions report_motion_allowed_decisions report_max_continuous report_yaw expected_transform
  allowed_log="$(awk -F= '$1=="ALLOWED_LOG"{print $2}' "${report}")"
  recovery_log="$(awk -F= '$1=="RECOVERY_LOG"{print $2}' "${report}")"
  stale_log="$(awk -F= '$1=="STALE_LOG"{print $2}' "${report}")"
  report_yaw="$(awk -F= '$1=="YAW"{print $2}' "${report}")"
  report_forward_x="$(awk -F= '$1=="FORWARD_X"{print $2}' "${report}")"
  report_min_abs_yaw="$(awk -F= '$1=="MIN_ABS_YAW"{print $2}' "${report}")"
  report_allowed_decisions="$(awk -F= '$1=="ALLOWED_DECISIONS"{print $2}' "${report}")"
  report_motion_allowed_decisions="$(awk -F= '$1=="MOTION_ALLOWED_DECISIONS"{print $2}' "${report}")"
  report_max_continuous="$(awk -F= '$1=="MAX_CONTINUOUS_YAW_ARC_SEC"{print $2}' "${report}")"

  [[ -f "${allowed_log}" ]] || die "missing allowed yaw-arc no-motion log: ${allowed_log}"
  [[ -f "${recovery_log}" ]] || die "missing recovery yaw-arc no-motion log: ${recovery_log}"
  [[ -f "${stale_log}" ]] || die "missing stale yaw-arc no-motion log: ${stale_log}"
  [[ "${report_allowed_decisions}" == "${yaw_arc_allowed_decisions}" ]] || \
    die "GO2_YAW_ARC_NOMOTION_REPORT allowed decisions (${report_allowed_decisions}) do not match current (${yaw_arc_allowed_decisions})"
  [[ "${report_motion_allowed_decisions}" == "${motion_allowed_decisions}" ]] || \
    die "GO2_YAW_ARC_NOMOTION_REPORT motion decisions (${report_motion_allowed_decisions}) do not match current (${motion_allowed_decisions})"
  /usr/bin/python3 - "${report_forward_x}" "${yaw_arc_forward_x}" "${report_min_abs_yaw}" "${yaw_arc_min_abs_yaw}" "${report_max_continuous}" "${max_continuous_yaw_arc_sec}" <<'PY'
import math
import sys

report_forward_x, current_forward_x, report_min_yaw, current_min_yaw, report_max_continuous, current_max_continuous = map(float, sys.argv[1:])
if not math.isclose(report_forward_x, current_forward_x, rel_tol=0.0, abs_tol=1e-9):
    raise SystemExit(
        f"GO2_YAW_ARC_NOMOTION_REPORT FORWARD_X={report_forward_x} "
        f"does not match current GO2_YAW_ARC_FORWARD_X={current_forward_x}"
    )
if not math.isclose(report_min_yaw, current_min_yaw, rel_tol=0.0, abs_tol=1e-9):
    raise SystemExit(
        f"GO2_YAW_ARC_NOMOTION_REPORT MIN_ABS_YAW={report_min_yaw} "
        f"does not match current GO2_YAW_ARC_MIN_ABS_YAW={current_min_yaw}"
    )
if not math.isclose(report_max_continuous, current_max_continuous, rel_tol=0.0, abs_tol=1e-9):
    raise SystemExit(
        f"GO2_YAW_ARC_NOMOTION_REPORT MAX_CONTINUOUS_YAW_ARC_SEC={report_max_continuous} "
        f"does not match current GO2_MAX_CONTINUOUS_YAW_ARC_SEC={current_max_continuous}"
    )
PY

  expected_transform="$(/usr/bin/python3 - "${report_forward_x}" "${report_yaw}" "${report_min_abs_yaw}" <<'PY'
import json
import math
import sys

x = float(sys.argv[1])
yaw = float(sys.argv[2])
min_abs_yaw = float(sys.argv[3])
shim_yaw = math.copysign(max(abs(yaw), min_abs_yaw), yaw)
print(json.dumps({"x": x, "y": 0.0, "z": shim_yaw}, separators=(",", ":")))
PY
)"

  rg -q --fixed-strings "transformed_sport=${expected_transform}" "${allowed_log}" || \
    die "allowed no-motion log does not show expected transform ${expected_transform}"
  rg -q --fixed-strings 'shim=recovery_pure_yaw_no_arc' "${recovery_log}" || \
    die "recovery no-motion log does not show pure-yaw pass without an arc"
  rg -q --fixed-strings 'motion decision gate blocked stale_decision=d_align_heading' "${stale_log}" || \
    die "stale no-motion log does not show blocked stale decision"

  echo "Validated yaw-arc no-motion report: ${report}"
}

publish_stopmove_burst() {
  local reason="$1"
  /usr/bin/python3 - "${REAL_REQUEST_TOPIC}" "$((nav_request_id_base + 900000))" "${reason}" <<'PY'
import sys
import time

import rclpy
from unitree_api.msg import Request

topic = sys.argv[1]
base_id = int(sys.argv[2])
reason = sys.argv[3]

rclpy.init()
node = rclpy.create_node("go2_xt16_nav_live_stopmove_cleanup")
pub = node.create_publisher(Request, topic, 10)
time.sleep(0.2)
for i in range(3):
    req = Request()
    req.header.identity.id = base_id + i + 1
    req.header.identity.api_id = 1003
    req.parameter = ""
    pub.publish(req)
    node.get_logger().warn(f"{reason}: published {topic} api_id=1003 StopMove")
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

start_docker_source() {
  echo "DOCKER_NAME=${docker_name}"
  echo "DOCKER_LOG=${docker_log}"
  DDDMR_DOCKER_NAME="${docker_name}" \
  RVIZ="${rviz}" \
  PUBLISH_STATIC_TF="${publish_static_tf}" \
    "${DOCKER_WRAPPER}" "${docker_live_command}" \
    >"${docker_log}" 2>&1 &
  docker_pid="$!"
  sleep 1
  if ! kill -0 "${docker_pid}" >/dev/null 2>&1; then
    cat "${docker_log}" >&2 || true
    die "Docker navigation source exited during startup"
  fi
}

start_adapter() {
  echo "ADAPTER_LOG=${adapter_log}"
  /usr/bin/python3 "${ADAPTER}" \
    --ros-args \
    -p cmd_vel_topic:="${CMD_TOPIC}" \
    -p request_topic:="${REAL_REQUEST_TOPIC}" \
    -p enable_sport_output:=true \
    -p allow_real_request_topic:=true \
    -p max_x:="${sport_max_x}" \
    -p max_y:="${sport_max_y}" \
    -p max_yaw:="${sport_max_yaw}" \
    -p publish_rate_hz:="${sport_publish_rate_hz}" \
    -p cmd_timeout_sec:="${sport_cmd_timeout_sec}" \
    -p zero_epsilon:="${sport_zero_epsilon}" \
    -p stop_keepalive_hz:="${sport_stop_keepalive_hz}" \
    -p request_id_base:="${nav_request_id_base}" \
    -p log_period_sec:=0.10 \
    -p enable_yaw_arc_shim:="${enable_yaw_arc_shim}" \
    -p yaw_arc_shim_mode:="'${yaw_arc_shim_mode}'" \
    -p yaw_arc_forward_x:="${yaw_arc_forward_x}" \
    -p yaw_arc_min_abs_yaw:="${yaw_arc_min_abs_yaw}" \
    -p yaw_arc_trigger_abs_yaw:="${yaw_arc_trigger_abs_yaw}" \
    -p yaw_arc_allowed_decisions:="'${yaw_arc_allowed_decisions}'" \
    -p decision_topic:="${decision_topic}" \
    -p decision_timeout_sec:="${decision_timeout_sec}" \
    -p require_motion_decision:="${require_motion_decision}" \
    -p motion_allowed_decisions:="'${motion_allowed_decisions}'" \
    -p zero_yaw_only_when_shim_disallowed:="${zero_yaw_only_when_shim_disallowed}" \
    -p max_continuous_yaw_arc_sec:="${max_continuous_yaw_arc_sec}" \
    >"${adapter_log}" 2>&1 &
  adapter_pid="$!"
  sleep 1
  if ! kill -0 "${adapter_pid}" >/dev/null 2>&1; then
    cat "${adapter_log}" >&2 || true
    die "host Sport adapter exited during startup"
  fi
}

start_request_echo() {
  echo "REQUEST_ECHO_LOG=${request_echo_log}"
  ros2 topic echo "${REAL_REQUEST_TOPIC}" unitree_api/msg/Request \
    >"${request_echo_log}" 2>&1 &
  echo_pid="$!"
}

assert_no_conflicting_runtime

case "${docker_dry_run_command}" in
  navigation-dry-run|outdoor-indoor-dry-run) ;;
  *) die "unsupported GO2_NAV_DRY_RUN_COMMAND=${docker_dry_run_command}" ;;
esac
case "${docker_live_command}" in
  navigation-live-source|outdoor-indoor-live-source) ;;
  *) die "unsupported GO2_NAV_DOCKER_COMMAND=${docker_live_command}" ;;
esac

if [[ "${mode}" == "dry-run" ]]; then
  echo "Launching Docker Go2 XT16 navigation in dry-run RViz mode."
  exec env RVIZ="${rviz}" PUBLISH_STATIC_TF="${publish_static_tf}" \
    "${DOCKER_WRAPPER}" "${docker_dry_run_command}"
fi

[[ "${GO2_NAV_LIVE_CONFIRM:-}" == "${CONFIRM_PHRASE}" ]] || \
  die "live navigation requires GO2_NAV_LIVE_CONFIRM=${CONFIRM_PHRASE}"
[[ -n "${GO2_SPORT_PROBE_SUMMARY:-}" ]] || \
  die "live navigation requires GO2_SPORT_PROBE_SUMMARY from the live adapter probe"
validate_probe_summary "${GO2_SPORT_PROBE_SUMMARY}"
validate_yaw_arc_settings

source_host_go2_ros
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
check_topic_type "${REAL_REQUEST_TOPIC}" "unitree_api/msg/Request"
check_topic_type "/sportmodestate" "unitree_go/msg/SportModeState"
check_topic_type "/lowstate" "unitree_go/msg/LowState"

echo "LIVE NAVIGATION CAN MOVE THE GO2 AFTER RViz GOALS."
echo "Launching Docker velocity source and host Sport adapter with request_id_base=${nav_request_id_base}."
echo "Yaw arc shim: enabled=${enable_yaw_arc_shim} mode=${yaw_arc_shim_mode} allowed_decisions=${yaw_arc_allowed_decisions} decision_topic=${decision_topic}"
start_docker_source
wait_for_cmd_publisher
start_request_echo
start_adapter
live_runtime_started="true"

echo "RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING"
echo "Click only a short, nearby, clear-space RViz goal. Press Ctrl-C here to stop and send StopMove."
wait "${docker_pid}"
