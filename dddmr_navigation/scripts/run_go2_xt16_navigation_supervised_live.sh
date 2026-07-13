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
  GO2_EXPECTED_MODE=-1              # -1 latches the pre-motion value
  GO2_EXPECTED_GAIT_TYPE=-1         # -1 latches the pre-motion value
  GO2_GAIT_STATUS_TIMEOUT_SEC=0.30
  GO2_GAIT_MONITOR_RATE_HZ=20.0

Live runtime shape:
  Docker: DDDMR navigation/RViz publishes /dddmr_go2/dry_run_cmd_vel,
          then go2_nav_cmd_gate republishes /dddmr_go2/safe_cmd_vel.
  Host:   a read-only mode/gait monitor latches the pre-motion state, then the
          timer-driven go2_sport_cmd_vel_adapter.py consumes safe_cmd_vel and
          publishes only Move(1008)/StopMove(1003) to /api/sport/request.
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
GAIT_MONITOR="${WS_ROOT}/src/dddmr_beginner_guide/scripts/go2_gait_state_monitor.py"
API_AUDITOR="${WS_ROOT}/src/dddmr_beginner_guide/scripts/audit_go2_navigation_request_echo.py"
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
expected_mode="${GO2_EXPECTED_MODE:--1}"
expected_gait_type="${GO2_EXPECTED_GAIT_TYPE:--1}"
gait_status_timeout_sec="${GO2_GAIT_STATUS_TIMEOUT_SEC:-0.30}"
gait_monitor_rate_hz="${GO2_GAIT_MONITOR_RATE_HZ:-20.0}"
rviz="${RVIZ:-true}"
publish_static_tf="${PUBLISH_STATIC_TF:-true}"
log_dir="${GO2_NAV_LOG_DIR:-/tmp}"
stamp="$(date +%Y%m%d_%H%M%S)"
nav_request_id_base="$(date +%s)"
docker_name="go2_xt16_nav_live_${stamp}"
docker_log="${log_dir}/go2_xt16_nav_live_${stamp}_docker.log"
adapter_log="${log_dir}/go2_xt16_nav_live_${stamp}_adapter.log"
request_echo_log="${log_dir}/go2_xt16_nav_live_${stamp}_request_echo.log"
gait_monitor_log="${log_dir}/go2_xt16_nav_live_${stamp}_gait_monitor.log"
api_audit_log="${log_dir}/go2_xt16_nav_live_${stamp}_api_audit.env"
summary_log="${log_dir}/go2_xt16_nav_live_${stamp}_summary.env"
docker_pid=""
adapter_pid=""
gait_monitor_pid=""
echo_pid=""
cleanup_started="false"
live_runtime_started="false"
api_audit_result="NOT_RUN"
gait_audit_result="NOT_RUN"
gait_monitor_alive_at_cleanup="false"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

process_is_running() {
  local pid="$1"
  local state=""
  kill -0 "${pid}" >/dev/null 2>&1 || return 1
  state="$(ps -o stat= -p "${pid}" 2>/dev/null || true)"
  [[ -n "${state}" && ! "${state}" =~ ^[[:space:]]*Z ]]
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
  if [[ -n "${gait_monitor_pid}" ]]; then
    if process_is_running "${gait_monitor_pid}"; then
      gait_monitor_alive_at_cleanup="true"
    fi
    kill "${gait_monitor_pid}" >/dev/null 2>&1 || true
    wait "${gait_monitor_pid}" >/dev/null 2>&1 || true
  fi

  if [[ "${mode}" == "live" && -f "${request_echo_log}" ]]; then
    if /usr/bin/python3 "${API_AUDITOR}" "${request_echo_log}" --require-stop \
      >"${api_audit_log}" 2>&1; then
      api_audit_result="PASS"
    else
      api_audit_result="FAIL"
      status=1
    fi
  fi
  if [[ "${mode}" == "live" && -f "${gait_monitor_log}" ]]; then
    if [[ "${gait_monitor_alive_at_cleanup}" == "true" ]] && \
       rg -q 'NORMAL_GAIT_CONTRACT reason=stable' "${gait_monitor_log}" && \
       ! rg -q 'NORMAL_GAIT_CONTRACT reason=(mode_changed|gait_changed|invalid_status|time_regression|invalid_time|stale_status)' "${gait_monitor_log}"; then
      gait_audit_result="PASS"
    else
      gait_audit_result="FAIL"
      status=1
    fi
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
NORMAL_GAIT_API_AUDIT_LOG=${api_audit_log}
NORMAL_GAIT_API_AUDIT=${api_audit_result}
GAIT_MONITOR_LOG=${gait_monitor_log}
GAIT_AUDIT=${gait_audit_result}
GAIT_MONITOR_ALIVE_AT_CLEANUP=${gait_monitor_alive_at_cleanup}
EXPECTED_MODE=${expected_mode}
EXPECTED_GAIT_TYPE=${expected_gait_type}
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
    echo "NORMAL_GAIT_API_AUDIT_LOG=${api_audit_log}"
    echo "NORMAL_GAIT_API_AUDIT=${api_audit_result}"
    echo "GAIT_MONITOR_LOG=${gait_monitor_log}"
    echo "GAIT_AUDIT=${gait_audit_result}"
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
    rg 'go2_gait_state_monitor|go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | \
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

  /usr/bin/python3 "${API_AUDITOR}" "${REQUEST_ECHO_LOG}" \
    --minimum-request-id "${REQUEST_ID_BASE}" --require-move --require-stop
}

validate_gait_monitor_settings() {
  [[ -x "${GAIT_MONITOR}" ]] || die "missing executable gait monitor: ${GAIT_MONITOR}"
  [[ -x "${API_AUDITOR}" ]] || die "missing executable API auditor: ${API_AUDITOR}"
  /usr/bin/python3 - "${expected_mode}" "${expected_gait_type}" \
    "${gait_status_timeout_sec}" "${gait_monitor_rate_hz}" <<'PY'
import math
import sys

expected_mode = int(sys.argv[1])
expected_gait = int(sys.argv[2])
timeout_sec = float(sys.argv[3])
rate_hz = float(sys.argv[4])
if expected_mode < -1 or expected_gait < -1:
    raise SystemExit("expected mode/gait must be -1 or nonnegative")
if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
    raise SystemExit("GO2_GAIT_STATUS_TIMEOUT_SEC must be finite and positive")
if not math.isfinite(rate_hz) or rate_hz <= 0.0:
    raise SystemExit("GO2_GAIT_MONITOR_RATE_HZ must be finite and positive")
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
    "${DOCKER_WRAPPER}" navigation-live-source \
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

start_gait_monitor() {
  echo "GAIT_MONITOR_LOG=${gait_monitor_log}"
  /usr/bin/python3 "${GAIT_MONITOR}" \
    --ros-args \
    -p expected_mode:="${expected_mode}" \
    -p expected_gait_type:="${expected_gait_type}" \
    -p status_timeout_sec:="${gait_status_timeout_sec}" \
    -p publish_rate_hz:="${gait_monitor_rate_hz}" \
    >"${gait_monitor_log}" 2>&1 &
  gait_monitor_pid="$!"
  sleep 0.5
  if ! process_is_running "${gait_monitor_pid}"; then
    cat "${gait_monitor_log}" >&2 || true
    die "read-only gait monitor exited during startup"
  fi
}

wait_for_gait_monitor_stable() {
  local deadline=$((SECONDS + 10))
  local sample=""
  while (( SECONDS < deadline )); do
    sample="$(timeout 2 ros2 topic echo /dddmr_go2/gait_unchanged std_msgs/msg/Bool --once 2>&1 || true)"
    if rg -q '^data: true$' <<<"${sample}"; then
      echo "Normal gait baseline latched before navigation output."
      return 0
    fi
  done
  echo "${sample}" >&2
  tail -n 30 "${gait_monitor_log}" >&2 || true
  die "gait monitor did not establish a stable pre-motion baseline"
}

start_request_echo() {
  echo "REQUEST_ECHO_LOG=${request_echo_log}"
  PYTHONUNBUFFERED=1 ros2 topic echo "${REAL_REQUEST_TOPIC}" unitree_api/msg/Request \
    >"${request_echo_log}" 2>&1 &
  echo_pid="$!"
}

wait_for_live_runtime() {
  local docker_status=0
  while process_is_running "${docker_pid}"; do
    process_is_running "${adapter_pid}" || \
      die "host Sport adapter exited while navigation was active"
    process_is_running "${gait_monitor_pid}" || \
      die "read-only gait monitor exited while navigation was active"
    if rg -q 'NORMAL_GAIT_CONTRACT reason=(mode_changed|gait_changed|invalid_status|time_regression|invalid_time|stale_status)' "${gait_monitor_log}"; then
      die "normal gait contract faulted while navigation was active"
    fi
    if awk '$1 == "api_id:" && $2 != "1003" && $2 != "1008" {found=1} END {exit !found}' \
      "${request_echo_log}"; then
      die "a non-Move/StopMove Sport API was observed while navigation was active"
    fi
    sleep 0.1
  done

  set +e
  wait "${docker_pid}"
  docker_status=$?
  set -e
  docker_pid=""
  return "${docker_status}"
}

assert_no_conflicting_runtime

if [[ "${mode}" == "dry-run" ]]; then
  echo "Launching Docker Go2 XT16 navigation in dry-run RViz mode."
  exec env RVIZ="${rviz}" PUBLISH_STATIC_TF="${publish_static_tf}" \
    "${DOCKER_WRAPPER}" navigation-dry-run
fi

[[ "${GO2_NAV_LIVE_CONFIRM:-}" == "${CONFIRM_PHRASE}" ]] || \
  die "live navigation requires GO2_NAV_LIVE_CONFIRM=${CONFIRM_PHRASE}"
[[ -n "${GO2_SPORT_PROBE_SUMMARY:-}" ]] || \
  die "live navigation requires GO2_SPORT_PROBE_SUMMARY from the live adapter probe"
validate_probe_summary "${GO2_SPORT_PROBE_SUMMARY}"
validate_yaw_arc_settings
validate_gait_monitor_settings

source_host_go2_ros
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
check_topic_type "${REAL_REQUEST_TOPIC}" "unitree_api/msg/Request"
check_topic_type "/sportmodestate" "unitree_go/msg/SportModeState"
check_topic_type "/lowstate" "unitree_go/msg/LowState"

echo "LIVE NAVIGATION CAN MOVE THE GO2 AFTER RViz GOALS."
echo "Launching Docker velocity source and host Sport adapter with request_id_base=${nav_request_id_base}."
echo "Yaw arc shim: enabled=${enable_yaw_arc_shim} mode=${yaw_arc_shim_mode} allowed_decisions=${yaw_arc_allowed_decisions} decision_topic=${decision_topic}"
start_gait_monitor
wait_for_gait_monitor_stable
start_docker_source
wait_for_cmd_publisher
start_request_echo
start_adapter
live_runtime_started="true"

echo "RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING"
echo "Click only a short, nearby, clear-space RViz goal. Press Ctrl-C here to stop and send StopMove."
wait_for_live_runtime
