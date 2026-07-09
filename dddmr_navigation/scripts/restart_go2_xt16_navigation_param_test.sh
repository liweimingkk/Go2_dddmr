#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  GO2_PARAM_TEST_CONFIRM=I_AM_SUPERVISING_GO2_PARAM_TEST \
    scripts/restart_go2_xt16_navigation_param_test.sh [options]

Purpose:
  Stop the previous Go2 XT16 navigation runtime, run a fresh live Sport probe,
  then start a new navigation/RViz runtime for parameter testing.

Options:
  --live                 Start supervised live navigation after cleanup. Default.
  --dry-run              Start navigation dry-run after cleanup; no /api/sport/request output.
  --stop-only            Stop old navigation runtime and exit.
  --skip-readiness       Skip the read-only live readiness gate.
  --skip-probe           Reuse GO2_SPORT_PROBE_SUMMARY instead of running a fresh probe.
  --probe-summary PATH   Reuse this live probe summary for --skip-probe.
  --no-rviz              Start without RViz.
  --rviz true|false      Explicit RViz setting. Default: true.
  --max-x VALUE          Navigation Sport max_x. Default: 0.30.
  --max-y VALUE          Navigation Sport max_y. Default: 0.0.
  --max-yaw VALUE        Navigation Sport max_yaw. Default: 0.25.
  --log-dir PATH         Runtime log directory. Default: run_logs.
  -h, --help             Show this help.

Environment:
  GO2_PARAM_TEST_CONFIRM=I_AM_SUPERVISING_GO2_PARAM_TEST is required for --live.
  GO2_SPORT_MAX_X/Y/YAW can also set the navigation limits, unless overridden
  by --max-x/--max-y/--max-yaw.

  The live probe is intentionally capped separately:
    GO2_SPORT_PROBE_MAX_X=0.10
    GO2_SPORT_PROBE_MAX_Y=0.0
    GO2_SPORT_PROBE_MAX_YAW=<navigation max_yaw>

Examples:
  # Normal supervised parameter-test restart at max_x=0.30.
  GO2_PARAM_TEST_CONFIRM=I_AM_SUPERVISING_GO2_PARAM_TEST \
    scripts/restart_go2_xt16_navigation_param_test.sh

  # Test a different speed cap.
  GO2_PARAM_TEST_CONFIRM=I_AM_SUPERVISING_GO2_PARAM_TEST \
    scripts/restart_go2_xt16_navigation_param_test.sh --max-x 0.25

  # If you edited YAML parameters such as inflation_radius, just rerun this.
  GO2_PARAM_TEST_CONFIRM=I_AM_SUPERVISING_GO2_PARAM_TEST \
    scripts/restart_go2_xt16_navigation_param_test.sh

  # Stop only.
  scripts/restart_go2_xt16_navigation_param_test.sh --stop-only
EOF
}

mode="live"
stop_only="false"
run_readiness="true"
run_probe="true"
rviz="${RVIZ:-true}"
nav_max_x="${GO2_SPORT_MAX_X:-0.30}"
nav_max_y="${GO2_SPORT_MAX_Y:-0.0}"
nav_max_yaw="${GO2_SPORT_MAX_YAW:-0.25}"
probe_summary="${GO2_SPORT_PROBE_SUMMARY:-}"
log_dir="${GO2_PARAM_TEST_LOG_DIR:-${GO2_NAV_LOG_DIR:-}}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --live)
      mode="live"
      shift
      ;;
    --dry-run)
      mode="dry-run"
      shift
      ;;
    --stop-only)
      stop_only="true"
      shift
      ;;
    --skip-readiness)
      run_readiness="false"
      shift
      ;;
    --skip-probe)
      run_probe="false"
      shift
      ;;
    --probe-summary)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --probe-summary requires a path" >&2
        exit 2
      }
      probe_summary="$2"
      run_probe="false"
      shift 2
      ;;
    --no-rviz)
      rviz="false"
      shift
      ;;
    --rviz)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --rviz requires true or false" >&2
        exit 2
      }
      rviz="$2"
      shift 2
      ;;
    --max-x)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --max-x requires a value" >&2
        exit 2
      }
      nav_max_x="$2"
      shift 2
      ;;
    --max-y)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --max-y requires a value" >&2
        exit 2
      }
      nav_max_y="$2"
      shift 2
      ;;
    --max-yaw)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --max-yaw requires a value" >&2
        exit 2
      }
      nav_max_yaw="$2"
      shift 2
      ;;
    --log-dir)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --log-dir requires a path" >&2
        exit 2
      }
      log_dir="$2"
      shift 2
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SPORT_PROBE="${WS_ROOT}/scripts/run_go2_sport_adapter_supervised_probe.sh"
NAV_LIVE="${WS_ROOT}/scripts/run_go2_xt16_navigation_supervised_live.sh"
READINESS="${WS_ROOT}/scripts/check_go2_xt16_sport_live_readiness.sh"
CLEAN_CHECK="${WS_ROOT}/scripts/check_go2_xt16_no_motion_runtime_clean.sh"
log_dir="${log_dir:-${WS_ROOT}/run_logs}"
stamp="$(date +%Y%m%d_%H%M%S)"
probe_stdout_log="${log_dir}/go2_xt16_param_restart_${stamp}_probe_stdout.log"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

info() {
  echo "[$(date +%H:%M:%S)] $*"
}

require_executable() {
  local path="$1"
  [[ -x "${path}" ]] || die "missing executable: ${path}"
}

validate_bool() {
  local name="$1"
  local value="$2"
  case "${value}" in
    true|false) ;;
    *) die "${name} must be true or false, got: ${value}" ;;
  esac
}

validate_numeric_limits() {
  /usr/bin/python3 - "${nav_max_x}" "${nav_max_y}" "${nav_max_yaw}" <<'PY'
import math
import sys

names = ("GO2_SPORT_MAX_X", "GO2_SPORT_MAX_Y", "GO2_SPORT_MAX_YAW")
limits = ((0.0, 0.30), (0.0, 0.05), (0.0, 0.35))
for name, raw, (low, high) in zip(names, sys.argv[1:], limits):
    try:
        value = float(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be numeric: {exc}")
    if not math.isfinite(value):
        raise SystemExit(f"{name} must be finite")
    if value < low or value > high:
        raise SystemExit(f"{name}={value} outside [{low}, {high}]")
PY
}

live_runner_pids() {
  ps -eo pid=,args= | awk '
    /run_go2_xt16_navigation_supervised_live[.]sh --live/ &&
    !/restart_go2_xt16_navigation_param_test[.]sh/ {
      print $1
    }
  '
}

residual_process_pids() {
  ps -eo pid=,args= | awk '
    (
      /go2_sport_cmd_vel_adapter[.]py/ ||
      /go2_sport_cmd_vel_dry_run[.]py/ ||
      /go2_xt16_navigation[.]launch/ ||
      /go2_xt16_navigation[.]rviz/ ||
      /p2p_move_base_node/ ||
      /global_planner_node/ ||
      /dddmr_pg_map_server_node/ ||
      /ros2 topic echo \/api\/sport\/request/ ||
      /ros2 topic pub .*\/dddmr_go2\/safe_cmd_vel/
    ) &&
    !/restart_go2_xt16_navigation_param_test[.]sh/ &&
    !/check_go2_xt16_no_motion_runtime_clean[.]sh/ &&
    !/check_go2_xt16_sport_live_readiness[.]sh/ &&
    !/awk/ {
      print $1
    }
  '
}

nav_container_names() {
  docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | awk '
    $1 ~ /^go2_xt16_nav_/ || $1 ~ /^dddmr_go2_xt16_nav_/ || $1 ~ /^go2_xt16_mapping_/ {
      print $1
    }
  '
}

wait_until_gone() {
  local timeout_sec="$1"
  shift
  local deadline=$((SECONDS + timeout_sec))
  local pid

  while (( SECONDS < deadline )); do
    local any_alive="false"
    for pid in "$@"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        any_alive="true"
        break
      fi
    done
    [[ "${any_alive}" == "false" ]] && return 0
    sleep 1
  done
  return 1
}

stop_existing_runtime() {
  local pids
  mapfile -t pids < <(live_runner_pids)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    info "stopping old supervised live runner(s): ${pids[*]}"
    kill -INT "${pids[@]}" >/dev/null 2>&1 || true
    if ! wait_until_gone 25 "${pids[@]}"; then
      info "old runner did not exit after SIGINT; sending SIGTERM"
      kill -TERM "${pids[@]}" >/dev/null 2>&1 || true
      wait_until_gone 10 "${pids[@]}" || die "old live runner is still alive: ${pids[*]}"
    fi
  else
    info "no old supervised live runner found"
  fi

  local containers
  mapfile -t containers < <(nav_container_names)
  if [[ "${#containers[@]}" -gt 0 ]]; then
    info "stopping residual Docker container(s): ${containers[*]}"
    docker stop "${containers[@]}" >/dev/null
  fi

  local residuals
  mapfile -t residuals < <(residual_process_pids)
  if [[ "${#residuals[@]}" -gt 0 ]]; then
    info "stopping residual nav/adapter/RViz process(es): ${residuals[*]}"
    kill -INT "${residuals[@]}" >/dev/null 2>&1 || true
    if ! wait_until_gone 10 "${residuals[@]}"; then
      kill -TERM "${residuals[@]}" >/dev/null 2>&1 || true
      wait_until_gone 10 "${residuals[@]}" || die "residual process is still alive: ${residuals[*]}"
    fi
  fi

  info "checking runtime is clean"
  "${CLEAN_CHECK}"
}

run_live_readiness() {
  info "running read-only Sport live readiness gate"
  env \
    -u GO2_SPORT_LIVE_CONFIRM \
    -u GO2_NAV_LIVE_CONFIRM \
    -u GO2_SPORT_PROBE_SUMMARY \
    "${READINESS}"
}

run_live_probe() {
  local probe_max_x="${GO2_SPORT_PROBE_MAX_X:-0.10}"
  local probe_max_y="${GO2_SPORT_PROBE_MAX_Y:-0.0}"
  local probe_max_yaw="${GO2_SPORT_PROBE_MAX_YAW:-${nav_max_yaw}}"

  info "running fresh supervised live Sport probe"
  env \
    GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
    GO2_SPORT_PROBE_LOG_DIR="${log_dir}" \
    GO2_SPORT_MAX_X="${probe_max_x}" \
    GO2_SPORT_MAX_Y="${probe_max_y}" \
    GO2_SPORT_MAX_YAW="${probe_max_yaw}" \
    GO2_SPORT_PUBLISH_RATE_HZ="${GO2_SPORT_PUBLISH_RATE_HZ:-50.0}" \
    GO2_SPORT_CMD_TIMEOUT_SEC="${GO2_SPORT_CMD_TIMEOUT_SEC:-0.20}" \
    GO2_SPORT_ZERO_EPSILON="${GO2_SPORT_ZERO_EPSILON:-0.001}" \
    GO2_SPORT_STOP_KEEPALIVE_HZ="${GO2_SPORT_STOP_KEEPALIVE_HZ:-2.0}" \
    GO2_ENABLE_YAW_ARC_SHIM="${GO2_ENABLE_YAW_ARC_SHIM:-false}" \
    GO2_YAW_ARC_SHIM_MODE="${GO2_YAW_ARC_SHIM_MODE:-off}" \
    GO2_YAW_ARC_FORWARD_X="${GO2_YAW_ARC_FORWARD_X:-0.03}" \
    GO2_YAW_ARC_MIN_ABS_YAW="${GO2_YAW_ARC_MIN_ABS_YAW:-0.20}" \
    GO2_YAW_ARC_TRIGGER_ABS_YAW="${GO2_YAW_ARC_TRIGGER_ABS_YAW:-0.03}" \
    GO2_YAW_ARC_ALLOWED_DECISIONS="${GO2_YAW_ARC_ALLOWED_DECISIONS:-d_align_heading}" \
    GO2_DECISION_TOPIC="${GO2_DECISION_TOPIC:-/dddmr_go2/p2p_decision}" \
    GO2_DECISION_TIMEOUT_SEC="${GO2_DECISION_TIMEOUT_SEC:-0.30}" \
    GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED="${GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED:-true}" \
    GO2_MAX_CONTINUOUS_YAW_ARC_SEC="${GO2_MAX_CONTINUOUS_YAW_ARC_SEC:-4.0}" \
    "${SPORT_PROBE}" --live | tee "${probe_stdout_log}"

  probe_summary="$(awk -F= '$1=="SUMMARY_LOG"{print $2}' "${probe_stdout_log}" | tail -n 1)"
  [[ -n "${probe_summary}" ]] || die "live probe did not print SUMMARY_LOG"
  [[ -f "${probe_summary}" ]] || die "live probe summary does not exist: ${probe_summary}"
  info "live probe summary: ${probe_summary}"
}

assert_live_confirmation() {
  [[ "${GO2_PARAM_TEST_CONFIRM:-}" == "I_AM_SUPERVISING_GO2_PARAM_TEST" ]] || \
    die "live restart requires GO2_PARAM_TEST_CONFIRM=I_AM_SUPERVISING_GO2_PARAM_TEST"
}

mkdir -p "${log_dir}"
require_executable "${NAV_LIVE}"
require_executable "${SPORT_PROBE}"
require_executable "${READINESS}"
require_executable "${CLEAN_CHECK}"
if [[ "${stop_only}" != "true" ]]; then
  validate_bool "RVIZ" "${rviz}"
  if [[ "${mode}" == "live" ]]; then
    validate_numeric_limits
  fi
fi

info "mode=${mode} stop_only=${stop_only} rviz=${rviz} max_x=${nav_max_x} max_y=${nav_max_y} max_yaw=${nav_max_yaw}"
if [[ "${mode}" == "live" && "${stop_only}" != "true" ]]; then
  assert_live_confirmation
fi
stop_existing_runtime

if [[ "${stop_only}" == "true" ]]; then
  info "stop-only complete"
  exit 0
fi

case "${mode}" in
  dry-run)
    info "starting fresh navigation dry-run"
    exec env \
      RVIZ="${rviz}" \
      PUBLISH_STATIC_TF="${PUBLISH_STATIC_TF:-true}" \
      "${NAV_LIVE}" --dry-run
    ;;

  live)
    if [[ "${run_readiness}" == "true" ]]; then
      run_live_readiness
    else
      info "skipping readiness gate by request"
    fi

    if [[ "${run_probe}" == "true" ]]; then
      run_live_probe
    else
      [[ -n "${probe_summary}" ]] || die "--skip-probe requires GO2_SPORT_PROBE_SUMMARY or --probe-summary PATH"
      [[ -f "${probe_summary}" ]] || die "probe summary does not exist: ${probe_summary}"
      info "reusing live probe summary: ${probe_summary}"
    fi

    info "starting fresh supervised live navigation"
    exec env \
      GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
      GO2_SPORT_PROBE_SUMMARY="${probe_summary}" \
      GO2_NAV_LOG_DIR="${log_dir}" \
      RVIZ="${rviz}" \
      PUBLISH_STATIC_TF="${PUBLISH_STATIC_TF:-true}" \
      GO2_SPORT_MAX_X="${nav_max_x}" \
      GO2_SPORT_MAX_Y="${nav_max_y}" \
      GO2_SPORT_MAX_YAW="${nav_max_yaw}" \
      GO2_SPORT_PUBLISH_RATE_HZ="${GO2_SPORT_PUBLISH_RATE_HZ:-50.0}" \
      GO2_SPORT_CMD_TIMEOUT_SEC="${GO2_SPORT_CMD_TIMEOUT_SEC:-0.20}" \
      GO2_SPORT_ZERO_EPSILON="${GO2_SPORT_ZERO_EPSILON:-0.001}" \
      GO2_SPORT_STOP_KEEPALIVE_HZ="${GO2_SPORT_STOP_KEEPALIVE_HZ:-2.0}" \
      GO2_ENABLE_YAW_ARC_SHIM="${GO2_ENABLE_YAW_ARC_SHIM:-false}" \
      GO2_YAW_ARC_SHIM_MODE="${GO2_YAW_ARC_SHIM_MODE:-off}" \
      GO2_YAW_ARC_SHIM_CONFIRM="${GO2_YAW_ARC_SHIM_CONFIRM:-}" \
      GO2_YAW_ARC_FORWARD_X="${GO2_YAW_ARC_FORWARD_X:-0.03}" \
      GO2_YAW_ARC_MIN_ABS_YAW="${GO2_YAW_ARC_MIN_ABS_YAW:-0.20}" \
      GO2_YAW_ARC_TRIGGER_ABS_YAW="${GO2_YAW_ARC_TRIGGER_ABS_YAW:-0.03}" \
      GO2_YAW_ARC_ALLOWED_DECISIONS="${GO2_YAW_ARC_ALLOWED_DECISIONS:-d_align_heading}" \
      GO2_YAW_ARC_NOMOTION_REPORT="${GO2_YAW_ARC_NOMOTION_REPORT:-}" \
      GO2_DECISION_TOPIC="${GO2_DECISION_TOPIC:-/dddmr_go2/p2p_decision}" \
      GO2_DECISION_TIMEOUT_SEC="${GO2_DECISION_TIMEOUT_SEC:-0.30}" \
      GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED="${GO2_ZERO_YAW_ONLY_WHEN_SHIM_DISALLOWED:-true}" \
      GO2_MAX_CONTINUOUS_YAW_ARC_SEC="${GO2_MAX_CONTINUOUS_YAW_ARC_SEC:-4.0}" \
      "${NAV_LIVE}" --live
    ;;

  *)
    die "unknown mode: ${mode}"
    ;;
esac
