#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_go2_sport_adapter_pure_yaw_pair.sh --live

Run the required supervised timer-driven Go2 Sport pure-yaw pair:
  1. Move(0,0,+yaw) through go2_sport_cmd_vel_adapter at 50Hz for duration
  2. Move(0,0,-yaw) through go2_sport_cmd_vel_adapter at 50Hz for duration

This can move the Go2 through /api/sport/request. It requires:
  GO2_SPORT_PURE_YAW_PAIR_CONFIRM=I_AM_SUPERVISING_GO2_PURE_YAW_PAIR

Environment:
  GO2_SPORT_PURE_YAW_PAIR_YAW=0.25
  GO2_SPORT_PURE_YAW_PAIR_DURATION=2.0
  GO2_SPORT_PURE_YAW_PAIR_LOG_DIR=/tmp
  GO2_SPORT_PUBLISH_RATE_HZ=50.0
  GO2_SPORT_CMD_TIMEOUT_SEC=0.20
  GO2_SPORT_MAX_YAW=<defaults to GO2_SPORT_PURE_YAW_PAIR_YAW>
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh

For the fallback 0.35 rad/s test:
  GO2_SPORT_PURE_YAW_PAIR_YAW=0.35 scripts/run_go2_sport_adapter_pure_yaw_pair.sh --live

Live-mode hard caps:
  x = 0
  y = 0
  yaw <= 0.35 rad/s
  duration <= 2.0 seconds
EOF
}

mode=""
while [[ $# -gt 0 ]]; do
  case "$1" in
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

[[ "${mode}" == "live" ]] || {
  usage >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROBE="${WS_ROOT}/scripts/run_go2_sport_adapter_supervised_probe.sh"
CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_PURE_YAW_PAIR"

pair_yaw="${GO2_SPORT_PURE_YAW_PAIR_YAW:-0.25}"
pair_duration="${GO2_SPORT_PURE_YAW_PAIR_DURATION:-2.0}"
log_dir="${GO2_SPORT_PURE_YAW_PAIR_LOG_DIR:-/tmp}"
publish_rate_hz="${GO2_SPORT_PUBLISH_RATE_HZ:-50.0}"
cmd_timeout_sec="${GO2_SPORT_CMD_TIMEOUT_SEC:-0.20}"
max_yaw="${GO2_SPORT_MAX_YAW:-${pair_yaw}}"
stamp="$(date +%Y%m%d_%H%M%S)"
pair_report="${log_dir}/go2_sport_pure_yaw_pair_${stamp}.env"
pos_output_log="${log_dir}/go2_sport_pure_yaw_pair_${stamp}_positive.log"
neg_output_log="${log_dir}/go2_sport_pure_yaw_pair_${stamp}_negative.log"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

validate_settings() {
  [[ "${GO2_SPORT_PURE_YAW_PAIR_CONFIRM:-}" == "${CONFIRM_PHRASE}" ]] || \
    die "live pure-yaw pair requires GO2_SPORT_PURE_YAW_PAIR_CONFIRM=${CONFIRM_PHRASE}"
  [[ -x "${PROBE}" ]] || die "missing executable: ${PROBE}"

  /usr/bin/python3 - "${pair_yaw}" "${pair_duration}" "${publish_rate_hz}" "${cmd_timeout_sec}" "${max_yaw}" <<'PY'
import math
import sys

yaw = float(sys.argv[1])
duration = float(sys.argv[2])
publish_rate_hz = float(sys.argv[3])
cmd_timeout_sec = float(sys.argv[4])
max_yaw = float(sys.argv[5])
if not all(math.isfinite(v) for v in (yaw, duration, publish_rate_hz, cmd_timeout_sec, max_yaw)):
    raise SystemExit("yaw, duration, publish_rate_hz, cmd_timeout_sec, and max_yaw must be finite")
if yaw <= 0.0 or yaw > 0.35:
    raise SystemExit("GO2_SPORT_PURE_YAW_PAIR_YAW must be within (0.0, 0.35]")
if duration <= 0.0 or duration > 2.0:
    raise SystemExit("GO2_SPORT_PURE_YAW_PAIR_DURATION must be within (0.0, 2.0]")
if publish_rate_hz <= 0.0:
    raise SystemExit("GO2_SPORT_PUBLISH_RATE_HZ must be > 0")
if cmd_timeout_sec <= 0.0:
    raise SystemExit("GO2_SPORT_CMD_TIMEOUT_SEC must be > 0")
if max_yaw < yaw or max_yaw > 0.35:
    raise SystemExit("GO2_SPORT_MAX_YAW must be >= pair yaw and <= 0.35")
PY
}

extract_summary_log() {
  local output_log="$1"
  awk -F= '$1 == "SUMMARY_LOG" {value=$2} END {print value}' "${output_log}"
}

run_one_probe() {
  local label="$1"
  local yaw="$2"
  local output_log="$3"
  local status
  local summary_log

  echo "=== ${label} timer-driven pure-yaw probe yaw=${yaw}"
  set +e
  GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
  GO2_SPORT_PROBE_X=0.0 \
  GO2_SPORT_PROBE_Y=0.0 \
  GO2_SPORT_PROBE_YAW="${yaw}" \
  GO2_SPORT_PROBE_DURATION="${pair_duration}" \
  GO2_SPORT_PROBE_LOG_DIR="${log_dir}" \
  GO2_SPORT_PUBLISH_RATE_HZ="${publish_rate_hz}" \
  GO2_SPORT_CMD_TIMEOUT_SEC="${cmd_timeout_sec}" \
  GO2_SPORT_MAX_X=0.10 \
  GO2_SPORT_MAX_Y=0.0 \
  GO2_SPORT_MAX_YAW="${max_yaw}" \
    "${PROBE}" --live >"${output_log}" 2>&1
  status=$?
  set -e

  cat "${output_log}"
  summary_log="$(extract_summary_log "${output_log}")"
  {
    echo "${label^^}_STATUS=${status}"
    echo "${label^^}_YAW=${yaw}"
    echo "${label^^}_OUTPUT_LOG=${output_log}"
    echo "${label^^}_SUMMARY_LOG=${summary_log}"
  } >>"${pair_report}"

  if [[ "${status}" -ne 0 ]]; then
    die "${label} pure-yaw probe failed; output=${output_log}"
  fi
  [[ -n "${summary_log}" && -f "${summary_log}" ]] || \
    die "${label} pure-yaw probe did not produce SUMMARY_LOG"
}

validate_settings

cat >"${pair_report}" <<EOF
RESULT=GO2_SPORT_PURE_YAW_PAIR_STARTED
STAMP=${stamp}
YAW=${pair_yaw}
DURATION=${pair_duration}
PUBLISH_RATE_HZ=${publish_rate_hz}
CMD_TIMEOUT_SEC=${cmd_timeout_sec}
MAX_YAW=${max_yaw}
POS_OUTPUT_LOG=${pos_output_log}
NEG_OUTPUT_LOG=${neg_output_log}
EOF

echo "PAIR_REPORT=${pair_report}"
echo "This supervised pure-yaw pair can move the Go2. Confirmation accepted."
run_one_probe "positive" "${pair_yaw}" "${pos_output_log}"
run_one_probe "negative" "-${pair_yaw}" "${neg_output_log}"

{
  echo "RESULT=GO2_SPORT_PURE_YAW_PAIR_PASS"
} >>"${pair_report}"

echo "PAIR_REPORT=${pair_report}"
echo "RESULT: GO2_SPORT_PURE_YAW_PAIR_PASS"
