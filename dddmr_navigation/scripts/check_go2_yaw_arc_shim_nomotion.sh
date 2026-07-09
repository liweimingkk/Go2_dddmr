#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/check_go2_yaw_arc_shim_nomotion.sh

No-motion verifier for the Go2 Sport yaw-arc shim.

It runs three host-side adapter dry-run probes and checks the adapter logs:

  1. d_align_heading transforms yaw-only into the configured small forward arc.
  2. d_recovery_waitdone blocks yaw-only forward injection and logs StopMove.
  3. stale d_align_heading blocks yaw-only forward injection and logs StopMove.

This script does not publish /api/sport/request and does not require live Go2
topic discovery. It still sources the local Unitree ROS2 setup so the ROS2
Python environment matches the real adapter runtime.

Environment:
  GO2_YAW_ARC_NOMOTION_LOG_DIR=/tmp
  GO2_YAW_ARC_NOMOTION_YAW=-0.15
  GO2_YAW_ARC_NOMOTION_FORWARD_X=0.03
  GO2_YAW_ARC_NOMOTION_MIN_ABS_YAW=0.20
  GO2_YAW_ARC_ALLOWED_DECISIONS=d_align_heading
  GO2_MAX_CONTINUOUS_YAW_ARC_SEC=4.0
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
PROBE="${WS_ROOT}/scripts/run_go2_sport_adapter_supervised_probe.sh"
LOG_DIR="${GO2_YAW_ARC_NOMOTION_LOG_DIR:-/tmp}"
YAW="${GO2_YAW_ARC_NOMOTION_YAW:--0.15}"
FORWARD_X="${GO2_YAW_ARC_NOMOTION_FORWARD_X:-0.03}"
MIN_ABS_YAW="${GO2_YAW_ARC_NOMOTION_MIN_ABS_YAW:-0.20}"
ALLOWED_DECISIONS="${GO2_YAW_ARC_ALLOWED_DECISIONS:-d_align_heading}"
MAX_CONTINUOUS_YAW_ARC_SEC="${GO2_MAX_CONTINUOUS_YAW_ARC_SEC:-4.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="${LOG_DIR}/go2_yaw_arc_shim_nomotion_${STAMP}.env"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

validate_settings() {
  [[ ",${ALLOWED_DECISIONS}," == *",d_align_heading,"* ]] || \
    die "GO2_YAW_ARC_ALLOWED_DECISIONS must include d_align_heading for this verifier"
}

expected_payload() {
  /usr/bin/python3 - "${FORWARD_X}" "${YAW}" "${MIN_ABS_YAW}" <<'PY'
import json
import math
import sys

x = float(sys.argv[1])
yaw = float(sys.argv[2])
min_abs_yaw = float(sys.argv[3])
shim_yaw = math.copysign(max(abs(yaw), min_abs_yaw), yaw)
print(json.dumps({"x": x, "y": 0.0, "z": shim_yaw}, separators=(",", ":")))
PY
}

original_payload() {
  /usr/bin/python3 - "${YAW}" <<'PY'
import json
import sys

yaw = float(sys.argv[1])
print(json.dumps({"x": 0.0, "y": 0.0, "z": yaw}, separators=(",", ":")))
PY
}

assert_file_contains() {
  local file="$1"
  local pattern="$2"
  local label="$3"
  if ! rg -q --fixed-strings "${pattern}" "${file}"; then
    echo "=== ${file}" >&2
    sed -n '1,220p' "${file}" >&2 || true
    die "${label}: missing pattern: ${pattern}"
  fi
}

run_probe() {
  local label="$1"
  local decision="$2"
  local decision_count="$3"
  local pre_cmd_sleep="$4"
  local output_file
  output_file="${LOG_DIR}/go2_yaw_arc_shim_nomotion_${STAMP}_${label}.out"

  echo "=== ${label}"
  GO2_SPORT_SKIP_LIVE_TOPIC_CHECK=true \
  GO2_ENABLE_YAW_ARC_SHIM=true \
  GO2_YAW_ARC_SHIM_MODE=live \
  GO2_YAW_ARC_SHIM_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_SHIM \
  GO2_YAW_ARC_FORWARD_X="${FORWARD_X}" \
  GO2_YAW_ARC_MIN_ABS_YAW="${MIN_ABS_YAW}" \
  GO2_YAW_ARC_ALLOWED_DECISIONS="${ALLOWED_DECISIONS}" \
  GO2_MAX_CONTINUOUS_YAW_ARC_SEC="${MAX_CONTINUOUS_YAW_ARC_SEC}" \
  GO2_SPORT_PROBE_X=0.0 \
  GO2_SPORT_PROBE_Y=0.0 \
  GO2_SPORT_PROBE_YAW="${YAW}" \
  GO2_SPORT_PROBE_DECISION="${decision}" \
  GO2_SPORT_PROBE_DECISION_PUB_COUNT="${decision_count}" \
  GO2_SPORT_PROBE_PRE_CMD_SLEEP="${pre_cmd_sleep}" \
  GO2_SPORT_PROBE_LOG_DIR="${LOG_DIR}" \
    "${PROBE}" --dry-run >"${output_file}" 2>&1

  local summary
  summary="$(rg -o 'SUMMARY_LOG=.*' "${output_file}" | tail -n 1 | cut -d= -f2-)"
  [[ -n "${summary}" && -f "${summary}" ]] || {
    sed -n '1,220p' "${output_file}" >&2 || true
    die "${label}: missing SUMMARY_LOG"
  }

  # shellcheck disable=SC1090
  source "${summary}"
  [[ "${MODE:-}" == "dry-run" ]] || die "${label}: MODE is not dry-run"
  [[ "${RESULT:-}" == "GO2_SPORT_ADAPTER_dry_run_COMPLETE" ]] || die "${label}: probe did not complete"
  [[ -f "${ADAPTER_LOG:-}" ]] || die "${label}: missing adapter log"

  printf '%s\n' "${summary}"
}

validate_settings
expected_transform="$(expected_payload)"
expected_original="$(original_payload)"

allowed_summary="$(run_probe allowed d_align_heading 42 0.0)"
allowed_summary="$(printf '%s\n' "${allowed_summary}" | tail -n 1)"
# shellcheck disable=SC1090
source "${allowed_summary}"
allowed_log="${ADAPTER_LOG}"
assert_file_contains "${allowed_log}" \
  "GATED would publish /api/sport/request: api_id=1008 parameter=${expected_transform}" \
  "allowed"
assert_file_contains "${allowed_log}" \
  "decision=d_align_heading original_sport=${expected_original} transformed_sport=${expected_transform}" \
  "allowed"

recovery_summary="$(run_probe recovery d_recovery_waitdone 42 0.0)"
recovery_summary="$(printf '%s\n' "${recovery_summary}" | tail -n 1)"
# shellcheck disable=SC1090
source "${recovery_summary}"
recovery_log="${ADAPTER_LOG}"
assert_file_contains "${recovery_log}" \
  "zero cmd_vel decision=d_recovery_waitdone original_sport=${expected_original} transformed_sport={\"x\":0.0,\"y\":0.0,\"z\":0.0}" \
  "recovery"
assert_file_contains "${recovery_log}" \
  'shim=blocked_blocked_state=d_recovery_waitdone' \
  "recovery"
assert_file_contains "${recovery_log}" \
  'api_id=1003 StopMove' \
  "recovery"

stale_summary="$(run_probe stale d_align_heading 1 0.8)"
stale_summary="$(printf '%s\n' "${stale_summary}" | tail -n 1)"
# shellcheck disable=SC1090
source "${stale_summary}"
stale_log="${ADAPTER_LOG}"
assert_file_contains "${stale_log}" \
  "zero cmd_vel decision=d_align_heading original_sport=${expected_original} transformed_sport={\"x\":0.0,\"y\":0.0,\"z\":0.0}" \
  "stale"
assert_file_contains "${stale_log}" \
  'shim=blocked_stale_decision=d_align_heading' \
  "stale"
assert_file_contains "${stale_log}" \
  'api_id=1003 StopMove' \
  "stale"

cat >"${REPORT}" <<EOF
RESULT=GO2_YAW_ARC_SHIM_NOMOTION_PASS
YAW=${YAW}
FORWARD_X=${FORWARD_X}
MIN_ABS_YAW=${MIN_ABS_YAW}
ALLOWED_DECISIONS=${ALLOWED_DECISIONS}
MAX_CONTINUOUS_YAW_ARC_SEC=${MAX_CONTINUOUS_YAW_ARC_SEC}
ALLOWED_SUMMARY=${allowed_summary}
ALLOWED_LOG=${allowed_log}
RECOVERY_SUMMARY=${recovery_summary}
RECOVERY_LOG=${recovery_log}
STALE_SUMMARY=${stale_summary}
STALE_LOG=${stale_log}
EOF

echo "REPORT=${REPORT}"
echo "RESULT: GO2_YAW_ARC_SHIM_NOMOTION_PASS"
