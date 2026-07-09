#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh [--evidence FILE]
  scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh --self-test

Print a local, no-motion summary of the remaining Go2 XT16 axis/tilt acceptance
gate evidence gaps. This is a checklist helper only; the acceptance verifier is
still the authority for PASS/NOT_READY.

This script parses KEY=VALUE files without sourcing them. It does not source
ROS, inspect live robot topics, start Docker, publish /cmd_vel, publish
/dddmr_go2/dry_run_cmd_vel, publish /api/sport/request, or publish /lowcmd.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
evidence_file="${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env"
self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --evidence)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --evidence requires a file" >&2
        exit 2
      }
      evidence_file="$2"
      shift 2
      ;;
    --self-test)
      self_test=true
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

die() {
  echo "ERROR: $*" >&2
  exit 2
}

get_value_from_file() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" '
    BEGIN { FS = "=" }
    /^[[:space:]]*#/ { next }
    /^[[:space:]]*$/ { next }
    {
      lhs = $1
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", lhs)
      if (lhs == key) {
        sub(/^[^=]*=/, "")
        gsub(/\r$/, "")
        print
        found = 1
        exit
      }
    }
    END {
      if (!found) {
        exit 1
      }
    }
  ' "${file}" || true
}

get_value() {
  get_value_from_file "${evidence_file}" "$1"
}

join_by_comma() {
  if [[ $# -eq 0 ]]; then
    printf 'none'
  else
    local IFS=,
    printf '%s' "$*"
  fi
}

add_gap_if_not_equal() {
  local -n gaps_ref="$1"
  local key="$2"
  local expected="$3"
  local value
  value="$(get_value "${key}")"
  [[ "${value}" == "${expected}" ]] || gaps_ref+=("${key}")
}

path_gap_reason() {
  local path="$1"
  if [[ -z "${path}" || "${path}" == "TODO" || "${path}" == "<path>" ]]; then
    printf 'missing'
  elif [[ "${path}" != /* ]]; then
    printf 'not_absolute'
  elif [[ ! -s "${path}" ]]; then
    printf 'missing_or_empty_file'
  else
    printf 'ok'
  fi
}

file_key_gap_reason() {
  local key="$1"
  local path
  path="$(get_value "${key}")"
  path_gap_reason "${path}"
}

image_key_gap_reason() {
  local key="$1"
  local path
  local reason
  local signature
  path="$(get_value "${key}")"
  reason="$(path_gap_reason "${path}")"
  if [[ "${reason}" != "ok" ]]; then
    printf '%s' "${reason}"
    return
  fi
  signature="$(LC_ALL=C head -c 12 "${path}" | od -An -tx1 | tr -d ' \n')"
  case "${signature}" in
    89504e470d0a1a0a*|ffd8ff*) printf 'ok' ;;
    *) printf 'not_png_or_jpeg' ;;
  esac
}

add_file_key_gap() {
  local -n gaps_ref="$1"
  local key="$2"
  local reason
  reason="$(file_key_gap_reason "${key}")"
  [[ "${reason}" == "ok" ]] || gaps_ref+=("${key}:${reason}")
}

add_image_key_gap() {
  local -n gaps_ref="$1"
  local key="$2"
  local reason
  reason="$(image_key_gap_reason "${key}")"
  [[ "${reason}" == "ok" ]] || gaps_ref+=("${key}:${reason}")
}

add_file_contains_gap() {
  local -n gaps_ref="$1"
  local key="$2"
  local text="$3"
  local label="$4"
  local path
  path="$(get_value "${key}")"
  [[ "$(path_gap_reason "${path}")" == "ok" ]] || return 0
  grep -Fq -- "${text}" "${path}" || gaps_ref+=("${label}")
}

add_summary_equal_gap() {
  local -n gaps_ref="$1"
  local summary_key="$2"
  local value_key="$3"
  local expected="$4"
  local label="$5"
  local path
  local actual
  path="$(get_value "${summary_key}")"
  [[ "$(path_gap_reason "${path}")" == "ok" ]] || return 0
  actual="$(get_value_from_file "${path}" "${value_key}")"
  [[ "${actual}" == "${expected}" ]] || gaps_ref+=("${label}")
}

add_summary_numeric_cap_gap() {
  local -n gaps_ref="$1"
  local summary_key="$2"
  local value_key="$3"
  local limit="$4"
  local label="$5"
  local path
  local value
  path="$(get_value "${summary_key}")"
  [[ "$(path_gap_reason "${path}")" == "ok" ]] || return 0
  value="$(get_value_from_file "${path}" "${value_key}")"
  python3 - "${value}" "${limit}" <<'PY' || gaps_ref+=("${label}")
import math
import sys

raw, limit_raw = sys.argv[1:]
try:
    value = float(raw)
    limit = float(limit_raw)
except ValueError:
    raise SystemExit(1)
if not math.isfinite(value) or not math.isfinite(limit):
    raise SystemExit(1)
if value < 0.0 or value > limit:
    raise SystemExit(1)
PY
}

add_live_speed_gap() {
  local -n gaps_ref="$1"
  local value
  value="$(get_value LIVE_NAV_MAX_X_MPS)"
  python3 - "${value}" <<'PY' || gaps_ref+=("LIVE_NAV_MAX_X_MPS")
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
if not math.isfinite(value) or value < 0.0 or value > 0.30:
    raise SystemExit(1)
PY
}

add_sport_probe_echo_gaps() {
  local -n gaps_ref="$1"
  local summary
  local echo_log
  local reason
  summary="$(get_value SPORT_PROBE_SUMMARY)"
  [[ "$(path_gap_reason "${summary}")" == "ok" ]] || return 0
  echo_log="$(get_value_from_file "${summary}" REQUEST_ECHO_LOG)"
  reason="$(path_gap_reason "${echo_log}")"
  if [[ "${reason}" != "ok" ]]; then
    gaps_ref+=("REQUEST_ECHO_LOG:${reason}")
    return
  fi
  grep -Eq 'api_id:[[:space:]]*1008' "${echo_log}" || gaps_ref+=("REQUEST_ECHO_LOG:missing_api_1008")
  grep -Eq 'api_id:[[:space:]]*1003' "${echo_log}" || gaps_ref+=("REQUEST_ECHO_LOG:missing_api_1003")
}

print_gate() {
  local gate="$1"
  shift
  local gaps=("$@")
  local status="PASS"
  if [[ ${#gaps[@]} -gt 0 ]]; then
    status="NOT_READY"
  fi
  echo "GO2_XT16_GATE_GAPS_${gate}_STATUS=${status}"
  echo "GO2_XT16_GATE_GAPS_${gate}_MISSING=$(join_by_comma "${gaps[@]}")"
}

summarize_gaps() {
  [[ -f "${evidence_file}" ]] || die "missing evidence file: ${evidence_file}"

  local gate1_gaps=()
  local gate2_gaps=()
  local gate3_gaps=()
  local gate4_gaps=()
  local remaining=()

  add_gap_if_not_equal gate1_gaps RVIZ_BASE_LINK_FRONT_OBJECT_STATUS PASS
  add_gap_if_not_equal gate1_gaps RVIZ_FIXED_FRAME base_link
  add_gap_if_not_equal gate1_gaps RVIZ_FRONT_OBJECT_DIRECTION +X
  add_image_key_gap gate1_gaps SCREENSHOT

  add_gap_if_not_equal gate2_gaps TF_HEALTH_MAP_ODOM_STATUS PASS
  add_gap_if_not_equal gate2_gaps TF_HEALTH_MAP_BASE_STATUS PASS
  add_gap_if_not_equal gate2_gaps TF_HEALTH_STATUS PASS
  add_file_key_gap gate2_gaps TF_HEALTH_LOG
  add_file_contains_gap gate2_gaps TF_HEALTH_LOG "TF_HEALTH_MAP_ODOM_STATUS=PASS" "TF_HEALTH_LOG:missing_MAP_ODOM_PASS"
  add_file_contains_gap gate2_gaps TF_HEALTH_LOG "TF_HEALTH_MAP_BASE_STATUS=PASS" "TF_HEALTH_LOG:missing_MAP_BASE_PASS"
  add_file_contains_gap gate2_gaps TF_HEALTH_LOG "TF_HEALTH_STATUS=PASS" "TF_HEALTH_LOG:missing_OVERALL_PASS"

  add_gap_if_not_equal gate3_gaps RAW_ODOM_AXIS_STATUS PASS
  add_gap_if_not_equal gate3_gaps STANDARD_ODOM_AXIS_STATUS PASS
  add_gap_if_not_equal gate3_gaps RAW_STANDARD_ODOM_MATERIAL_MATCH true
  add_gap_if_not_equal gate3_gaps ODOM_AXIS_ROTATION_CHECK no_90deg_rotation
  add_file_key_gap gate3_gaps RAW_ODOM_AXIS_LOG
  add_file_contains_gap gate3_gaps RAW_ODOM_AXIS_LOG "ODOM_AXIS_TOPIC=/utlidar/robot_odom" "RAW_ODOM_AXIS_LOG:missing_raw_topic"
  add_file_contains_gap gate3_gaps RAW_ODOM_AXIS_LOG "ODOM_AXIS_STATUS=PASS" "RAW_ODOM_AXIS_LOG:missing_status_pass"
  add_file_key_gap gate3_gaps STANDARD_ODOM_AXIS_LOG
  add_file_contains_gap gate3_gaps STANDARD_ODOM_AXIS_LOG "ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard" "STANDARD_ODOM_AXIS_LOG:missing_standard_topic"
  add_file_contains_gap gate3_gaps STANDARD_ODOM_AXIS_LOG "ODOM_AXIS_STATUS=PASS" "STANDARD_ODOM_AXIS_LOG:missing_status_pass"

  add_gap_if_not_equal gate4_gaps GO2_XT16_SPORT_LIVE_READINESS PASS
  add_file_key_gap gate4_gaps SPORT_LIVE_READINESS_LOG
  add_file_contains_gap gate4_gaps SPORT_LIVE_READINESS_LOG "RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS" "SPORT_LIVE_READINESS_LOG:missing_pass_result"
  add_gap_if_not_equal gate4_gaps SPORT_PROBE_RESULT GO2_SPORT_ADAPTER_live_COMPLETE
  add_file_key_gap gate4_gaps SPORT_PROBE_SUMMARY
  add_summary_equal_gap gate4_gaps SPORT_PROBE_SUMMARY MODE live "SPORT_PROBE_SUMMARY:missing_live_mode"
  add_summary_equal_gap gate4_gaps SPORT_PROBE_SUMMARY RESULT GO2_SPORT_ADAPTER_live_COMPLETE "SPORT_PROBE_SUMMARY:missing_live_complete"
  add_summary_equal_gap gate4_gaps SPORT_PROBE_SUMMARY REQUEST_TOPIC /api/sport/request "SPORT_PROBE_SUMMARY:missing_real_request_topic"
  add_sport_probe_echo_gaps gate4_gaps
  add_gap_if_not_equal gate4_gaps SPORT_PROBE_HAS_MOVE_1008 true
  add_gap_if_not_equal gate4_gaps SPORT_PROBE_HAS_STOPMOVE_1003 true
  add_gap_if_not_equal gate4_gaps LIVE_NAV_SHORT_GOAL_STATUS PASS
  add_file_key_gap gate4_gaps LIVE_NAV_SUMMARY
  add_summary_equal_gap gate4_gaps LIVE_NAV_SUMMARY MODE live "LIVE_NAV_SUMMARY:missing_live_mode"
  add_summary_equal_gap gate4_gaps LIVE_NAV_SUMMARY RESULT GO2_XT16_MIXED_LIVE_NAV_STOPPED "LIVE_NAV_SUMMARY:missing_stopped_result"
  add_summary_equal_gap gate4_gaps LIVE_NAV_SUMMARY REQUEST_TOPIC /api/sport/request "LIVE_NAV_SUMMARY:missing_real_request_topic"
  add_summary_numeric_cap_gap gate4_gaps LIVE_NAV_SUMMARY SPORT_MAX_X 0.30 "LIVE_NAV_SUMMARY:SPORT_MAX_X_missing_or_above_0.30"
  add_file_key_gap gate4_gaps LIVE_NAV_LOG
  add_file_contains_gap gate4_gaps LIVE_NAV_LOG "RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING" "LIVE_NAV_LOG:missing_running_result"
  add_file_contains_gap gate4_gaps LIVE_NAV_LOG "SUMMARY_LOG=" "LIVE_NAV_LOG:missing_summary_log_path"
  add_live_speed_gap gate4_gaps
  add_gap_if_not_equal gate4_gaps NO_RESIDUAL_RUNTIME_STATUS PASS
  add_file_key_gap gate4_gaps NO_RESIDUAL_RUNTIME_LOG
  add_file_contains_gap gate4_gaps NO_RESIDUAL_RUNTIME_LOG "GO2_XT16_RUNTIME_DOCKER_STATUS=PASS" "NO_RESIDUAL_RUNTIME_LOG:missing_docker_pass"
  add_file_contains_gap gate4_gaps NO_RESIDUAL_RUNTIME_LOG "GO2_XT16_RUNTIME_PROCESS_STATUS=PASS" "NO_RESIDUAL_RUNTIME_LOG:missing_process_pass"
  add_file_contains_gap gate4_gaps NO_RESIDUAL_RUNTIME_LOG "GO2_XT16_RUNTIME_CLEAN_STATUS=PASS" "NO_RESIDUAL_RUNTIME_LOG:missing_clean_pass"

  [[ ${#gate1_gaps[@]} -eq 0 ]] || remaining+=("gate1")
  [[ ${#gate2_gaps[@]} -eq 0 ]] || remaining+=("gate2")
  [[ ${#gate3_gaps[@]} -eq 0 ]] || remaining+=("gate3")
  [[ ${#gate4_gaps[@]} -eq 0 ]] || remaining+=("gate4")

  echo "GO2_XT16_GATE_GAPS_EVIDENCE_FILE=${evidence_file}"
  print_gate GATE1_RVIZ_FRONT_OBJECT "${gate1_gaps[@]}"
  print_gate GATE2_INITIAL_POSE_TF_TILT "${gate2_gaps[@]}"
  print_gate GATE3_ODOM_AXIS "${gate3_gaps[@]}"
  print_gate GATE4_SUPERVISED_LIVE_SHORT_GOAL "${gate4_gaps[@]}"
  echo "GO2_XT16_GATE_GAPS_REMAINING_GATES=$(join_by_comma "${remaining[@]}")"
  if [[ ${#remaining[@]} -eq 0 ]]; then
    echo "GO2_XT16_GATE_GAPS_OVERALL=PASS"
  else
    echo "GO2_XT16_GATE_GAPS_OVERALL=NOT_READY"
  fi
  echo "GO2_XT16_GATE_GAPS_STATUS=PASS"
}

run_self_test() {
  local self_path
  local tmp_dir
  local pass_evidence
  local not_ready_evidence
  local relative_evidence
  local screenshot_file
  local tf_health_log
  local raw_odom_axis_log
  local standard_odom_axis_log
  local sport_readiness_log
  local sport_probe_summary
  local sport_probe_echo_log
  local live_nav_summary
  local live_nav_log
  local runtime_clean_log
  local out

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  pass_evidence="${tmp_dir}/pass.env"
  not_ready_evidence="${tmp_dir}/not_ready.env"
  relative_evidence="${tmp_dir}/relative.env"
  screenshot_file="${tmp_dir}/gate1.png"
  tf_health_log="${tmp_dir}/gate2_tf_health.txt"
  raw_odom_axis_log="${tmp_dir}/gate3_raw.txt"
  standard_odom_axis_log="${tmp_dir}/gate3_standard.txt"
  sport_readiness_log="${tmp_dir}/gate4_readiness.txt"
  sport_probe_summary="${tmp_dir}/gate4_probe_summary.env"
  sport_probe_echo_log="${tmp_dir}/gate4_probe_echo.log"
  live_nav_summary="${tmp_dir}/gate4_live_summary.env"
  live_nav_log="${tmp_dir}/gate4_live.log"
  runtime_clean_log="${tmp_dir}/runtime_clean.txt"

  printf '\x89PNG\r\n\x1a\n' >"${screenshot_file}"
  cat >"${tf_health_log}" <<'EOF'
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
EOF
  cat >"${raw_odom_axis_log}" <<'EOF'
ODOM_AXIS_TOPIC=/utlidar/robot_odom
ODOM_AXIS_STATUS=PASS
EOF
  cat >"${standard_odom_axis_log}" <<'EOF'
ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard
ODOM_AXIS_STATUS=PASS
EOF
  cat >"${sport_readiness_log}" <<'EOF'
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
EOF
  cat >"${sport_probe_echo_log}" <<'EOF'
api_id: 1008
api_id: 1003
EOF
  cat >"${sport_probe_summary}" <<EOF
MODE=live
RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
REQUEST_TOPIC=/api/sport/request
REQUEST_ECHO_LOG=${sport_probe_echo_log}
EOF
  cat >"${live_nav_summary}" <<'EOF'
MODE=live
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_TOPIC=/api/sport/request
SPORT_MAX_X=0.30
EOF
  cat >"${live_nav_log}" <<EOF
RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING
SUMMARY_LOG=${live_nav_summary}
EOF
  cat >"${runtime_clean_log}" <<'EOF'
GO2_XT16_RUNTIME_DOCKER_STATUS=PASS
GO2_XT16_RUNTIME_PROCESS_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
EOF

  cat >"${pass_evidence}" <<EOF
REPORT_PATH=/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md
RVIZ_BASE_LINK_FRONT_OBJECT_STATUS=PASS
RVIZ_FIXED_FRAME=base_link
RVIZ_FRONT_OBJECT_DIRECTION=+X
SCREENSHOT=${screenshot_file}
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
TF_HEALTH_LOG=${tf_health_log}
RAW_ODOM_AXIS_STATUS=PASS
STANDARD_ODOM_AXIS_STATUS=PASS
RAW_STANDARD_ODOM_MATERIAL_MATCH=true
ODOM_AXIS_ROTATION_CHECK=no_90deg_rotation
RAW_ODOM_AXIS_LOG=${raw_odom_axis_log}
STANDARD_ODOM_AXIS_LOG=${standard_odom_axis_log}
GO2_XT16_SPORT_LIVE_READINESS=PASS
SPORT_LIVE_READINESS_LOG=${sport_readiness_log}
SPORT_PROBE_RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
SPORT_PROBE_SUMMARY=${sport_probe_summary}
SPORT_PROBE_HAS_MOVE_1008=true
SPORT_PROBE_HAS_STOPMOVE_1003=true
LIVE_NAV_SHORT_GOAL_STATUS=PASS
LIVE_NAV_SUMMARY=${live_nav_summary}
LIVE_NAV_LOG=${live_nav_log}
LIVE_NAV_MAX_X_MPS=0.30
NO_RESIDUAL_RUNTIME_STATUS=PASS
NO_RESIDUAL_RUNTIME_LOG=${runtime_clean_log}
EOF
  cp "${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env.example" "${not_ready_evidence}"
  cp "${pass_evidence}" "${relative_evidence}"
  sed -i 's|^SCREENSHOT=.*|SCREENSHOT=gate1.png|' "${relative_evidence}"

  out="$("${self_path}" --evidence "${pass_evidence}")"
  [[ "${out}" == *"GO2_XT16_GATE_GAPS_OVERALL=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic PASS gaps did not clear" >&2
    return 1
  }
  echo "GO2_XT16_GATE_GAPS_SELFTEST_SYNTHETIC_PASS=PASS"

  out="$("${self_path}" --evidence "${not_ready_evidence}")"
  [[ "${out}" == *"GO2_XT16_GATE_GAPS_OVERALL=NOT_READY"* ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic NOT_READY gaps did not report NOT_READY" >&2
    return 1
  }
  [[ "${out}" == *"GO2_XT16_GATE_GAPS_REMAINING_GATES=gate1,gate2,gate3,gate4"* ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic NOT_READY gaps did not list all gates" >&2
    return 1
  }
  echo "GO2_XT16_GATE_GAPS_SELFTEST_SYNTHETIC_NOT_READY=PASS"

  out="$("${self_path}" --evidence "${relative_evidence}")"
  [[ "${out}" == *"SCREENSHOT:not_absolute"* ]] || {
    echo "${out}" >&2
    echo "ERROR: relative screenshot gap was not reported" >&2
    return 1
  }
  echo "GO2_XT16_GATE_GAPS_SELFTEST_RELATIVE_ARTIFACT=PASS"
  echo "GO2_XT16_GATE_GAPS_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  [[ "${evidence_file}" == "${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env" ]] || \
    die "--self-test cannot be combined with --evidence"
  run_self_test
  exit 0
fi

summarize_gaps
