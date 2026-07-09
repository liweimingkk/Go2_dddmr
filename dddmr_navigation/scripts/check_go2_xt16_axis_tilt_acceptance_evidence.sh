#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh --evidence FILE [--allow-incomplete]
  scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh --evidence FILE --status-report [--allow-incomplete]
  scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh --self-test

Check a KEY=VALUE evidence file for the remaining Go2 XT16 axis/tilt
acceptance gates. The evidence file is parsed as data and is not sourced.

This script does not source ROS, inspect live robot topics, start Docker,
publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel, or publish /api/sport/request.

Expected template:
  docs/go2_xt16_axis_tilt_acceptance_evidence.env.example

Exit status:
  0 when all gates are complete, or when --allow-incomplete is set.
  1 when one or more gates are incomplete.
  2 for usage or malformed input.
EOF
}

evidence_file=""
allow_incomplete=false
self_test=false
status_report=false

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
    --allow-incomplete)
      allow_incomplete=true
      shift
      ;;
    --status-report)
      status_report=true
      shift
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

run_self_test() {
  local self_path
  local tmp_dir
  local pass_file
  local high_speed_file
  local missing_artifact_file
  local relative_artifact_file
  local bad_content_file
  local bad_screenshot_file
  local bad_report_file
  local screenshot_file
  local text_screenshot_file
  local tf_health_log
  local bad_tf_health_log
  local raw_odom_axis_log
  local standard_odom_axis_log
  local sport_readiness_log
  local sport_probe_summary
  local sport_probe_echo_log
  local live_nav_summary
  local live_nav_log
  local runtime_clean_log
  local out
  local status

  self_path="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  pass_file="${tmp_dir}/synthetic_pass.env"
  high_speed_file="${tmp_dir}/synthetic_high_speed.env"
  missing_artifact_file="${tmp_dir}/synthetic_missing_artifact.env"
  relative_artifact_file="${tmp_dir}/synthetic_relative_artifact.env"
  bad_content_file="${tmp_dir}/synthetic_bad_content.env"
  bad_screenshot_file="${tmp_dir}/synthetic_bad_screenshot.env"
  bad_report_file="${tmp_dir}/synthetic_bad_report.env"
  screenshot_file="${tmp_dir}/gate1_base_link_front_object.png"
  text_screenshot_file="${tmp_dir}/gate1_text_screenshot.txt"
  tf_health_log="${tmp_dir}/gate2_tf_health.txt"
  bad_tf_health_log="${tmp_dir}/gate2_bad_tf_health.txt"
  raw_odom_axis_log="${tmp_dir}/gate3_raw_odom_axis.txt"
  standard_odom_axis_log="${tmp_dir}/gate3_standard_odom_axis.txt"
  sport_readiness_log="${tmp_dir}/gate4_sport_readiness.txt"
  sport_probe_summary="${tmp_dir}/gate4_sport_probe_summary.env"
  sport_probe_echo_log="${tmp_dir}/gate4_sport_probe_echo.log"
  live_nav_summary="${tmp_dir}/gate4_live_nav_summary.env"
  live_nav_log="${tmp_dir}/gate4_live_nav.log"
  runtime_clean_log="${tmp_dir}/gate4_runtime_clean.log"

  printf '\x89PNG\r\n\x1a\n' >"${screenshot_file}"
  printf 'not an image\n' >"${text_screenshot_file}"
  cat >"${tf_health_log}" <<'EOF'
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
EOF
  cat >"${bad_tf_health_log}" <<'EOF'
TF_HEALTH_STATUS=PASS
EOF
  cat >"${raw_odom_axis_log}" <<'EOF'
ODOM_AXIS_TOPIC=/utlidar/robot_odom
ODOM_AXIS_FORWARD_OK=true
ODOM_AXIS_HEADING_OK=true
ODOM_AXIS_LATERAL_OK=true
ODOM_AXIS_STATUS=PASS
EOF
  cat >"${standard_odom_axis_log}" <<'EOF'
ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard
ODOM_AXIS_FORWARD_OK=true
ODOM_AXIS_HEADING_OK=true
ODOM_AXIS_LATERAL_OK=true
ODOM_AXIS_STATUS=PASS
EOF
  cat >"${sport_readiness_log}" <<'EOF'
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
EOF
  cat >"${sport_probe_echo_log}" <<'EOF'
id: 1001
api_id: 1008
---
id: 1002
api_id: 1003
EOF
  cat >"${sport_probe_summary}" <<EOF
MODE=live
RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
REQUEST_TOPIC=/api/sport/request
REQUEST_ECHO_LOG=${sport_probe_echo_log}
MAX_X=0.10
EOF
  cat >"${live_nav_summary}" <<'EOF'
MODE=live
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_TOPIC=/api/sport/request
SPORT_MAX_X=0.30
EOF
  cat >"${live_nav_log}" <<'EOF'
RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING
SUMMARY_LOG=/tmp/synthetic_live_nav_summary.env
EOF
  cat >"${runtime_clean_log}" <<'EOF'
GO2_XT16_RUNTIME_DOCKER_STATUS=PASS
GO2_XT16_RUNTIME_PROCESS_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
EOF

  cat >"${pass_file}" <<EOF
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

  cp "${pass_file}" "${high_speed_file}"
  sed -i 's/^LIVE_NAV_MAX_X_MPS=.*/LIVE_NAV_MAX_X_MPS=0.50/' "${high_speed_file}"
  cp "${pass_file}" "${missing_artifact_file}"
  sed -i "s|^SCREENSHOT=.*|SCREENSHOT=${tmp_dir}/missing_gate1.png|" "${missing_artifact_file}"
  cp "${pass_file}" "${relative_artifact_file}"
  sed -i "s|^SCREENSHOT=.*|SCREENSHOT=gate1_base_link_front_object.png|" "${relative_artifact_file}"
  cp "${pass_file}" "${bad_screenshot_file}"
  sed -i "s|^SCREENSHOT=.*|SCREENSHOT=${text_screenshot_file}|" "${bad_screenshot_file}"
  cp "${pass_file}" "${bad_content_file}"
  sed -i "s|^TF_HEALTH_LOG=.*|TF_HEALTH_LOG=${bad_tf_health_log}|" "${bad_content_file}"
  cp "${pass_file}" "${bad_report_file}"
  sed -i "s|^REPORT_PATH=.*|REPORT_PATH=${tmp_dir}/wrong_report.md|" "${bad_report_file}"

  out="$("${self_path}" --evidence "${pass_file}")"
  [[ "${out}" == *"GO2_XT16_ACCEPTANCE_STATUS=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic PASS evidence was not accepted" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_SYNTHETIC_PASS=PASS"

  out="$("${self_path}" --evidence "${pass_file}" --status-report)"
  [[ "${out}" == *"GO2_XT16_STATUS_REPORT_OVERALL=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: status-report mode did not accept synthetic PASS evidence" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_STATUS_REPORT=PASS"

  set +e
  out="$("${self_path}" --evidence "${high_speed_file}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: high-speed synthetic evidence unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"GO2_XT16_ACCEPTANCE_STATUS=NOT_READY"* ]] || {
    echo "${out}" >&2
    echo "ERROR: high-speed synthetic evidence did not report NOT_READY" >&2
    return 1
  }
  [[ "${out}" == *"LIVE_NAV_MAX_X_MPS_missing_or_above_0.30"* ]] || {
    echo "${out}" >&2
    echo "ERROR: high-speed synthetic evidence did not report speed-cap reason" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_SPEED_CAP_REJECT=PASS"

  set +e
  out="$("${self_path}" --evidence "${missing_artifact_file}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: missing-artifact synthetic evidence unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"SCREENSHOT_missing_or_empty_file"* ]] || {
    echo "${out}" >&2
    echo "ERROR: missing-artifact synthetic evidence did not report screenshot reason" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_REJECT=PASS"

  set +e
  out="$("${self_path}" --evidence "${relative_artifact_file}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: relative-artifact synthetic evidence unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"SCREENSHOT_not_absolute"* ]] || {
    echo "${out}" >&2
    echo "ERROR: relative-artifact synthetic evidence did not report absolute path reason" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_ABSOLUTE_PATH_REJECT=PASS"

  set +e
  out="$("${self_path}" --evidence "${bad_screenshot_file}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: bad-screenshot synthetic evidence unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"SCREENSHOT_not_png_or_jpeg"* ]] || {
    echo "${out}" >&2
    echo "ERROR: bad-screenshot synthetic evidence did not report image reason" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_SCREENSHOT_TYPE_REJECT=PASS"

  set +e
  out="$("${self_path}" --evidence "${bad_report_file}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: bad-report synthetic evidence unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"REPORT_PATH_mismatch"* ]] || {
    echo "${out}" >&2
    echo "ERROR: bad-report synthetic evidence did not report REPORT_PATH mismatch" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_REPORT_LINK_REJECT=PASS"

  set +e
  out="$("${self_path}" --evidence "${bad_content_file}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: bad-content synthetic evidence unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"TF_HEALTH_LOG_missing_MAP_ODOM_PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: bad-content synthetic evidence did not report TF content reason" >&2
    return 1
  }
  echo "GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_CONTENT_REJECT=PASS"

  echo "GO2_XT16_ACCEPTANCE_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  if [[ -n "${evidence_file}" || "${allow_incomplete}" == "true" || "${status_report}" == "true" ]]; then
    echo "ERROR: --self-test cannot be combined with --evidence, --allow-incomplete, or --status-report" >&2
    exit 2
  fi
  run_self_test
  exit 0
fi

if [[ -z "${evidence_file}" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -f "${evidence_file}" ]]; then
  if [[ "${status_report}" == "true" ]]; then
    echo "GO2_XT16_STATUS_REPORT_EVIDENCE_FILE=${evidence_file}"
    echo "GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY"
    echo "GO2_XT16_STATUS_REPORT_REASON=missing_evidence_file"
  else
    echo "GO2_XT16_ACCEPTANCE_EVIDENCE_FILE=${evidence_file}"
    echo "GO2_XT16_ACCEPTANCE_STATUS=NOT_READY"
    echo "GO2_XT16_ACCEPTANCE_REASON=missing_evidence_file"
  fi
  if [[ "${allow_incomplete}" == "true" ]]; then
    exit 0
  fi
  exit 1
fi

get_value() {
  local key="$1"
  awk -v key="${key}" '
    BEGIN { FS = "=" }
    /^[[:space:]]*#/ { next }
    /^[[:space:]]*$/ { next }
    $1 == key {
      sub(/^[^=]*=/, "")
      gsub(/\r$/, "")
      print
      found = 1
      exit
    }
    END {
      if (!found) {
        exit 1
      }
    }
  ' "${evidence_file}" || true
}

print_gate() {
  local name="$1"
  local status="$2"
  local reason="$3"
  if [[ "${status_report}" == "true" ]]; then
    echo "GO2_XT16_STATUS_REPORT_${name}=${status}"
    if [[ -n "${reason}" ]]; then
      echo "GO2_XT16_STATUS_REPORT_${name}_REASON=${reason}"
    fi
  else
    echo "GO2_XT16_ACCEPTANCE_${name}_STATUS=${status}"
    if [[ -n "${reason}" ]]; then
      echo "GO2_XT16_ACCEPTANCE_${name}_REASON=${reason}"
    fi
  fi
}

require_equal() {
  local key="$1"
  local expected="$2"
  local value
  value="$(get_value "${key}")"
  [[ "${value}" == "${expected}" ]]
}

require_nonempty_pathish() {
  local key="$1"
  local value
  value="$(get_value "${key}")"
  [[ -n "${value}" && "${value}" != "TODO" && "${value}" != "<path>" ]]
}

require_absolute_path() {
  local key="$1"
  local value
  value="$(get_value "${key}")"
  [[ "${value}" == /* ]]
}

require_existing_file() {
  local key="$1"
  local value
  value="$(get_value "${key}")"
  require_nonempty_pathish "${key}" && require_absolute_path "${key}" && [[ -f "${value}" && -s "${value}" ]]
}

require_image_file() {
  local key="$1"
  local value
  local signature
  value="$(get_value "${key}")"
  require_existing_file "${key}" || return 1
  signature="$(LC_ALL=C head -c 12 "${value}" | od -An -tx1 | tr -d ' \n')"
  case "${signature}" in
    89504e470d0a1a0a*|ffd8ff*) return 0 ;;
    *) return 1 ;;
  esac
}

require_file_contains() {
  local key="$1"
  local text="$2"
  local value
  value="$(get_value "${key}")"
  require_existing_file "${key}" || return 1
  rg -q --fixed-strings -- "${text}" "${value}"
}

get_file_value() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" '
    BEGIN { FS = "=" }
    /^[[:space:]]*#/ { next }
    /^[[:space:]]*$/ { next }
    $1 == key {
      sub(/^[^=]*=/, "")
      gsub(/\r$/, "")
      print
      found = 1
      exit
    }
    END {
      if (!found) {
        exit 1
      }
    }
  ' "${file}" || true
}

require_summary_equal() {
  local evidence_key="$1"
  local summary_key="$2"
  local expected="$3"
  local file
  local value
  file="$(get_value "${evidence_key}")"
  [[ "${file}" == /* && -f "${file}" ]] || return 1
  value="$(get_file_value "${file}" "${summary_key}")"
  [[ "${value}" == "${expected}" ]]
}

require_summary_numeric_cap() {
  local evidence_key="$1"
  local summary_key="$2"
  local limit="$3"
  local file
  local value
  file="$(get_value "${evidence_key}")"
  [[ "${file}" == /* && -f "${file}" ]] || return 1
  value="$(get_file_value "${file}" "${summary_key}")"
  python3 - "${value}" "${limit}" <<'PY'
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

require_sport_probe_summary_api_ids() {
  local summary
  local echo_log
  summary="$(get_value SPORT_PROBE_SUMMARY)"
  [[ "${summary}" == /* && -f "${summary}" ]] || return 1
  echo_log="$(get_file_value "${summary}" REQUEST_ECHO_LOG)"
  [[ "${echo_log}" == /* && -f "${echo_log}" ]] || return 1
  rg -q 'api_id:[[:space:]]*1008' "${echo_log}" || return 1
  rg -q 'api_id:[[:space:]]*1003' "${echo_log}" || return 1
}

require_live_speed_cap() {
  local value
  value="$(get_value LIVE_NAV_MAX_X_MPS)"
  python3 - "${value}" <<'PY'
import math
import sys

raw = sys.argv[1]
try:
    value = float(raw)
except ValueError:
    raise SystemExit(1)
if not math.isfinite(value):
    raise SystemExit(1)
if value < 0.0 or value > 0.30:
    raise SystemExit(1)
PY
}

if [[ "${status_report}" == "true" ]]; then
  echo "GO2_XT16_STATUS_REPORT_EVIDENCE_FILE=${evidence_file}"
else
  echo "GO2_XT16_ACCEPTANCE_EVIDENCE_FILE=${evidence_file}"
fi

expected_report_path="${GO2_XT16_AXIS_TILT_REPORT:-/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md}"
evidence_report_path="$(get_value REPORT_PATH)"
report_link_status="PASS"
report_link_reason=""
if [[ -z "${evidence_report_path}" ]]; then
  report_link_status="NOT_READY"
  report_link_reason="REPORT_PATH_missing"
elif [[ "${evidence_report_path}" != "${expected_report_path}" ]]; then
  report_link_status="NOT_READY"
  report_link_reason="REPORT_PATH_mismatch"
elif [[ ! -f "${evidence_report_path}" ]]; then
  report_link_status="NOT_READY"
  report_link_reason="REPORT_PATH_not_file"
fi
if [[ "${status_report}" == "true" ]]; then
  echo "GO2_XT16_STATUS_REPORT_REPORT_LINK=${report_link_status}"
  if [[ -n "${report_link_reason}" ]]; then
    echo "GO2_XT16_STATUS_REPORT_REPORT_LINK_REASON=${report_link_reason}"
  fi
else
  echo "GO2_XT16_ACCEPTANCE_REPORT_LINK_STATUS=${report_link_status}"
  if [[ -n "${report_link_reason}" ]]; then
    echo "GO2_XT16_ACCEPTANCE_REPORT_LINK_REASON=${report_link_reason}"
  fi
fi

gate1_status="PASS"
gate1_reason=""
if ! require_equal RVIZ_BASE_LINK_FRONT_OBJECT_STATUS PASS; then
  gate1_status="NOT_READY"
  gate1_reason="RVIZ_BASE_LINK_FRONT_OBJECT_STATUS_not_PASS"
elif ! require_equal RVIZ_FIXED_FRAME base_link; then
  gate1_status="NOT_READY"
  gate1_reason="RVIZ_FIXED_FRAME_not_base_link"
elif ! require_equal RVIZ_FRONT_OBJECT_DIRECTION +X; then
  gate1_status="NOT_READY"
  gate1_reason="RVIZ_FRONT_OBJECT_DIRECTION_not_plus_X"
elif ! require_nonempty_pathish SCREENSHOT; then
  gate1_status="NOT_READY"
  gate1_reason="SCREENSHOT_missing"
elif ! require_absolute_path SCREENSHOT; then
  gate1_status="NOT_READY"
  gate1_reason="SCREENSHOT_not_absolute"
elif ! require_existing_file SCREENSHOT; then
  gate1_status="NOT_READY"
  gate1_reason="SCREENSHOT_missing_or_empty_file"
elif ! require_image_file SCREENSHOT; then
  gate1_status="NOT_READY"
  gate1_reason="SCREENSHOT_not_png_or_jpeg"
fi
print_gate GATE1_RVIZ_FRONT_OBJECT "${gate1_status}" "${gate1_reason}"

gate2_status="PASS"
gate2_reason=""
if ! require_equal TF_HEALTH_MAP_ODOM_STATUS PASS; then
  gate2_status="NOT_READY"
  gate2_reason="TF_HEALTH_MAP_ODOM_STATUS_not_PASS"
elif ! require_equal TF_HEALTH_MAP_BASE_STATUS PASS; then
  gate2_status="NOT_READY"
  gate2_reason="TF_HEALTH_MAP_BASE_STATUS_not_PASS"
elif ! require_equal TF_HEALTH_STATUS PASS; then
  gate2_status="NOT_READY"
  gate2_reason="TF_HEALTH_STATUS_not_PASS"
elif ! require_existing_file TF_HEALTH_LOG; then
  gate2_status="NOT_READY"
  if ! require_absolute_path TF_HEALTH_LOG; then
    gate2_reason="TF_HEALTH_LOG_not_absolute"
  else
    gate2_reason="TF_HEALTH_LOG_missing_or_empty_file"
  fi
elif ! require_file_contains TF_HEALTH_LOG "TF_HEALTH_MAP_ODOM_STATUS=PASS"; then
  gate2_status="NOT_READY"
  gate2_reason="TF_HEALTH_LOG_missing_MAP_ODOM_PASS"
elif ! require_file_contains TF_HEALTH_LOG "TF_HEALTH_MAP_BASE_STATUS=PASS"; then
  gate2_status="NOT_READY"
  gate2_reason="TF_HEALTH_LOG_missing_MAP_BASE_PASS"
elif ! require_file_contains TF_HEALTH_LOG "TF_HEALTH_STATUS=PASS"; then
  gate2_status="NOT_READY"
  gate2_reason="TF_HEALTH_LOG_missing_OVERALL_PASS"
fi
print_gate GATE2_INITIAL_POSE_TF_TILT "${gate2_status}" "${gate2_reason}"

gate3_status="PASS"
gate3_reason=""
if ! require_equal RAW_ODOM_AXIS_STATUS PASS; then
  gate3_status="NOT_READY"
  gate3_reason="RAW_ODOM_AXIS_STATUS_not_PASS"
elif ! require_equal STANDARD_ODOM_AXIS_STATUS PASS; then
  gate3_status="NOT_READY"
  gate3_reason="STANDARD_ODOM_AXIS_STATUS_not_PASS"
elif ! require_equal RAW_STANDARD_ODOM_MATERIAL_MATCH true; then
  gate3_status="NOT_READY"
  gate3_reason="RAW_STANDARD_ODOM_MATERIAL_MATCH_not_true"
elif ! require_equal ODOM_AXIS_ROTATION_CHECK no_90deg_rotation; then
  gate3_status="NOT_READY"
  gate3_reason="ODOM_AXIS_ROTATION_CHECK_not_no_90deg_rotation"
elif ! require_existing_file RAW_ODOM_AXIS_LOG; then
  gate3_status="NOT_READY"
  if ! require_absolute_path RAW_ODOM_AXIS_LOG; then
    gate3_reason="RAW_ODOM_AXIS_LOG_not_absolute"
  else
    gate3_reason="RAW_ODOM_AXIS_LOG_missing_or_empty_file"
  fi
elif ! require_file_contains RAW_ODOM_AXIS_LOG "ODOM_AXIS_TOPIC=/utlidar/robot_odom"; then
  gate3_status="NOT_READY"
  gate3_reason="RAW_ODOM_AXIS_LOG_missing_raw_topic"
elif ! require_file_contains RAW_ODOM_AXIS_LOG "ODOM_AXIS_STATUS=PASS"; then
  gate3_status="NOT_READY"
  gate3_reason="RAW_ODOM_AXIS_LOG_missing_status_pass"
elif ! require_existing_file STANDARD_ODOM_AXIS_LOG; then
  gate3_status="NOT_READY"
  if ! require_absolute_path STANDARD_ODOM_AXIS_LOG; then
    gate3_reason="STANDARD_ODOM_AXIS_LOG_not_absolute"
  else
    gate3_reason="STANDARD_ODOM_AXIS_LOG_missing_or_empty_file"
  fi
elif ! require_file_contains STANDARD_ODOM_AXIS_LOG "ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard"; then
  gate3_status="NOT_READY"
  gate3_reason="STANDARD_ODOM_AXIS_LOG_missing_standard_topic"
elif ! require_file_contains STANDARD_ODOM_AXIS_LOG "ODOM_AXIS_STATUS=PASS"; then
  gate3_status="NOT_READY"
  gate3_reason="STANDARD_ODOM_AXIS_LOG_missing_status_pass"
fi
print_gate GATE3_ODOM_AXIS "${gate3_status}" "${gate3_reason}"

gate4_status="PASS"
gate4_reason=""
if ! require_equal GO2_XT16_SPORT_LIVE_READINESS PASS; then
  gate4_status="NOT_READY"
  gate4_reason="GO2_XT16_SPORT_LIVE_READINESS_not_PASS"
elif ! require_existing_file SPORT_LIVE_READINESS_LOG; then
  gate4_status="NOT_READY"
  if ! require_absolute_path SPORT_LIVE_READINESS_LOG; then
    gate4_reason="SPORT_LIVE_READINESS_LOG_not_absolute"
  else
    gate4_reason="SPORT_LIVE_READINESS_LOG_missing_or_empty_file"
  fi
elif ! require_file_contains SPORT_LIVE_READINESS_LOG "RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS"; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_LIVE_READINESS_LOG_missing_pass_result"
elif ! require_equal SPORT_PROBE_RESULT GO2_SPORT_ADAPTER_live_COMPLETE; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_RESULT_not_live_complete"
elif ! require_existing_file SPORT_PROBE_SUMMARY; then
  gate4_status="NOT_READY"
  if ! require_absolute_path SPORT_PROBE_SUMMARY; then
    gate4_reason="SPORT_PROBE_SUMMARY_not_absolute"
  else
    gate4_reason="SPORT_PROBE_SUMMARY_missing_or_empty_file"
  fi
elif ! require_summary_equal SPORT_PROBE_SUMMARY MODE live; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_SUMMARY_missing_live_mode"
elif ! require_summary_equal SPORT_PROBE_SUMMARY RESULT GO2_SPORT_ADAPTER_live_COMPLETE; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_SUMMARY_missing_live_complete"
elif ! require_summary_equal SPORT_PROBE_SUMMARY REQUEST_TOPIC /api/sport/request; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_SUMMARY_missing_real_request_topic"
elif ! require_sport_probe_summary_api_ids; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_SUMMARY_missing_move_or_stop_echo"
elif ! require_equal SPORT_PROBE_HAS_MOVE_1008 true; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_HAS_MOVE_1008_not_true"
elif ! require_equal SPORT_PROBE_HAS_STOPMOVE_1003 true; then
  gate4_status="NOT_READY"
  gate4_reason="SPORT_PROBE_HAS_STOPMOVE_1003_not_true"
elif ! require_equal LIVE_NAV_SHORT_GOAL_STATUS PASS; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_SHORT_GOAL_STATUS_not_PASS"
elif ! require_existing_file LIVE_NAV_SUMMARY; then
  gate4_status="NOT_READY"
  if ! require_absolute_path LIVE_NAV_SUMMARY; then
    gate4_reason="LIVE_NAV_SUMMARY_not_absolute"
  else
    gate4_reason="LIVE_NAV_SUMMARY_missing_or_empty_file"
  fi
elif ! require_summary_equal LIVE_NAV_SUMMARY MODE live; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_SUMMARY_missing_live_mode"
elif ! require_summary_equal LIVE_NAV_SUMMARY RESULT GO2_XT16_MIXED_LIVE_NAV_STOPPED; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_SUMMARY_missing_stopped_result"
elif ! require_summary_equal LIVE_NAV_SUMMARY REQUEST_TOPIC /api/sport/request; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_SUMMARY_missing_real_request_topic"
elif ! require_summary_numeric_cap LIVE_NAV_SUMMARY SPORT_MAX_X 0.30; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_SUMMARY_max_x_missing_or_above_0.30"
elif ! require_existing_file LIVE_NAV_LOG; then
  gate4_status="NOT_READY"
  if ! require_absolute_path LIVE_NAV_LOG; then
    gate4_reason="LIVE_NAV_LOG_not_absolute"
  else
    gate4_reason="LIVE_NAV_LOG_missing_or_empty_file"
  fi
elif ! require_file_contains LIVE_NAV_LOG "RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING"; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_LOG_missing_running_result"
elif ! require_file_contains LIVE_NAV_LOG "SUMMARY_LOG="; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_LOG_missing_summary_log_path"
elif ! require_live_speed_cap; then
  gate4_status="NOT_READY"
  gate4_reason="LIVE_NAV_MAX_X_MPS_missing_or_above_0.30"
elif ! require_equal NO_RESIDUAL_RUNTIME_STATUS PASS; then
  gate4_status="NOT_READY"
  gate4_reason="NO_RESIDUAL_RUNTIME_STATUS_not_PASS"
elif ! require_existing_file NO_RESIDUAL_RUNTIME_LOG; then
  gate4_status="NOT_READY"
  if ! require_absolute_path NO_RESIDUAL_RUNTIME_LOG; then
    gate4_reason="NO_RESIDUAL_RUNTIME_LOG_not_absolute"
  else
    gate4_reason="NO_RESIDUAL_RUNTIME_LOG_missing_or_empty_file"
  fi
elif ! require_file_contains NO_RESIDUAL_RUNTIME_LOG "GO2_XT16_RUNTIME_DOCKER_STATUS=PASS"; then
  gate4_status="NOT_READY"
  gate4_reason="NO_RESIDUAL_RUNTIME_LOG_missing_docker_pass"
elif ! require_file_contains NO_RESIDUAL_RUNTIME_LOG "GO2_XT16_RUNTIME_PROCESS_STATUS=PASS"; then
  gate4_status="NOT_READY"
  gate4_reason="NO_RESIDUAL_RUNTIME_LOG_missing_process_pass"
elif ! require_file_contains NO_RESIDUAL_RUNTIME_LOG "GO2_XT16_RUNTIME_CLEAN_STATUS=PASS"; then
  gate4_status="NOT_READY"
  gate4_reason="NO_RESIDUAL_RUNTIME_LOG_missing_clean_pass"
fi
print_gate GATE4_SUPERVISED_LIVE_SHORT_GOAL "${gate4_status}" "${gate4_reason}"

overall="PASS"
for status in "${report_link_status}" "${gate1_status}" "${gate2_status}" "${gate3_status}" "${gate4_status}"; do
  if [[ "${status}" != "PASS" ]]; then
    overall="NOT_READY"
  fi
done

if [[ "${status_report}" == "true" ]]; then
  echo "GO2_XT16_STATUS_REPORT_OVERALL=${overall}"
else
  echo "GO2_XT16_ACCEPTANCE_STATUS=${overall}"
fi
if [[ "${overall}" != "PASS" ]]; then
  if [[ "${status_report}" == "true" ]]; then
    echo "GO2_XT16_STATUS_REPORT_REASON=manual_or_supervised_gate_evidence_incomplete"
  else
    echo "GO2_XT16_ACCEPTANCE_REASON=manual_or_supervised_gate_evidence_incomplete"
  fi
  if [[ "${allow_incomplete}" == "true" ]]; then
    exit 0
  fi
  exit 1
fi

if [[ "${status_report}" == "true" ]]; then
  echo "GO2_XT16_STATUS_REPORT_REASON=all_required_evidence_present"
else
  echo "GO2_XT16_ACCEPTANCE_REASON=all_required_evidence_present"
fi
