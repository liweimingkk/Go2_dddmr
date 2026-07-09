#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/build_go2_xt16_axis_tilt_gate_update.sh --gate gate1 \
    --screenshot PNG_OR_JPEG [--fixed-frame base_link] [--direction +X]

  scripts/build_go2_xt16_axis_tilt_gate_update.sh --gate gate2 \
    --tf-health-log LOG

  scripts/build_go2_xt16_axis_tilt_gate_update.sh --gate gate3 \
    --raw-odom-axis-log LOG --standard-odom-axis-log LOG

  scripts/build_go2_xt16_axis_tilt_gate_update.sh --gate gate4 \
    --sport-readiness-log LOG --sport-probe-summary SUMMARY_ENV \
    --live-nav-summary SUMMARY_ENV --live-nav-log LOG --runtime-clean-log LOG \
    [--live-nav-max-x 0.30]

  scripts/build_go2_xt16_axis_tilt_gate_update.sh --self-test

Build a KEY=VALUE update file from already captured Go2 XT16 axis/tilt gate
artifacts. Output is written to stdout and can be passed to
update_go2_xt16_axis_tilt_acceptance_evidence.sh.

This script reads local artifact files only. It does not source ROS, inspect
live robot topics, start Docker, publish /cmd_vel, publish
/dddmr_go2/dry_run_cmd_vel, publish /api/sport/request, or publish /lowcmd.
EOF
}

gate=""
screenshot=""
fixed_frame="base_link"
direction="+X"
tf_health_log=""
raw_odom_axis_log=""
standard_odom_axis_log=""
sport_readiness_log=""
sport_probe_summary=""
live_nav_summary=""
live_nav_log=""
runtime_clean_log=""
live_nav_max_x="0.30"
self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gate)
      [[ $# -ge 2 ]] || { echo "ERROR: --gate requires a value" >&2; exit 2; }
      gate="$2"
      shift 2
      ;;
    --screenshot)
      [[ $# -ge 2 ]] || { echo "ERROR: --screenshot requires a path" >&2; exit 2; }
      screenshot="$2"
      shift 2
      ;;
    --fixed-frame)
      [[ $# -ge 2 ]] || { echo "ERROR: --fixed-frame requires a value" >&2; exit 2; }
      fixed_frame="$2"
      shift 2
      ;;
    --direction)
      [[ $# -ge 2 ]] || { echo "ERROR: --direction requires a value" >&2; exit 2; }
      direction="$2"
      shift 2
      ;;
    --tf-health-log)
      [[ $# -ge 2 ]] || { echo "ERROR: --tf-health-log requires a path" >&2; exit 2; }
      tf_health_log="$2"
      shift 2
      ;;
    --raw-odom-axis-log)
      [[ $# -ge 2 ]] || { echo "ERROR: --raw-odom-axis-log requires a path" >&2; exit 2; }
      raw_odom_axis_log="$2"
      shift 2
      ;;
    --standard-odom-axis-log)
      [[ $# -ge 2 ]] || { echo "ERROR: --standard-odom-axis-log requires a path" >&2; exit 2; }
      standard_odom_axis_log="$2"
      shift 2
      ;;
    --sport-readiness-log)
      [[ $# -ge 2 ]] || { echo "ERROR: --sport-readiness-log requires a path" >&2; exit 2; }
      sport_readiness_log="$2"
      shift 2
      ;;
    --sport-probe-summary)
      [[ $# -ge 2 ]] || { echo "ERROR: --sport-probe-summary requires a path" >&2; exit 2; }
      sport_probe_summary="$2"
      shift 2
      ;;
    --live-nav-summary)
      [[ $# -ge 2 ]] || { echo "ERROR: --live-nav-summary requires a path" >&2; exit 2; }
      live_nav_summary="$2"
      shift 2
      ;;
    --live-nav-log)
      [[ $# -ge 2 ]] || { echo "ERROR: --live-nav-log requires a path" >&2; exit 2; }
      live_nav_log="$2"
      shift 2
      ;;
    --runtime-clean-log)
      [[ $# -ge 2 ]] || { echo "ERROR: --runtime-clean-log requires a path" >&2; exit 2; }
      runtime_clean_log="$2"
      shift 2
      ;;
    --live-nav-max-x)
      [[ $# -ge 2 ]] || { echo "ERROR: --live-nav-max-x requires a value" >&2; exit 2; }
      live_nav_max_x="$2"
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

die() {
  echo "ERROR: $*" >&2
  exit 2
}

require_absolute_path() {
  local path="$1"
  [[ "${path}" == /* ]] || die "path is not absolute: ${path}"
}

require_file() {
  local path="$1"
  require_absolute_path "${path}"
  [[ -s "${path}" ]] || die "missing or empty file: ${path}"
}

require_contains() {
  local path="$1"
  local text="$2"
  require_file "${path}"
  grep -Fq -- "${text}" "${path}" || die "${path} missing required text: ${text}"
}

summary_value() {
  local path="$1"
  local key="$2"
  awk -F= -v key="${key}" '
    /^[[:space:]]*#/ { next }
    /^[[:space:]]*$/ { next }
    $1 == key {
      sub(/^[^=]*=/, "")
      gsub(/\r$/, "")
      print
      found = 1
      exit
    }
    END { exit found ? 0 : 1 }
  ' "${path}"
}

require_summary_equal() {
  local path="$1"
  local key="$2"
  local expected="$3"
  local actual
  require_file "${path}"
  actual="$(summary_value "${path}" "${key}")" || die "${path} missing ${key}"
  [[ "${actual}" == "${expected}" ]] || die "${path} ${key}=${actual}, expected ${expected}"
}

require_numeric_cap() {
  local value="$1"
  local cap="$2"
  awk -v value="${value}" -v cap="${cap}" 'BEGIN {
    if (value == "" || value !~ /^[-+]?[0-9]+([.][0-9]+)?$/) exit 1
    exit (value + 0 <= cap + 0) ? 0 : 1
  }' || die "numeric value ${value:-MISSING} exceeds cap ${cap}"
}

require_image_file() {
  local path="$1"
  local magic
  require_file "${path}"
  magic="$(od -An -tx1 -N8 "${path}" | tr -d ' \n')"
  if [[ "${magic}" != 89504e470d0a1a0a* && "${magic}" != ffd8ff* ]]; then
    die "${path} is not a PNG or JPEG"
  fi
}

emit_gate1() {
  [[ -n "${screenshot}" ]] || die "--screenshot is required for gate1"
  [[ "${fixed_frame}" == "base_link" ]] || die "gate1 fixed frame must be base_link"
  [[ "${direction}" == "+X" ]] || die "gate1 direction must be +X"
  require_image_file "${screenshot}"
  cat <<EOF
RVIZ_BASE_LINK_FRONT_OBJECT_STATUS=PASS
RVIZ_FIXED_FRAME=${fixed_frame}
RVIZ_FRONT_OBJECT_DIRECTION=${direction}
SCREENSHOT=${screenshot}
EOF
}

emit_gate2() {
  [[ -n "${tf_health_log}" ]] || die "--tf-health-log is required for gate2"
  require_contains "${tf_health_log}" "TF_HEALTH_MAP_ODOM_STATUS=PASS"
  require_contains "${tf_health_log}" "TF_HEALTH_MAP_BASE_STATUS=PASS"
  require_contains "${tf_health_log}" "TF_HEALTH_STATUS=PASS"
  cat <<EOF
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
TF_HEALTH_LOG=${tf_health_log}
EOF
}

emit_gate3() {
  [[ -n "${raw_odom_axis_log}" ]] || die "--raw-odom-axis-log is required for gate3"
  [[ -n "${standard_odom_axis_log}" ]] || die "--standard-odom-axis-log is required for gate3"
  require_contains "${raw_odom_axis_log}" "ODOM_AXIS_TOPIC=/utlidar/robot_odom"
  require_contains "${raw_odom_axis_log}" "ODOM_AXIS_STATUS=PASS"
  require_contains "${standard_odom_axis_log}" "ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard"
  require_contains "${standard_odom_axis_log}" "ODOM_AXIS_STATUS=PASS"
  cat <<EOF
RAW_ODOM_AXIS_STATUS=PASS
STANDARD_ODOM_AXIS_STATUS=PASS
RAW_STANDARD_ODOM_MATERIAL_MATCH=true
ODOM_AXIS_ROTATION_CHECK=no_90deg_rotation
RAW_ODOM_AXIS_LOG=${raw_odom_axis_log}
STANDARD_ODOM_AXIS_LOG=${standard_odom_axis_log}
EOF
}

emit_gate4() {
  local request_echo_log
  local sport_max_x

  [[ -n "${sport_readiness_log}" ]] || die "--sport-readiness-log is required for gate4"
  [[ -n "${sport_probe_summary}" ]] || die "--sport-probe-summary is required for gate4"
  [[ -n "${live_nav_summary}" ]] || die "--live-nav-summary is required for gate4"
  [[ -n "${live_nav_log}" ]] || die "--live-nav-log is required for gate4"
  [[ -n "${runtime_clean_log}" ]] || die "--runtime-clean-log is required for gate4"

  require_contains "${sport_readiness_log}" "RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS"
  require_summary_equal "${sport_probe_summary}" MODE live
  require_summary_equal "${sport_probe_summary}" RESULT GO2_SPORT_ADAPTER_live_COMPLETE
  require_summary_equal "${sport_probe_summary}" REQUEST_TOPIC /api/sport/request
  request_echo_log="$(summary_value "${sport_probe_summary}" REQUEST_ECHO_LOG)" || die "${sport_probe_summary} missing REQUEST_ECHO_LOG"
  require_absolute_path "${request_echo_log}"
  require_contains "${request_echo_log}" "api_id: 1008"
  require_contains "${request_echo_log}" "api_id: 1003"

  require_summary_equal "${live_nav_summary}" MODE live
  require_summary_equal "${live_nav_summary}" RESULT GO2_XT16_MIXED_LIVE_NAV_STOPPED
  require_summary_equal "${live_nav_summary}" REQUEST_TOPIC /api/sport/request
  sport_max_x="$(summary_value "${live_nav_summary}" SPORT_MAX_X)" || die "${live_nav_summary} missing SPORT_MAX_X"
  require_numeric_cap "${sport_max_x}" 0.30
  require_contains "${live_nav_log}" "RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING"
  require_contains "${live_nav_log}" "SUMMARY_LOG="

  require_contains "${runtime_clean_log}" "GO2_XT16_RUNTIME_DOCKER_STATUS=PASS"
  require_contains "${runtime_clean_log}" "GO2_XT16_RUNTIME_PROCESS_STATUS=PASS"
  require_contains "${runtime_clean_log}" "GO2_XT16_RUNTIME_CLEAN_STATUS=PASS"
  require_numeric_cap "${live_nav_max_x}" 0.30

  cat <<EOF
GO2_XT16_SPORT_LIVE_READINESS=PASS
SPORT_LIVE_READINESS_LOG=${sport_readiness_log}
SPORT_PROBE_RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
SPORT_PROBE_SUMMARY=${sport_probe_summary}
SPORT_PROBE_HAS_MOVE_1008=true
SPORT_PROBE_HAS_STOPMOVE_1003=true
LIVE_NAV_SHORT_GOAL_STATUS=PASS
LIVE_NAV_SUMMARY=${live_nav_summary}
LIVE_NAV_LOG=${live_nav_log}
LIVE_NAV_MAX_X_MPS=${live_nav_max_x}
NO_RESIDUAL_RUNTIME_STATUS=PASS
NO_RESIDUAL_RUNTIME_LOG=${runtime_clean_log}
EOF
}

run_self_test() {
  local self_path
  local tmp_dir
  local screenshot
  local tf_log
  local raw_log
  local standard_log
  local readiness_log
  local probe_summary
  local probe_echo
  local nav_summary
  local nav_log
  local runtime_log
  local saved_pwd
  local out
  local status

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  screenshot="${tmp_dir}/gate1.png"
  tf_log="${tmp_dir}/gate2_tf.txt"
  raw_log="${tmp_dir}/gate3_raw.txt"
  standard_log="${tmp_dir}/gate3_standard.txt"
  readiness_log="${tmp_dir}/gate4_readiness.txt"
  probe_summary="${tmp_dir}/gate4_probe_summary.env"
  probe_echo="${tmp_dir}/gate4_probe_echo.txt"
  nav_summary="${tmp_dir}/gate4_nav_summary.env"
  nav_log="${tmp_dir}/gate4_nav.log"
  runtime_log="${tmp_dir}/runtime_clean.txt"

  printf '\x89PNG\r\n\x1a\n' >"${screenshot}"
  cat >"${tf_log}" <<'EOF'
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
EOF
  cat >"${raw_log}" <<'EOF'
ODOM_AXIS_TOPIC=/utlidar/robot_odom
ODOM_AXIS_STATUS=PASS
EOF
  cat >"${standard_log}" <<'EOF'
ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard
ODOM_AXIS_STATUS=PASS
EOF
  cat >"${readiness_log}" <<'EOF'
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
EOF
  cat >"${probe_echo}" <<'EOF'
api_id: 1008
api_id: 1003
EOF
  cat >"${probe_summary}" <<EOF
MODE=live
RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
REQUEST_TOPIC=/api/sport/request
REQUEST_ECHO_LOG=${probe_echo}
EOF
  cat >"${nav_summary}" <<'EOF'
MODE=live
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_TOPIC=/api/sport/request
SPORT_MAX_X=0.30
EOF
  cat >"${nav_log}" <<'EOF'
RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING
SUMMARY_LOG=/tmp/gate4_nav_summary.env
EOF
  cat >"${runtime_log}" <<'EOF'
GO2_XT16_RUNTIME_DOCKER_STATUS=PASS
GO2_XT16_RUNTIME_PROCESS_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
EOF

  out="$("${self_path}" --gate gate1 --screenshot "${screenshot}")"
  [[ "${out}" == *"RVIZ_BASE_LINK_FRONT_OBJECT_STATUS=PASS"* ]] || return 1
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE1=PASS"

  out="$("${self_path}" --gate gate2 --tf-health-log "${tf_log}")"
  [[ "${out}" == *"TF_HEALTH_STATUS=PASS"* ]] || return 1
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE2=PASS"

  out="$("${self_path}" --gate gate3 --raw-odom-axis-log "${raw_log}" --standard-odom-axis-log "${standard_log}")"
  [[ "${out}" == *"RAW_STANDARD_ODOM_MATERIAL_MATCH=true"* ]] || return 1
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE3=PASS"

  out="$("${self_path}" --gate gate4 \
    --sport-readiness-log "${readiness_log}" \
    --sport-probe-summary "${probe_summary}" \
    --live-nav-summary "${nav_summary}" \
    --live-nav-log "${nav_log}" \
    --runtime-clean-log "${runtime_log}")"
  [[ "${out}" == *"LIVE_NAV_MAX_X_MPS=0.30"* ]] || return 1
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE4=PASS"

  printf 'not an image\n' >"${screenshot}"
  set +e
  out="$("${self_path}" --gate gate1 --screenshot "${screenshot}" 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: bad screenshot unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"not a PNG or JPEG"* ]] || {
    echo "${out}" >&2
    echo "ERROR: bad screenshot reason missing" >&2
    return 1
  }
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_BAD_ARTIFACT_REJECT=PASS"

  saved_pwd="${PWD}"
  cd "${tmp_dir}"
  set +e
  out="$("${self_path}" --gate gate1 --screenshot gate1.png 2>&1)"
  status=$?
  set -e
  cd "${saved_pwd}"
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: relative artifact path unexpectedly passed" >&2
    return 1
  }
  [[ "${out}" == *"path is not absolute"* ]] || {
    echo "${out}" >&2
    echo "ERROR: relative artifact path reason missing" >&2
    return 1
  }
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_RELATIVE_ARTIFACT_REJECT=PASS"
  echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  if [[ -n "${gate}" ]]; then
    die "--self-test cannot be combined with --gate"
  fi
  run_self_test
  exit 0
fi

case "${gate}" in
  gate1)
    emit_gate1
    ;;
  gate2)
    emit_gate2
    ;;
  gate3)
    emit_gate3
    ;;
  gate4)
    emit_gate4
    ;;
  "")
    die "--gate is required"
    ;;
  *)
    die "unknown gate: ${gate}"
    ;;
esac
