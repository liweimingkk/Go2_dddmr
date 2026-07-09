#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/check_go2_xt16_acceptance_pipeline_selftest.sh

End-to-end local self-test for the Go2 XT16 axis/tilt acceptance evidence
pipeline:

  synthetic artifacts -> gate update builder -> evidence updater -> verifier PASS

The test uses only temporary local files and does not modify the current
evidence file.

This script does not source ROS, inspect live robot topics, start Docker,
publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel, publish /api/sport/request,
or publish /lowcmd.
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
BUILDER="${WS_ROOT}/scripts/build_go2_xt16_axis_tilt_gate_update.sh"
UPDATER="${WS_ROOT}/scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh"
VERIFIER="${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh"
TEMPLATE="${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env.example"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_file() {
  local path="$1"
  [[ -f "${path}" ]] || die "missing file: ${path}"
}

for path in "${BUILDER}" "${UPDATER}" "${VERIFIER}" "${TEMPLATE}"; do
  require_file "${path}"
done
[[ -x "${BUILDER}" ]] || die "builder is not executable: ${BUILDER}"
[[ -x "${UPDATER}" ]] || die "updater is not executable: ${UPDATER}"
[[ -x "${VERIFIER}" ]] || die "verifier is not executable: ${VERIFIER}"

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

evidence="${tmp_dir}/synthetic_evidence.env"
screenshot="${tmp_dir}/gate1_base_link_front_object.png"
tf_log="${tmp_dir}/gate2_tf_health.txt"
raw_log="${tmp_dir}/gate3_raw_odom_axis.txt"
standard_log="${tmp_dir}/gate3_standard_odom_axis.txt"
readiness_log="${tmp_dir}/gate4_sport_readiness.txt"
probe_summary="${tmp_dir}/gate4_sport_probe_summary.env"
probe_echo="${tmp_dir}/gate4_sport_probe_echo.txt"
live_summary="${tmp_dir}/gate4_live_nav_summary.env"
live_log="${tmp_dir}/gate4_live_nav.log"
runtime_log="${tmp_dir}/gate4_runtime_clean.txt"
gate1_update="${tmp_dir}/gate1_update.env"
gate2_update="${tmp_dir}/gate2_update.env"
gate3_update="${tmp_dir}/gate3_update.env"
gate4_update="${tmp_dir}/gate4_update.env"

cp "${TEMPLATE}" "${evidence}"
printf '\x89PNG\r\n\x1a\n' >"${screenshot}"
cat >"${tf_log}" <<'EOF'
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
EOF
cat >"${raw_log}" <<'EOF'
ODOM_AXIS_TOPIC=/utlidar/robot_odom
ODOM_AXIS_FORWARD_OK=true
ODOM_AXIS_HEADING_OK=true
ODOM_AXIS_LATERAL_OK=true
ODOM_AXIS_STATUS=PASS
EOF
cat >"${standard_log}" <<'EOF'
ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard
ODOM_AXIS_FORWARD_OK=true
ODOM_AXIS_HEADING_OK=true
ODOM_AXIS_LATERAL_OK=true
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
cat >"${live_summary}" <<'EOF'
MODE=live
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_TOPIC=/api/sport/request
SPORT_MAX_X=0.30
EOF
cat >"${live_log}" <<EOF
RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING
SUMMARY_LOG=${live_summary}
EOF
cat >"${runtime_log}" <<'EOF'
GO2_XT16_RUNTIME_DOCKER_STATUS=PASS
GO2_XT16_RUNTIME_PROCESS_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
EOF

"${BUILDER}" --gate gate1 --screenshot "${screenshot}" >"${gate1_update}"
"${BUILDER}" --gate gate2 --tf-health-log "${tf_log}" >"${gate2_update}"
"${BUILDER}" --gate gate3 \
  --raw-odom-axis-log "${raw_log}" \
  --standard-odom-axis-log "${standard_log}" >"${gate3_update}"
"${BUILDER}" --gate gate4 \
  --sport-readiness-log "${readiness_log}" \
  --sport-probe-summary "${probe_summary}" \
  --live-nav-summary "${live_summary}" \
  --live-nav-log "${live_log}" \
  --runtime-clean-log "${runtime_log}" >"${gate4_update}"
echo "GO2_XT16_ACCEPTANCE_PIPELINE_BUILDER_STATUS=PASS"

"${UPDATER}" --evidence "${evidence}" --update "${gate1_update}" >/dev/null
"${UPDATER}" --evidence "${evidence}" --update "${gate2_update}" >/dev/null
"${UPDATER}" --evidence "${evidence}" --update "${gate3_update}" >/dev/null
"${UPDATER}" --evidence "${evidence}" --update "${gate4_update}" >/dev/null
echo "GO2_XT16_ACCEPTANCE_PIPELINE_UPDATER_STATUS=PASS"

verifier_out="$("${VERIFIER}" --evidence "${evidence}")"
printf '%s\n' "${verifier_out}" | grep -Fxq "GO2_XT16_ACCEPTANCE_STATUS=PASS" || {
  printf '%s\n' "${verifier_out}" >&2
  die "synthetic pipeline evidence did not reach acceptance PASS"
}
echo "GO2_XT16_ACCEPTANCE_PIPELINE_VERIFIER_STATUS=PASS"

status_out="$("${VERIFIER}" --evidence "${evidence}" --status-report)"
printf '%s\n' "${status_out}" | grep -Fxq "GO2_XT16_STATUS_REPORT_OVERALL=PASS" || {
  printf '%s\n' "${status_out}" >&2
  die "synthetic pipeline evidence did not reach status-report PASS"
}
echo "GO2_XT16_ACCEPTANCE_PIPELINE_STATUS_REPORT=PASS"
echo "GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_STATUS=PASS"
