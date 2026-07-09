#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/check_go2_xt16_report_completion.sh [--allow-incomplete]
  scripts/check_go2_xt16_report_completion.sh --self-test

Static completion audit for /home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md.

This script only reads local files and calls the local acceptance-evidence
verifier. It does not source ROS, inspect live robot topics, start Docker,
publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel, or publish /api/sport/request.
EOF
}

allow_incomplete=false
self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --allow-incomplete)
      allow_incomplete=true
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${GO2_XT16_AXIS_TILT_REPORT:-/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md}"
EVIDENCE_FILE="${GO2_XT16_AXIS_TILT_EVIDENCE:-${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env}"
SNAPSHOT_FILE="${GO2_XT16_AXIS_TILT_STATUS_SNAPSHOT:-${WS_ROOT}/run_logs/go2_xt16_axis_tilt/status_snapshot_20260704_current.txt}"
HANDOFF_FILE="${GO2_XT16_AXIS_TILT_MANUAL_HANDOFF:-${WS_ROOT}/run_logs/go2_xt16_axis_tilt/manual_gate_handoff_20260704_current.md}"
VERIFIER="${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh"
GAP_SUMMARY="${WS_ROOT}/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh"
SNAPSHOT_COLLECTOR="${WS_ROOT}/scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh"
HANDOFF_BUILDER="${WS_ROOT}/scripts/build_go2_xt16_axis_tilt_manual_handoff.sh"
STATUS_SYNC="${WS_ROOT}/scripts/sync_go2_xt16_axis_tilt_report_status.sh"
COMPLETION_AUDIT="${WS_ROOT}/scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh"

die() {
  echo "ERROR: $*" >&2
  exit 2
}

write_synthetic_report() {
  local path="$1"
  local acceptance="$2"
  local completion="$3"
  local status_overall="$4"

  cat >"${path}" <<EOF
# Synthetic Go2 XT16 report completion self-test

当前权威状态（2026-07-04）：

\`\`\`text
report shape: PASS
runtime clean: PASS
acceptance: ${acceptance}
completion: ${completion}
\`\`\`

## 18. 2026-07-03 current evidence file

## 19. 2026-07-04 compact status-report mode

\`\`\`text
GO2_XT16_STATUS_REPORT_REPORT_LINK=PASS
GO2_XT16_STATUS_REPORT_OVERALL=${status_overall}
GO2_XT16_EVIDENCE_SCHEMA_STATUS=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_REPORT_LINK_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_ABSOLUTE_PATH_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_SCREENSHOT_TYPE_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_CONTENT_REJECT=PASS
GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=PASS
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_STATUS=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_STATUS=PASS
GO2_XT16_GATE_GAPS_SELFTEST_STATUS=PASS
GO2_XT16_GATE_GAPS_OVERALL=${status_overall}
GO2_XT16_STATUS_SNAPSHOT_SELFTEST_STATUS=PASS
GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS
GO2_XT16_MANUAL_HANDOFF_SELFTEST_STATUS=PASS
GO2_XT16_MANUAL_HANDOFF_WRITE_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_STATUS=PASS
GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_STATUS=PASS
GO2_XT16_REPORT_STATUS_SYNC_RUNTIME_CLEAN=PASS
GO2_XT16_REPORT_STATUS_SYNC_ACCEPTANCE=${status_overall}
GO2_XT16_REPORT_STATUS_SYNC_COMPLETION=${completion}
GO2_XT16_REPORT_STATUS_SYNC_CHANGED=false
GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FINAL_ACCEPTANCE_STATUS=${status_overall}
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REMAINING_GATES=gate1,gate2,gate3,gate4
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE=${completion}
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_STATUS=PASS
\`\`\`

## 20. PASS completion rule

\`\`\`text
GO2_XT16_ACCEPTANCE_STATUS=PASS
\`\`\`

## 21. 2026-07-04 no-motion runtime clean checker

## 22. 2026-07-04 evidence update helper

## 23. 2026-07-04 artifact-to-update builder

## 24. 2026-07-04 synthetic acceptance pipeline self-test

## 25. 2026-07-04 gate evidence gap summary

## 26. 2026-07-04 no-motion status snapshot artifact

## 27. 2026-07-04 manual gate handoff artifact

## 28. 2026-07-04 static artifact refresh runner

## 29. 2026-07-04 report top status sync helper

## 30. 2026-07-04 completion requirements audit
EOF
}

write_synthetic_evidence() {
  local path="$1"
  local report_path="$2"
  local state="$3"
  local artifact_dir
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

  artifact_dir="$(dirname "${path}")"
  screenshot_file="${artifact_dir}/gate1_base_link_front_object.png"
  tf_health_log="${artifact_dir}/gate2_tf_health.txt"
  raw_odom_axis_log="${artifact_dir}/gate3_raw_odom_axis.txt"
  standard_odom_axis_log="${artifact_dir}/gate3_standard_odom_axis.txt"
  sport_readiness_log="${artifact_dir}/gate4_sport_readiness.txt"
  sport_probe_summary="${artifact_dir}/gate4_sport_probe_summary.env"
  sport_probe_echo_log="${artifact_dir}/gate4_sport_probe_echo.log"
  live_nav_summary="${artifact_dir}/gate4_live_nav_summary.env"
  live_nav_log="${artifact_dir}/gate4_live_nav.log"
  runtime_clean_log="${artifact_dir}/gate4_runtime_clean.log"

  if [[ "${state}" == "pass" ]]; then
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

    cat >"${path}" <<EOF
REPORT_PATH=${report_path}
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
  else
    cat >"${path}" <<EOF
REPORT_PATH=${report_path}
RVIZ_BASE_LINK_FRONT_OBJECT_STATUS=NOT_READY
RVIZ_FIXED_FRAME=TODO
RVIZ_FRONT_OBJECT_DIRECTION=TODO
SCREENSHOT=TODO
TF_HEALTH_MAP_ODOM_STATUS=FAIL
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=FAIL
TF_HEALTH_LOG=TODO
RAW_ODOM_AXIS_STATUS=NOT_RUN
STANDARD_ODOM_AXIS_STATUS=NOT_RUN
RAW_STANDARD_ODOM_MATERIAL_MATCH=TODO
ODOM_AXIS_ROTATION_CHECK=TODO
RAW_ODOM_AXIS_LOG=TODO
STANDARD_ODOM_AXIS_LOG=TODO
GO2_XT16_SPORT_LIVE_READINESS=NOT_RUN
SPORT_LIVE_READINESS_LOG=TODO
SPORT_PROBE_RESULT=NOT_RUN
SPORT_PROBE_SUMMARY=TODO
SPORT_PROBE_HAS_MOVE_1008=TODO
SPORT_PROBE_HAS_STOPMOVE_1003=TODO
LIVE_NAV_SHORT_GOAL_STATUS=NOT_RUN
LIVE_NAV_SUMMARY=TODO
LIVE_NAV_LOG=TODO
LIVE_NAV_MAX_X_MPS=TODO
NO_RESIDUAL_RUNTIME_STATUS=PASS
NO_RESIDUAL_RUNTIME_LOG=TODO
EOF
  fi
}

write_synthetic_snapshot() {
  local path="$1"
  local status_overall="$2"

  cat >"${path}" <<EOF
GO2_XT16_STATUS_SNAPSHOT_CREATED_AT=2026-07-04T00:00:00+08:00
GO2_XT16_STATUS_SNAPSHOT_WORKDIR=/tmp/synthetic
GO2_XT16_STATUS_SNAPSHOT_MODE=no_motion_local_static
GO2_XT16_STATUS_REPORT_OVERALL=${status_overall}
GO2_XT16_GATE_GAPS_OVERALL=${status_overall}
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS
EOF
}

write_synthetic_handoff() {
  local path="$1"
  local status_overall="$2"

  cat >"${path}" <<EOF
# Synthetic handoff

GO2_XT16_STATUS_REPORT_OVERALL=${status_overall}
GO2_XT16_GATE_GAPS_OVERALL=${status_overall}

\`\`\`text
/cmd_vel
/dddmr_go2/dry_run_cmd_vel
/api/sport/request
/lowcmd
GO2_XT16_ACCEPTANCE_STATUS=PASS
GO2_XT16_REPORT_COMPLETION_STATUS=PASS
\`\`\`
EOF
}

run_self_test() {
  local self_path
  local tmp_dir
  local pass_report
  local pass_evidence
  local pass_snapshot
  local pass_handoff
  local not_ready_report
  local not_ready_evidence
  local not_ready_snapshot
  local not_ready_handoff
  local out
  local status

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN

  pass_report="${tmp_dir}/synthetic_pass_report.md"
  pass_evidence="${tmp_dir}/synthetic_pass_evidence.env"
  pass_snapshot="${tmp_dir}/synthetic_pass_snapshot.txt"
  pass_handoff="${tmp_dir}/synthetic_pass_handoff.md"
  not_ready_report="${tmp_dir}/synthetic_not_ready_report.md"
  not_ready_evidence="${tmp_dir}/synthetic_not_ready_evidence.env"
  not_ready_snapshot="${tmp_dir}/synthetic_not_ready_snapshot.txt"
  not_ready_handoff="${tmp_dir}/synthetic_not_ready_handoff.md"

  write_synthetic_report "${pass_report}" PASS PASS PASS
  write_synthetic_evidence "${pass_evidence}" "${pass_report}" pass
  write_synthetic_snapshot "${pass_snapshot}" PASS
  write_synthetic_handoff "${pass_handoff}" PASS
  out="$(
    GO2_XT16_AXIS_TILT_REPORT="${pass_report}" \
    GO2_XT16_AXIS_TILT_EVIDENCE="${pass_evidence}" \
    GO2_XT16_AXIS_TILT_STATUS_SNAPSHOT="${pass_snapshot}" \
    GO2_XT16_AXIS_TILT_MANUAL_HANDOFF="${pass_handoff}" \
    "${self_path}"
  )"
  [[ "${out}" == *"GO2_XT16_REPORT_COMPLETION_STATUS=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic PASS report did not complete" >&2
    return 1
  }
  echo "GO2_XT16_REPORT_COMPLETION_SELFTEST_SYNTHETIC_PASS=PASS"

  write_synthetic_report "${not_ready_report}" NOT_READY NOT_READY NOT_READY
  write_synthetic_evidence "${not_ready_evidence}" "${not_ready_report}" not_ready
  write_synthetic_snapshot "${not_ready_snapshot}" NOT_READY
  write_synthetic_handoff "${not_ready_handoff}" NOT_READY
  out="$(
    GO2_XT16_AXIS_TILT_REPORT="${not_ready_report}" \
    GO2_XT16_AXIS_TILT_EVIDENCE="${not_ready_evidence}" \
    GO2_XT16_AXIS_TILT_STATUS_SNAPSHOT="${not_ready_snapshot}" \
    GO2_XT16_AXIS_TILT_MANUAL_HANDOFF="${not_ready_handoff}" \
    "${self_path}" --allow-incomplete
  )"
  [[ "${out}" == *"GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY"* ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic NOT_READY report was not reported as NOT_READY" >&2
    return 1
  }
  echo "GO2_XT16_REPORT_COMPLETION_SELFTEST_SYNTHETIC_NOT_READY=PASS"

  set +e
  out="$(
    GO2_XT16_AXIS_TILT_REPORT="${not_ready_report}" \
    GO2_XT16_AXIS_TILT_EVIDENCE="${not_ready_evidence}" \
    GO2_XT16_AXIS_TILT_STATUS_SNAPSHOT="${not_ready_snapshot}" \
    GO2_XT16_AXIS_TILT_MANUAL_HANDOFF="${not_ready_handoff}" \
    "${self_path}" 2>&1
  )"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: synthetic NOT_READY report unexpectedly exited 0 without --allow-incomplete" >&2
    return 1
  }
  echo "GO2_XT16_REPORT_COMPLETION_SELFTEST_INCOMPLETE_EXIT=PASS"

  echo "GO2_XT16_REPORT_COMPLETION_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  if [[ "${allow_incomplete}" == "true" ]]; then
    echo "ERROR: --self-test cannot be combined with --allow-incomplete" >&2
    exit 2
  fi
  run_self_test
  exit 0
fi

require_text() {
  local file="$1"
  local text="$2"
  local label="$3"

  if ! grep -Fq -- "${text}" "${file}"; then
    echo "GO2_XT16_REPORT_${label}=MISSING"
    return 1
  fi
  echo "GO2_XT16_REPORT_${label}=PASS"
}

get_evidence_value() {
  local key="$1"

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
  ' "${EVIDENCE_FILE}" || true
}

[[ -f "${REPORT_PATH}" ]] || die "missing report: ${REPORT_PATH}"
[[ -f "${EVIDENCE_FILE}" ]] || die "missing evidence file: ${EVIDENCE_FILE}"
[[ -x "${VERIFIER}" ]] || die "missing executable verifier: ${VERIFIER}"
[[ -x "${GAP_SUMMARY}" ]] || die "missing executable gate gap summary: ${GAP_SUMMARY}"
[[ -x "${SNAPSHOT_COLLECTOR}" ]] || die "missing executable status snapshot collector: ${SNAPSHOT_COLLECTOR}"
[[ -x "${HANDOFF_BUILDER}" ]] || die "missing executable manual handoff builder: ${HANDOFF_BUILDER}"
[[ -x "${STATUS_SYNC}" ]] || die "missing executable report status sync helper: ${STATUS_SYNC}"
[[ -x "${COMPLETION_AUDIT}" ]] || die "missing executable completion requirements audit: ${COMPLETION_AUDIT}"

echo "GO2_XT16_REPORT_PATH=${REPORT_PATH}"
echo "GO2_XT16_REPORT_EVIDENCE_FILE=${EVIDENCE_FILE}"
echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_FILE=${SNAPSHOT_FILE}"
echo "GO2_XT16_REPORT_MANUAL_HANDOFF_FILE=${HANDOFF_FILE}"
echo "GO2_XT16_REPORT_STATUS_SYNC_HELPER=${STATUS_SYNC}"
echo "GO2_XT16_REPORT_COMPLETION_REQUIREMENTS_AUDIT_HELPER=${COMPLETION_AUDIT}"
echo "GO2_XT16_REPORT_FILE_STATUS=PASS"

status_out="$(
  GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
    "${VERIFIER}" --evidence "${EVIDENCE_FILE}" --status-report --allow-incomplete
)"
overall="$(
  printf '%s\n' "${status_out}" | awk -F= '$1 == "GO2_XT16_STATUS_REPORT_OVERALL" {print $2; exit}'
)"
[[ -n "${overall}" ]] || die "verifier did not print GO2_XT16_STATUS_REPORT_OVERALL"
gap_out="$("${GAP_SUMMARY}" --evidence "${EVIDENCE_FILE}")"
gap_overall="$(
  printf '%s\n' "${gap_out}" | awk -F= '$1 == "GO2_XT16_GATE_GAPS_OVERALL" {print $2; exit}'
)"
[[ -n "${gap_overall}" ]] || die "gate gap summary did not print GO2_XT16_GATE_GAPS_OVERALL"
status_sync_out="$("${STATUS_SYNC}" --dry-run)"
status_sync_status="$(
  printf '%s\n' "${status_sync_out}" | awk -F= '$1 == "GO2_XT16_REPORT_STATUS_SYNC_STATUS" {print $2; exit}'
)"
status_sync_changed="$(
  printf '%s\n' "${status_sync_out}" | awk -F= '$1 == "GO2_XT16_REPORT_STATUS_SYNC_CHANGED" {print $2; exit}'
)"
[[ "${status_sync_status}" == "PASS" ]] || die "report status sync helper did not return PASS"
[[ -n "${status_sync_changed}" ]] || die "report status sync helper did not print changed status"
completion_audit_out="$(
  GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
  GO2_XT16_AXIS_TILT_EVIDENCE="${EVIDENCE_FILE}" \
    "${COMPLETION_AUDIT}"
)"
completion_audit_ready="$(
  printf '%s\n' "${completion_audit_out}" | awk -F= '$1 == "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE" {print $2; exit}'
)"
completion_audit_status="$(
  printf '%s\n' "${completion_audit_out}" | awk -F= '$1 == "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS" {print $2; exit}'
)"
[[ "${completion_audit_status}" == "PASS" ]] || die "completion requirements audit did not return PASS"
[[ -n "${completion_audit_ready}" ]] || die "completion requirements audit did not print ready status"

expected_completion="NOT_READY"
if [[ "${overall}" == "PASS" ]]; then
  expected_completion="PASS"
fi

report_shape="PASS"
require_text "${REPORT_PATH}" "当前权威状态（2026-07-04）" "TOP_AUTHORITY_BLOCK" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "report shape: PASS" "TOP_REPORT_SHAPE_STATUS" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "runtime clean: PASS" "TOP_RUNTIME_CLEAN_STATUS" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "acceptance: ${overall}" "TOP_ACCEPTANCE_STATUS" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "completion: ${expected_completion}" "TOP_COMPLETION_STATUS" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 18. 2026-07-03 current evidence file" "SECTION_18_CURRENT_EVIDENCE" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 19. 2026-07-04 compact status-report mode" "SECTION_19_STATUS_REPORT" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 21. 2026-07-04 no-motion runtime clean checker" "SECTION_21_RUNTIME_CLEAN" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 22. 2026-07-04 evidence update helper" "SECTION_22_EVIDENCE_UPDATER" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 23. 2026-07-04 artifact-to-update builder" "SECTION_23_GATE_UPDATE_BUILDER" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 24. 2026-07-04 synthetic acceptance pipeline self-test" "SECTION_24_ACCEPTANCE_PIPELINE" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 25. 2026-07-04 gate evidence gap summary" "SECTION_25_GATE_GAPS" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 26. 2026-07-04 no-motion status snapshot artifact" "SECTION_26_STATUS_SNAPSHOT" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 27. 2026-07-04 manual gate handoff artifact" "SECTION_27_MANUAL_HANDOFF" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 28. 2026-07-04 static artifact refresh runner" "SECTION_28_STATIC_REFRESH" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 29. 2026-07-04 report top status sync helper" "SECTION_29_STATUS_SYNC" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "## 30. 2026-07-04 completion requirements audit" "SECTION_30_COMPLETION_REQUIREMENTS_AUDIT" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_STATUS_REPORT_REPORT_LINK=PASS" "STATUS_REPORT_LINK_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_STATUS_REPORT_OVERALL=${overall}" "CURRENT_STATUS_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_GATE_GAPS_OVERALL=${gap_overall}" "CURRENT_GATE_GAPS_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS" "STATUS_SNAPSHOT_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_MANUAL_HANDOFF_WRITE_STATUS=PASS" "MANUAL_HANDOFF_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_STATUS=PASS" "PASS_COMPLETION_RULE_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_EVIDENCE_SCHEMA_STATUS=PASS" "EVIDENCE_SCHEMA_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_SELFTEST_REPORT_LINK_REJECT=PASS" "REPORT_LINK_REJECT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_REJECT=PASS" "ARTIFACT_REJECT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_SELFTEST_ABSOLUTE_PATH_REJECT=PASS" "ABSOLUTE_PATH_REJECT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_SELFTEST_SCREENSHOT_TYPE_REJECT=PASS" "SCREENSHOT_TYPE_REJECT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_CONTENT_REJECT=PASS" "ARTIFACT_CONTENT_REJECT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=PASS" "EVIDENCE_RUNTIME_LOG_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_EVIDENCE_UPDATER_SELFTEST_STATUS=PASS" "EVIDENCE_UPDATER_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_STATUS=PASS" "GATE_UPDATE_BUILDER_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_GATE_GAPS_SELFTEST_STATUS=PASS" "GATE_GAPS_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_STATUS_SNAPSHOT_SELFTEST_STATUS=PASS" "STATUS_SNAPSHOT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_MANUAL_HANDOFF_SELFTEST_STATUS=PASS" "MANUAL_HANDOFF_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_STATUS=PASS" "STATIC_ARTIFACT_REFRESH_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_STATIC_ARTIFACT_REFRESH_STATUS=PASS" "STATIC_ARTIFACT_REFRESH_STATUS_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_STATUS=PASS" "STATUS_SYNC_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_REPORT_STATUS_SYNC_ACCEPTANCE=${overall}" "STATUS_SYNC_ACCEPTANCE_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_REPORT_STATUS_SYNC_COMPLETION=${expected_completion}" "STATUS_SYNC_COMPLETION_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_REPORT_STATUS_SYNC_CHANGED=false" "STATUS_SYNC_UNCHANGED_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS" "STATUS_SYNC_STATUS_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_STATUS=PASS" "COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FINAL_ACCEPTANCE_STATUS=${overall}" "COMPLETION_REQUIREMENTS_AUDIT_ACCEPTANCE_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE=${expected_completion}" "COMPLETION_REQUIREMENTS_AUDIT_READY_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS=PASS" "COMPLETION_REQUIREMENTS_AUDIT_STATUS_RECORDED" || report_shape="NOT_READY"
require_text "${REPORT_PATH}" "GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_STATUS=PASS" "ACCEPTANCE_PIPELINE_SELFTEST_RECORDED" || report_shape="NOT_READY"

if [[ "${status_sync_changed}" == "false" ]]; then
  echo "GO2_XT16_REPORT_STATUS_SYNC_CURRENT_STATUS=PASS"
else
  echo "GO2_XT16_REPORT_STATUS_SYNC_CURRENT_STATUS=NOT_READY"
  echo "GO2_XT16_REPORT_STATUS_SYNC_CURRENT_REASON=top_authority_block_out_of_sync"
  report_shape="NOT_READY"
fi
if [[ "${completion_audit_ready}" == "${expected_completion}" ]]; then
  echo "GO2_XT16_REPORT_COMPLETION_REQUIREMENTS_AUDIT_CURRENT_STATUS=PASS"
else
  echo "GO2_XT16_REPORT_COMPLETION_REQUIREMENTS_AUDIT_CURRENT_STATUS=NOT_READY"
  echo "GO2_XT16_REPORT_COMPLETION_REQUIREMENTS_AUDIT_CURRENT_REASON=ready_status_mismatch"
  report_shape="NOT_READY"
fi

evidence_report_path="$(get_evidence_value REPORT_PATH)"
if [[ "${evidence_report_path}" == "${REPORT_PATH}" ]]; then
  echo "GO2_XT16_REPORT_EVIDENCE_REPORT_PATH_STATUS=PASS"
else
  echo "GO2_XT16_REPORT_EVIDENCE_REPORT_PATH_STATUS=NOT_READY"
  echo "GO2_XT16_REPORT_EVIDENCE_REPORT_PATH_VALUE=${evidence_report_path:-MISSING}"
  report_shape="NOT_READY"
fi

evidence_runtime_status="$(get_evidence_value NO_RESIDUAL_RUNTIME_STATUS)"
if [[ "${evidence_runtime_status}" == "PASS" ]]; then
  echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_STATUS=PASS"
  evidence_runtime_log="$(get_evidence_value NO_RESIDUAL_RUNTIME_LOG)"
  if [[ -z "${evidence_runtime_log}" || "${evidence_runtime_log}" == "TODO" ]]; then
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_VALUE=${evidence_runtime_log:-MISSING}"
    report_shape="NOT_READY"
  elif [[ "${evidence_runtime_log}" != /* ]]; then
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_VALUE=${evidence_runtime_log}"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_REASON=not_absolute"
    report_shape="NOT_READY"
  elif [[ ! -s "${evidence_runtime_log}" ]]; then
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_VALUE=${evidence_runtime_log}"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_REASON=missing_or_empty_file"
    report_shape="NOT_READY"
  elif ! grep -Fq "GO2_XT16_RUNTIME_DOCKER_STATUS=PASS" "${evidence_runtime_log}"; then
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_REASON=missing_docker_pass"
    report_shape="NOT_READY"
  elif ! grep -Fq "GO2_XT16_RUNTIME_PROCESS_STATUS=PASS" "${evidence_runtime_log}"; then
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_REASON=missing_process_pass"
    report_shape="NOT_READY"
  elif ! grep -Fq "GO2_XT16_RUNTIME_CLEAN_STATUS=PASS" "${evidence_runtime_log}"; then
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_REASON=missing_clean_pass"
    report_shape="NOT_READY"
  else
    echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=PASS"
  fi
else
  echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_STATUS=NOT_READY"
  echo "GO2_XT16_REPORT_EVIDENCE_RUNTIME_VALUE=${evidence_runtime_status:-MISSING}"
  report_shape="NOT_READY"
fi

if [[ ! -s "${SNAPSHOT_FILE}" ]]; then
  echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_FILE_STATUS=NOT_READY"
  echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_REASON=missing_or_empty_file"
  report_shape="NOT_READY"
else
  echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_FILE_STATUS=PASS"
  if ! grep -Fxq "GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS" "${SNAPSHOT_FILE}"; then
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_REASON=missing_snapshot_pass"
    report_shape="NOT_READY"
  elif ! grep -Fxq "GO2_XT16_STATUS_REPORT_OVERALL=${overall}" "${SNAPSHOT_FILE}"; then
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_REASON=status_report_overall_mismatch"
    report_shape="NOT_READY"
  elif ! grep -Fxq "GO2_XT16_GATE_GAPS_OVERALL=${gap_overall}" "${SNAPSHOT_FILE}"; then
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_REASON=gate_gap_overall_mismatch"
    report_shape="NOT_READY"
  elif ! grep -Fxq "GO2_XT16_RUNTIME_CLEAN_STATUS=PASS" "${SNAPSHOT_FILE}"; then
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_REASON=runtime_clean_missing_or_fail"
    report_shape="NOT_READY"
  else
    echo "GO2_XT16_REPORT_STATUS_SNAPSHOT_STATUS=PASS"
  fi
fi

if [[ ! -s "${HANDOFF_FILE}" ]]; then
  echo "GO2_XT16_REPORT_MANUAL_HANDOFF_FILE_STATUS=NOT_READY"
  echo "GO2_XT16_REPORT_MANUAL_HANDOFF_REASON=missing_or_empty_file"
  report_shape="NOT_READY"
else
  echo "GO2_XT16_REPORT_MANUAL_HANDOFF_FILE_STATUS=PASS"
  if ! grep -Fxq "GO2_XT16_STATUS_REPORT_OVERALL=${overall}" "${HANDOFF_FILE}"; then
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_REASON=status_report_overall_mismatch"
    report_shape="NOT_READY"
  elif ! grep -Fxq "GO2_XT16_GATE_GAPS_OVERALL=${gap_overall}" "${HANDOFF_FILE}"; then
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_REASON=gate_gap_overall_mismatch"
    report_shape="NOT_READY"
  elif ! grep -Fq "GO2_XT16_ACCEPTANCE_STATUS=PASS" "${HANDOFF_FILE}"; then
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_REASON=missing_final_acceptance_condition"
    report_shape="NOT_READY"
  elif ! grep -Fq "/api/sport/request" "${HANDOFF_FILE}"; then
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_STATUS=NOT_READY"
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_REASON=missing_safety_motion_interface_marker"
    report_shape="NOT_READY"
  else
    echo "GO2_XT16_REPORT_MANUAL_HANDOFF_STATUS=PASS"
  fi
fi

printf '%s\n' "${status_out}"
printf '%s\n' "${gap_out}"
printf '%s\n' "${status_sync_out}"
printf '%s\n' "${completion_audit_out}"

echo "GO2_XT16_REPORT_SHAPE_STATUS=${report_shape}"
echo "GO2_XT16_REPORT_ACCEPTANCE_STATUS=${overall}"

completion="PASS"
reason="all_required_evidence_present"
if [[ "${report_shape}" != "PASS" ]]; then
  completion="NOT_READY"
  reason="report_missing_required_completion_audit_text"
elif [[ "${overall}" != "PASS" ]]; then
  completion="NOT_READY"
  reason="acceptance_evidence_not_ready"
fi

echo "GO2_XT16_REPORT_COMPLETION_STATUS=${completion}"
echo "GO2_XT16_REPORT_COMPLETION_REASON=${reason}"

if [[ "${completion}" != "PASS" && "${allow_incomplete}" != "true" ]]; then
  exit 1
fi
