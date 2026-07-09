#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh [--require-complete]
  scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh \
    [--report FILE] [--evidence FILE] [--require-complete]
  scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh --self-test

Print a local, no-motion requirement-to-evidence audit for the Go2 XT16
axis/tilt report. The acceptance evidence verifier remains the authority for
PASS/NOT_READY; this script only makes the completion requirements explicit.

This script parses files as data. It does not source ROS, inspect live robot
topics, start Docker, publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel,
publish /api/sport/request, or publish /lowcmd.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${GO2_XT16_AXIS_TILT_REPORT:-/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md}"
EVIDENCE_FILE="${GO2_XT16_AXIS_TILT_EVIDENCE:-${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env}"
VERIFIER="${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh"
GAP_SUMMARY="${WS_ROOT}/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh"

require_complete=false
self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --report)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --report requires a file" >&2
        exit 2
      }
      REPORT_PATH="$2"
      shift 2
      ;;
    --evidence)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --evidence requires a file" >&2
        exit 2
      }
      EVIDENCE_FILE="$2"
      shift 2
      ;;
    --require-complete)
      require_complete=true
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

die() {
  echo "ERROR: $*" >&2
  exit 2
}

kv_get() {
  local data="$1"
  local key="$2"

  printf '%s\n' "${data}" | awk -F= -v key="${key}" '
    $1 == key {
      sub(/^[^=]*=/, "")
      print
      found = 1
      exit
    }
    END {
      if (!found) {
        exit 1
      }
    }
  ' || true
}

emit_gate_audit() {
  local gate_name="$1"
  local required="$2"
  local status_key="$3"
  local reason_key="$4"
  local missing_key="$5"
  local status_out="$6"
  local gap_out="$7"
  local status
  local reason
  local missing

  status="$(kv_get "${status_out}" "${status_key}")"
  reason="$(kv_get "${status_out}" "${reason_key}")"
  missing="$(kv_get "${gap_out}" "${missing_key}")"
  [[ -n "${status}" ]] || status="UNKNOWN"
  [[ -n "${reason}" ]] || reason="none"
  [[ -n "${missing}" ]] || missing="unknown"

  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_${gate_name}_REQUIRED=${required}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_${gate_name}_STATUS=${status}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_${gate_name}_REASON=${reason}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_${gate_name}_MISSING=${missing}"
}

emit_audit() {
  local status_out
  local gap_out
  local overall
  local remaining_gates
  local ready
  local reason

  [[ -f "${REPORT_PATH}" ]] || die "missing report: ${REPORT_PATH}"
  [[ -f "${EVIDENCE_FILE}" ]] || die "missing evidence file: ${EVIDENCE_FILE}"
  [[ -x "${VERIFIER}" ]] || die "missing executable verifier: ${VERIFIER}"
  [[ -x "${GAP_SUMMARY}" ]] || die "missing executable gap summary: ${GAP_SUMMARY}"

  status_out="$(
    GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
      "${VERIFIER}" --evidence "${EVIDENCE_FILE}" --status-report --allow-incomplete
  )"
  gap_out="$("${GAP_SUMMARY}" --evidence "${EVIDENCE_FILE}")"
  overall="$(kv_get "${status_out}" GO2_XT16_STATUS_REPORT_OVERALL)"
  remaining_gates="$(kv_get "${gap_out}" GO2_XT16_GATE_GAPS_REMAINING_GATES)"
  [[ -n "${overall}" ]] || die "verifier did not print GO2_XT16_STATUS_REPORT_OVERALL"
  [[ -n "${remaining_gates}" ]] || die "gap summary did not print remaining gates"

  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REPORT=${REPORT_PATH}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_EVIDENCE=${EVIDENCE_FILE}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_MODE=no_motion_local_static"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FORBIDDEN_INTERFACES=/cmd_vel,/dddmr_go2/dry_run_cmd_vel,/api/sport/request,/lowcmd"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REQUIRED_GATES=gate1,gate2,gate3,gate4"

  emit_gate_audit \
    GATE1_RVIZ_FRONT_OBJECT \
    RViz_base_link_known_front_object_PASS \
    GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT \
    GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT_REASON \
    GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_MISSING \
    "${status_out}" \
    "${gap_out}"
  emit_gate_audit \
    GATE2_INITIAL_POSE_TF_TILT \
    initial_pose_then_map_odom_and_map_base_TF_tilt_PASS \
    GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT \
    GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT_REASON \
    GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_MISSING \
    "${status_out}" \
    "${gap_out}"
  emit_gate_audit \
    GATE3_ODOM_AXIS \
    raw_and_standard_odom_axis_probe_PASS \
    GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS \
    GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS_REASON \
    GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_MISSING \
    "${status_out}" \
    "${gap_out}"
  emit_gate_audit \
    GATE4_SUPERVISED_LIVE_SHORT_GOAL \
    supervised_live_short_goal_under_0p30mps_cap_PASS \
    GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL \
    GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REASON \
    GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_MISSING \
    "${status_out}" \
    "${gap_out}"

  ready="NOT_READY"
  reason="manual_or_supervised_gate_evidence_incomplete"
  if [[ "${overall}" == "PASS" ]]; then
    ready="PASS"
    reason="all_required_gate_evidence_verified"
  fi

  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FINAL_ACCEPTANCE_STATUS=${overall}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REMAINING_GATES=${remaining_gates}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE=${ready}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REASON=${reason}"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS=PASS"

  if [[ "${ready}" != "PASS" && "${require_complete}" == "true" ]]; then
    return 1
  fi
}

run_self_test() {
  local self_path
  local out
  local require_out
  local ready
  local status

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  out="$("${self_path}")"
  [[ "${out}" == *"GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: audit status marker missing" >&2
    return 1
  }
  [[ "${out}" == *"GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REQUIRED_GATES=gate1,gate2,gate3,gate4"* ]] || {
    echo "${out}" >&2
    echo "ERROR: required-gates marker missing" >&2
    return 1
  }
  ready="$(kv_get "${out}" GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE)"
  [[ "${ready}" == "PASS" || "${ready}" == "NOT_READY" ]] || {
    echo "${out}" >&2
    echo "ERROR: invalid ready-to-mark-complete status: ${ready}" >&2
    return 1
  }

  set +e
  require_out="$("${self_path}" --require-complete 2>&1)"
  status=$?
  set -e
  if [[ "${ready}" == "PASS" ]]; then
    [[ "${status}" -eq 0 ]] || {
      echo "${require_out}" >&2
      echo "ERROR: --require-complete failed although audit is PASS" >&2
      return 1
    }
  else
    [[ "${status}" -ne 0 ]] || {
      echo "${require_out}" >&2
      echo "ERROR: --require-complete succeeded although audit is NOT_READY" >&2
      return 1
    }
  fi

  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_CURRENT_RUN=PASS"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_REQUIRE_COMPLETE_BEHAVIOR=PASS"
  echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  [[ "${require_complete}" == "false" ]] || die "--self-test cannot be combined with --require-complete"
  run_self_test
  exit 0
fi

emit_audit
