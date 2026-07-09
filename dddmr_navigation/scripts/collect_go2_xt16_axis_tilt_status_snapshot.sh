#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh [--output ABSOLUTE_FILE]
  scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh --self-test

Collect a local no-motion status snapshot for the Go2 XT16 axis/tilt report.
The snapshot is an artifact bundle in text form: current verifier status,
gate-gap summary, completion requirements audit, report completion audit, and
residual runtime clean check.

This script only calls local no-motion checkers. It does not source ROS, inspect
live robot topics, start Docker, publish /cmd_vel, publish
/dddmr_go2/dry_run_cmd_vel, publish /api/sport/request, or publish /lowcmd.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${GO2_XT16_AXIS_TILT_REPORT:-/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md}"
EVIDENCE_FILE="${GO2_XT16_AXIS_TILT_EVIDENCE:-${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env}"
VERIFIER="${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh"
GAP_SUMMARY="${WS_ROOT}/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh"
COMPLETION_AUDIT="${WS_ROOT}/scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh"
REPORT_CHECKER="${WS_ROOT}/scripts/check_go2_xt16_report_completion.sh"
RUNTIME_CLEAN="${WS_ROOT}/scripts/check_go2_xt16_no_motion_runtime_clean.sh"

output_file=""
self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      [[ $# -ge 2 ]] || {
        echo "ERROR: --output requires a file" >&2
        exit 2
      }
      output_file="$2"
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

require_executable() {
  local path="$1"
  [[ -x "${path}" ]] || die "missing executable: ${path}"
}

require_readable() {
  local path="$1"
  [[ -r "${path}" ]] || die "missing readable file: ${path}"
}

emit_snapshot() {
  require_readable "${REPORT_PATH}"
  require_readable "${EVIDENCE_FILE}"
  require_executable "${VERIFIER}"
  require_executable "${GAP_SUMMARY}"
  require_executable "${COMPLETION_AUDIT}"
  require_executable "${REPORT_CHECKER}"
  require_executable "${RUNTIME_CLEAN}"

  echo "GO2_XT16_STATUS_SNAPSHOT_CREATED_AT=$(date -Is)"
  echo "GO2_XT16_STATUS_SNAPSHOT_WORKDIR=${WS_ROOT}"
  echo "GO2_XT16_STATUS_SNAPSHOT_REPORT=${REPORT_PATH}"
  echo "GO2_XT16_STATUS_SNAPSHOT_EVIDENCE=${EVIDENCE_FILE}"
  echo "GO2_XT16_STATUS_SNAPSHOT_MODE=no_motion_local_static"
  echo "GO2_XT16_STATUS_SNAPSHOT_FORBIDDEN_INTERFACES=/cmd_vel,/dddmr_go2/dry_run_cmd_vel,/api/sport/request,/lowcmd"

  echo "=== acceptance status-report"
  GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
    "${VERIFIER}" --evidence "${EVIDENCE_FILE}" --status-report --allow-incomplete

  echo "=== gate evidence gaps"
  "${GAP_SUMMARY}" --evidence "${EVIDENCE_FILE}"

  echo "=== completion requirements audit"
  GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
  GO2_XT16_AXIS_TILT_EVIDENCE="${EVIDENCE_FILE}" \
    "${COMPLETION_AUDIT}"

  echo "=== report completion"
  GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
  GO2_XT16_AXIS_TILT_EVIDENCE="${EVIDENCE_FILE}" \
    "${REPORT_CHECKER}" --allow-incomplete

  echo "=== residual runtime"
  "${RUNTIME_CLEAN}"

  echo "GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS"
}

write_snapshot() {
  local path="$1"
  local dir
  local tmp_file

  [[ "${path}" == /* ]] || die "--output must be an absolute path: ${path}"
  dir="$(dirname "${path}")"
  mkdir -p "${dir}"
  tmp_file="$(mktemp "${dir}/.snapshot_tmp.XXXXXX")"
  emit_snapshot >"${tmp_file}"
  mv "${tmp_file}" "${path}"
  echo "GO2_XT16_STATUS_SNAPSHOT_OUTPUT=${path}"
  echo "GO2_XT16_STATUS_SNAPSHOT_WRITE_STATUS=PASS"
}

run_self_test() {
  local self_path
  local tmp_dir
  local snapshot_file
  local out

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  snapshot_file="${tmp_dir}/snapshot.txt"

  out="$("${self_path}" --output "${snapshot_file}")"
  [[ "${out}" == *"GO2_XT16_STATUS_SNAPSHOT_WRITE_STATUS=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: snapshot write status missing" >&2
    return 1
  }
  [[ -s "${snapshot_file}" ]] || {
    echo "ERROR: snapshot file was not written" >&2
    return 1
  }
  grep -Fxq "GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS" "${snapshot_file}" || {
    echo "ERROR: snapshot status marker missing" >&2
    return 1
  }
  grep -Fq "GO2_XT16_STATUS_REPORT_OVERALL=" "${snapshot_file}" || {
    echo "ERROR: status-report output missing from snapshot" >&2
    return 1
  }
  grep -Fq "GO2_XT16_GATE_GAPS_OVERALL=" "${snapshot_file}" || {
    echo "ERROR: gate-gap output missing from snapshot" >&2
    return 1
  }
  grep -Fq "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE=" "${snapshot_file}" || {
    echo "ERROR: completion requirements audit output missing from snapshot" >&2
    return 1
  }
  grep -Fq "GO2_XT16_REPORT_COMPLETION_STATUS=" "${snapshot_file}" || {
    echo "ERROR: report-completion output missing from snapshot" >&2
    return 1
  }
  grep -Fq "GO2_XT16_RUNTIME_CLEAN_STATUS=" "${snapshot_file}" || {
    echo "ERROR: runtime-clean output missing from snapshot" >&2
    return 1
  }
  echo "GO2_XT16_STATUS_SNAPSHOT_SELFTEST_WRITE=PASS"
  echo "GO2_XT16_STATUS_SNAPSHOT_SELFTEST_CONTENT=PASS"
  echo "GO2_XT16_STATUS_SNAPSHOT_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  [[ -z "${output_file}" ]] || die "--self-test cannot be combined with --output"
  run_self_test
  exit 0
fi

if [[ -n "${output_file}" ]]; then
  write_snapshot "${output_file}"
else
  emit_snapshot
fi
