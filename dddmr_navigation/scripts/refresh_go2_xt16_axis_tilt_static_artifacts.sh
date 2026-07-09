#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh
  scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh --self-test

Refresh the local no-motion artifacts for the Go2 XT16 axis/tilt report:
  - residual runtime clean log
  - status snapshot artifact
  - manual gate handoff artifact

Then run the local report completion audit and manual gate static audit. This
does not complete the report; it only keeps local static artifacts current.

This script does not source ROS, inspect live robot topics, start Docker,
publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel, publish /api/sport/request,
or publish /lowcmd.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUN_DIR="${WS_ROOT}/run_logs/go2_xt16_axis_tilt"
RUNTIME_LOG="${RUN_DIR}/no_motion_runtime_clean_20260704_current.txt"
SNAPSHOT_FILE="${RUN_DIR}/status_snapshot_20260704_current.txt"
HANDOFF_FILE="${RUN_DIR}/manual_gate_handoff_20260704_current.md"

RUNTIME_CLEAN="${WS_ROOT}/scripts/check_go2_xt16_no_motion_runtime_clean.sh"
SNAPSHOT_COLLECTOR="${WS_ROOT}/scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh"
HANDOFF_BUILDER="${WS_ROOT}/scripts/build_go2_xt16_axis_tilt_manual_handoff.sh"
REPORT_CHECKER="${WS_ROOT}/scripts/check_go2_xt16_report_completion.sh"
MANUAL_AUDIT="${WS_ROOT}/scripts/check_go2_xt16_manual_gate_status.sh"

self_test=false

while [[ $# -gt 0 ]]; do
  case "$1" in
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

atomic_capture() {
  local output="$1"
  shift
  local dir
  local tmp_file

  [[ "${output}" == /* ]] || die "output path must be absolute: ${output}"
  dir="$(dirname "${output}")"
  mkdir -p "${dir}"
  tmp_file="$(mktemp "${dir}/.refresh_tmp.XXXXXX")"
  if "$@" >"${tmp_file}"; then
    mv "${tmp_file}" "${output}"
  else
    local status=$?
    cat "${tmp_file}" >&2 || true
    rm -f "${tmp_file}"
    return "${status}"
  fi
}

refresh_artifacts() {
  require_executable "${RUNTIME_CLEAN}"
  require_executable "${SNAPSHOT_COLLECTOR}"
  require_executable "${HANDOFF_BUILDER}"
  require_executable "${REPORT_CHECKER}"
  require_executable "${MANUAL_AUDIT}"

  atomic_capture "${RUNTIME_LOG}" "${RUNTIME_CLEAN}"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_RUNTIME_LOG=${RUNTIME_LOG}"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_RUNTIME_STATUS=PASS"

  "${SNAPSHOT_COLLECTOR}" --output "${SNAPSHOT_FILE}"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_SNAPSHOT_STATUS=PASS"

  "${HANDOFF_BUILDER}" --output "${HANDOFF_FILE}"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_HANDOFF_STATUS=PASS"

  "${REPORT_CHECKER}" --allow-incomplete
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_REPORT_CHECK_STATUS=PASS"

  "${MANUAL_AUDIT}"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_MANUAL_AUDIT_STATUS=PASS"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_STATUS=PASS"
}

run_self_test() {
  local tmp_dir
  local runtime_out
  local snapshot_out
  local handoff_out

  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  runtime_out="${tmp_dir}/runtime_clean.txt"
  snapshot_out="${tmp_dir}/snapshot.txt"
  handoff_out="${tmp_dir}/handoff.md"

  require_executable "${RUNTIME_CLEAN}"
  require_executable "${SNAPSHOT_COLLECTOR}"
  require_executable "${HANDOFF_BUILDER}"

  atomic_capture "${runtime_out}" "${RUNTIME_CLEAN}"
  grep -Fxq "GO2_XT16_RUNTIME_CLEAN_STATUS=PASS" "${runtime_out}" || {
    echo "ERROR: runtime clean marker missing from self-test artifact" >&2
    return 1
  }
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_RUNTIME=PASS"

  "${SNAPSHOT_COLLECTOR}" --output "${snapshot_out}" >/dev/null
  grep -Fxq "GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS" "${snapshot_out}" || {
    echo "ERROR: snapshot marker missing from self-test artifact" >&2
    return 1
  }
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_SNAPSHOT=PASS"

  "${HANDOFF_BUILDER}" --output "${handoff_out}" >/dev/null
  grep -Fq "GO2_XT16_ACCEPTANCE_STATUS=PASS" "${handoff_out}" || {
    echo "ERROR: handoff final acceptance marker missing from self-test artifact" >&2
    return 1
  }
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_HANDOFF=PASS"
  echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  run_self_test
  exit 0
fi

refresh_artifacts
