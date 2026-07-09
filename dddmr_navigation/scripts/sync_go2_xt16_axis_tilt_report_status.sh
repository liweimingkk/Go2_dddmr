#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/sync_go2_xt16_axis_tilt_report_status.sh [--dry-run]
  scripts/sync_go2_xt16_axis_tilt_report_status.sh \
    [--report FILE] [--evidence FILE] [--dry-run]
  scripts/sync_go2_xt16_axis_tilt_report_status.sh --self-test

Synchronize the top "current authority" status block in
/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md from the current
acceptance evidence verifier output.

Only the top four status lines are managed:
  report shape
  runtime clean
  acceptance
  completion

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

dry_run=false
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
    --dry-run)
      dry_run=true
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

compute_acceptance_status() {
  local status_out
  local overall

  [[ -x "${VERIFIER}" ]] || die "missing executable verifier: ${VERIFIER}"
  status_out="$(
    GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
      "${VERIFIER}" --evidence "${EVIDENCE_FILE}" --status-report --allow-incomplete
  )"
  overall="$(
    printf '%s\n' "${status_out}" | awk -F= '$1 == "GO2_XT16_STATUS_REPORT_OVERALL" {print $2; exit}'
  )"
  [[ -n "${overall}" ]] || die "verifier did not print GO2_XT16_STATUS_REPORT_OVERALL"
  printf '%s' "${overall}"
}

compute_runtime_status() {
  local runtime_status
  runtime_status="$(get_evidence_value NO_RESIDUAL_RUNTIME_STATUS)"
  if [[ "${runtime_status}" == "PASS" ]]; then
    printf 'PASS'
  else
    printf 'NOT_READY'
  fi
}

rewrite_report_file() {
  local source_file="$1"
  local output_file="$2"
  local report_shape="$3"
  local runtime_clean="$4"
  local acceptance="$5"
  local completion="$6"

  awk \
    -v report_shape="${report_shape}" \
    -v runtime_clean="${runtime_clean}" \
    -v acceptance="${acceptance}" \
    -v completion="${completion}" '
      $0 == "当前权威状态（2026-07-04）：" {
        in_authority = 1
        print
        next
      }
      in_authority && $0 == "```text" {
        in_block = 1
        print
        next
      }
      in_authority && in_block && $0 == "```" {
        in_authority = 0
        in_block = 0
        print
        next
      }
      in_authority && in_block && $0 ~ /^report shape:/ {
        print "report shape: " report_shape
        seen_report_shape = 1
        next
      }
      in_authority && in_block && $0 ~ /^runtime clean:/ {
        print "runtime clean: " runtime_clean
        seen_runtime_clean = 1
        next
      }
      in_authority && in_block && $0 ~ /^acceptance:/ {
        print "acceptance: " acceptance
        seen_acceptance = 1
        next
      }
      in_authority && in_block && $0 ~ /^completion:/ {
        print "completion: " completion
        seen_completion = 1
        next
      }
      { print }
      END {
        if (!seen_report_shape || !seen_runtime_clean || !seen_acceptance || !seen_completion) {
          exit 3
        }
      }
    ' "${source_file}" >"${output_file}"
}

sync_report_status() {
  local acceptance
  local completion
  local runtime_clean
  local tmp_file
  local backup_file

  [[ -f "${REPORT_PATH}" ]] || die "missing report: ${REPORT_PATH}"
  [[ -f "${EVIDENCE_FILE}" ]] || die "missing evidence file: ${EVIDENCE_FILE}"

  acceptance="$(compute_acceptance_status)"
  runtime_clean="$(compute_runtime_status)"
  completion="NOT_READY"
  if [[ "${acceptance}" == "PASS" ]]; then
    completion="PASS"
  fi

  tmp_file="$(mktemp)"
  cleanup() {
    rm -f "${tmp_file:-}"
  }
  trap cleanup RETURN

  rewrite_report_file "${REPORT_PATH}" "${tmp_file}" PASS "${runtime_clean}" "${acceptance}" "${completion}" || {
    rm -f "${tmp_file}"
    die "failed to rewrite top authority block in ${REPORT_PATH}"
  }

  echo "GO2_XT16_REPORT_STATUS_SYNC_REPORT=${REPORT_PATH}"
  echo "GO2_XT16_REPORT_STATUS_SYNC_EVIDENCE=${EVIDENCE_FILE}"
  echo "GO2_XT16_REPORT_STATUS_SYNC_RUNTIME_CLEAN=${runtime_clean}"
  echo "GO2_XT16_REPORT_STATUS_SYNC_ACCEPTANCE=${acceptance}"
  echo "GO2_XT16_REPORT_STATUS_SYNC_COMPLETION=${completion}"

  if cmp -s "${REPORT_PATH}" "${tmp_file}"; then
    echo "GO2_XT16_REPORT_STATUS_SYNC_CHANGED=false"
    echo "GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS"
    return 0
  fi

  if [[ "${dry_run}" == "true" ]]; then
    echo "GO2_XT16_REPORT_STATUS_SYNC_CHANGED=true"
    echo "GO2_XT16_REPORT_STATUS_SYNC_DRY_RUN=PASS"
    echo "GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS"
    return 0
  fi

  backup_file="${REPORT_PATH}.bak.$(date +%Y%m%d_%H%M%S)"
  cp "${REPORT_PATH}" "${backup_file}"
  mv "${tmp_file}" "${REPORT_PATH}"
  tmp_file=""
  echo "GO2_XT16_REPORT_STATUS_SYNC_CHANGED=true"
  echo "GO2_XT16_REPORT_STATUS_SYNC_BACKUP=${backup_file}"
  echo "GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS"
}

run_self_test() {
  local tmp_dir
  local report
  local out
  local rewritten
  local status

  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  report="${tmp_dir}/report.md"
  rewritten="${tmp_dir}/rewritten.md"

  cat >"${report}" <<'EOF'
# Synthetic report

当前权威状态（2026-07-04）：

```text
report shape: OLD
runtime clean: OLD
acceptance: OLD
completion: OLD
```

body
EOF

  rewrite_report_file "${report}" "${rewritten}" PASS PASS PASS PASS
  grep -Fxq "report shape: PASS" "${rewritten}" || return 1
  grep -Fxq "runtime clean: PASS" "${rewritten}" || return 1
  grep -Fxq "acceptance: PASS" "${rewritten}" || return 1
  grep -Fxq "completion: PASS" "${rewritten}" || return 1
  echo "GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_PASS_UPDATE=PASS"

  rewrite_report_file "${rewritten}" "${report}" PASS PASS NOT_READY NOT_READY
  grep -Fxq "acceptance: NOT_READY" "${report}" || return 1
  grep -Fxq "completion: NOT_READY" "${report}" || return 1
  echo "GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_NOT_READY_UPDATE=PASS"

  set +e
  out="$(rewrite_report_file /dev/null "${rewritten}" PASS PASS PASS PASS 2>&1)"
  status=$?
  set -e
  [[ "${status}" -ne 0 ]] || {
    echo "${out}" >&2
    echo "ERROR: malformed report unexpectedly passed" >&2
    return 1
  }
  echo "GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_MALFORMED_REJECT=PASS"
  echo "GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  [[ "${dry_run}" == "false" ]] || die "--self-test cannot be combined with --dry-run"
  run_self_test
  exit 0
fi

sync_report_status
