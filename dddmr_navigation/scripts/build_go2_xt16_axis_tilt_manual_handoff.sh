#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/build_go2_xt16_axis_tilt_manual_handoff.sh [--output ABSOLUTE_FILE]
  scripts/build_go2_xt16_axis_tilt_manual_handoff.sh --self-test

Build a no-motion Markdown handoff for the remaining Go2 XT16 axis/tilt manual
and supervised gates. The handoff lists the current verifier/gap status, the
artifact paths an operator must capture, and the local builder/updater commands
to apply those artifacts after each gate.

This script only reads local files and calls local no-motion status helpers. It
does not source ROS, inspect live robot topics, start Docker, publish /cmd_vel,
publish /dddmr_go2/dry_run_cmd_vel, publish /api/sport/request, or publish
/lowcmd.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${GO2_XT16_AXIS_TILT_REPORT:-/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md}"
EVIDENCE_FILE="${GO2_XT16_AXIS_TILT_EVIDENCE:-${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env}"
CHECKLIST="${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_checklist.md"
SNAPSHOT_FILE="${WS_ROOT}/run_logs/go2_xt16_axis_tilt/status_snapshot_20260704_current.txt"
VERIFIER="${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh"
GAP_SUMMARY="${WS_ROOT}/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh"
BUILDER="${WS_ROOT}/scripts/build_go2_xt16_axis_tilt_gate_update.sh"
UPDATER="${WS_ROOT}/scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh"

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

require_readable() {
  local path="$1"
  [[ -r "${path}" ]] || die "missing readable file: ${path}"
}

require_executable() {
  local path="$1"
  [[ -x "${path}" ]] || die "missing executable: ${path}"
}

indent_block() {
  sed 's/^/    /'
}

emit_handoff() {
  local status_out
  local gap_out

  require_readable "${REPORT_PATH}"
  require_readable "${EVIDENCE_FILE}"
  require_readable "${CHECKLIST}"
  require_executable "${VERIFIER}"
  require_executable "${GAP_SUMMARY}"
  require_executable "${BUILDER}"
  require_executable "${UPDATER}"

  status_out="$(
    GO2_XT16_AXIS_TILT_REPORT="${REPORT_PATH}" \
      "${VERIFIER}" --evidence "${EVIDENCE_FILE}" --status-report --allow-incomplete
  )"
  gap_out="$("${GAP_SUMMARY}" --evidence "${EVIDENCE_FILE}")"

  cat <<EOF
# Go2 XT16 Axis/Tilt Manual Gate Handoff

Generated: $(date -Is)

## Scope

This is a local no-motion handoff for finishing:

\`\`\`text
${REPORT_PATH}
\`\`\`

Current evidence file:

\`\`\`text
${EVIDENCE_FILE}
\`\`\`

Current status snapshot:

\`\`\`text
${SNAPSHOT_FILE}
\`\`\`

Checklist:

\`\`\`text
${CHECKLIST}
\`\`\`

## Safety Boundary

This handoff does not authorize motion. Do not publish or enable:

\`\`\`text
/cmd_vel
/dddmr_go2/dry_run_cmd_vel
/api/sport/request
/lowcmd
\`\`\`

Gate 4 requires explicit onsite approval and supervision before any live robot
movement. Use this file only to organize captured artifacts and update evidence
after the required manual/supervised checks are complete.

## Current Status

\`\`\`text
${status_out}
\`\`\`

## Current Gate Gaps

\`\`\`text
${gap_out}
\`\`\`

## Gate 1: RViz Base_Link Front Object

Required captured artifact:

\`\`\`text
/absolute/path/to/gate1_base_link_front_object.png
\`\`\`

The screenshot must be PNG/JPEG and must support:

\`\`\`text
RVIZ_FIXED_FRAME=base_link
RVIZ_FRONT_OBJECT_DIRECTION=+X
\`\`\`

After the screenshot is captured:

\`\`\`bash
${BUILDER} \\
  --gate gate1 \\
  --screenshot /absolute/path/to/gate1_base_link_front_object.png \\
  >/tmp/go2_xt16_gate1_update.env

${UPDATER} \\
  --evidence ${EVIDENCE_FILE} \\
  --update /tmp/go2_xt16_gate1_update.env \\
  --dry-run
\`\`\`

If the dry-run passes, rerun the updater without \`--dry-run\`.

## Gate 2: Initial Pose Then TF Tilt Recheck

Required captured artifact:

\`\`\`text
/absolute/path/to/gate2_tf_health.txt
\`\`\`

The log must contain:

\`\`\`text
TF_HEALTH_MAP_ODOM_STATUS=PASS
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=PASS
\`\`\`

After the log is captured:

\`\`\`bash
${BUILDER} \\
  --gate gate2 \\
  --tf-health-log /absolute/path/to/gate2_tf_health.txt \\
  >/tmp/go2_xt16_gate2_update.env

${UPDATER} \\
  --evidence ${EVIDENCE_FILE} \\
  --update /tmp/go2_xt16_gate2_update.env \\
  --dry-run
\`\`\`

If the dry-run passes, rerun the updater without \`--dry-run\`.

## Gate 3: Raw And Standard Odom Axis

Required captured artifacts:

\`\`\`text
/absolute/path/to/gate3_raw_odom_axis.txt
/absolute/path/to/gate3_standard_odom_axis.txt
\`\`\`

The raw log must contain:

\`\`\`text
ODOM_AXIS_TOPIC=/utlidar/robot_odom
ODOM_AXIS_STATUS=PASS
\`\`\`

The standard log must contain:

\`\`\`text
ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard
ODOM_AXIS_STATUS=PASS
\`\`\`

After both logs are captured:

\`\`\`bash
${BUILDER} \\
  --gate gate3 \\
  --raw-odom-axis-log /absolute/path/to/gate3_raw_odom_axis.txt \\
  --standard-odom-axis-log /absolute/path/to/gate3_standard_odom_axis.txt \\
  >/tmp/go2_xt16_gate3_update.env

${UPDATER} \\
  --evidence ${EVIDENCE_FILE} \\
  --update /tmp/go2_xt16_gate3_update.env \\
  --dry-run
\`\`\`

If the dry-run passes, rerun the updater without \`--dry-run\`.

## Gate 4: Supervised Live Short Goal

Required captured artifacts:

\`\`\`text
/absolute/path/to/gate4_sport_readiness.txt
/absolute/path/to/gate4_sport_probe_summary.env
/absolute/path/to/gate4_live_nav_summary.env
/absolute/path/to/gate4_live_nav.log
/absolute/path/to/no_motion_runtime_clean.txt
\`\`\`

The sport probe summary must reference an absolute \`REQUEST_ECHO_LOG\` that
contains both:

\`\`\`text
api_id: 1008
api_id: 1003
\`\`\`

The live nav summary must record:

\`\`\`text
MODE=live
RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED
REQUEST_TOPIC=/api/sport/request
SPORT_MAX_X<=0.30
\`\`\`

After all Gate 4 artifacts are captured:

\`\`\`bash
${BUILDER} \\
  --gate gate4 \\
  --sport-readiness-log /absolute/path/to/gate4_sport_readiness.txt \\
  --sport-probe-summary /absolute/path/to/gate4_sport_probe_summary.env \\
  --live-nav-summary /absolute/path/to/gate4_live_nav_summary.env \\
  --live-nav-log /absolute/path/to/gate4_live_nav.log \\
  --runtime-clean-log /absolute/path/to/no_motion_runtime_clean.txt \\
  >/tmp/go2_xt16_gate4_update.env

${UPDATER} \\
  --evidence ${EVIDENCE_FILE} \\
  --update /tmp/go2_xt16_gate4_update.env \\
  --dry-run
\`\`\`

If the dry-run passes, rerun the updater without \`--dry-run\`.

## Final Verification

After all four gate updates are applied, the required final commands are:

\`\`\`bash
${VERIFIER} --evidence ${EVIDENCE_FILE}
${WS_ROOT}/scripts/check_go2_xt16_report_completion.sh
${WS_ROOT}/scripts/check_go2_xt16_manual_gate_status.sh
\`\`\`

Completion requires:

\`\`\`text
GO2_XT16_ACCEPTANCE_STATUS=PASS
GO2_XT16_REPORT_COMPLETION_STATUS=PASS
\`\`\`
EOF
}

write_handoff() {
  local path="$1"
  local dir
  local tmp_file

  [[ "${path}" == /* ]] || die "--output must be an absolute path: ${path}"
  dir="$(dirname "${path}")"
  mkdir -p "${dir}"
  tmp_file="$(mktemp "${dir}/.manual_handoff_tmp.XXXXXX")"
  emit_handoff >"${tmp_file}"
  mv "${tmp_file}" "${path}"
  echo "GO2_XT16_MANUAL_HANDOFF_OUTPUT=${path}"
  echo "GO2_XT16_MANUAL_HANDOFF_WRITE_STATUS=PASS"
}

run_self_test() {
  local self_path
  local tmp_dir
  local handoff_file
  local out

  self_path="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  handoff_file="${tmp_dir}/handoff.md"

  out="$("${self_path}" --output "${handoff_file}")"
  [[ "${out}" == *"GO2_XT16_MANUAL_HANDOFF_WRITE_STATUS=PASS"* ]] || {
    echo "${out}" >&2
    echo "ERROR: handoff write status missing" >&2
    return 1
  }
  [[ -s "${handoff_file}" ]] || {
    echo "ERROR: handoff file was not written" >&2
    return 1
  }
  grep -Fq "GO2_XT16_STATUS_REPORT_OVERALL=" "${handoff_file}" || {
    echo "ERROR: status-report output missing from handoff" >&2
    return 1
  }
  grep -Fq "GO2_XT16_GATE_GAPS_OVERALL=" "${handoff_file}" || {
    echo "ERROR: gate-gap output missing from handoff" >&2
    return 1
  }
  grep -Fq "GO2_XT16_ACCEPTANCE_STATUS=PASS" "${handoff_file}" || {
    echo "ERROR: final acceptance condition missing from handoff" >&2
    return 1
  }
  grep -Fq "/api/sport/request" "${handoff_file}" || {
    echo "ERROR: sport request safety marker missing from handoff" >&2
    return 1
  }
  echo "GO2_XT16_MANUAL_HANDOFF_SELFTEST_WRITE=PASS"
  echo "GO2_XT16_MANUAL_HANDOFF_SELFTEST_CONTENT=PASS"
  echo "GO2_XT16_MANUAL_HANDOFF_SELFTEST_STATUS=PASS"
}

if [[ "${self_test}" == "true" ]]; then
  [[ -z "${output_file}" ]] || die "--self-test cannot be combined with --output"
  run_self_test
  exit 0
fi

if [[ -n "${output_file}" ]]; then
  write_handoff "${output_file}"
else
  emit_handoff
fi
