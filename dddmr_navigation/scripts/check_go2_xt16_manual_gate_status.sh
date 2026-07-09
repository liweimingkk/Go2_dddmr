#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/check_go2_xt16_manual_gate_status.sh

Static no-motion audit for the Go2 XT16 manual/supervised validation gates.

Checks only local files:
  - key shell wrappers parse with bash -n
  - key Python probes parse with ast.parse
  - navigation launch defaults keep Sport output disabled
  - p2p /cmd_vel is remapped to the dry-run topic
  - live wrappers require explicit supervision confirmations
  - supervised live navigation wrapper defaults to GO2_SPORT_MAX_X=0.30
  - supervised live Sport probe remains hard-capped at x<=0.10
  - planner config still carries the 0.30 m/s local-planner value
  - acceptance evidence verifier, current evidence file, and template are present
  - gate evidence gap summary is runnable and reports the current remaining gates
  - completion requirements audit is runnable and refuses incomplete completion
  - status snapshot collector can write a no-motion artifact
  - manual gate handoff builder can write the operator handoff artifact
  - static artifact refresh runner can refresh temp artifacts in self-test
  - report top status sync helper can update the authority block in self-test

This script does not source ROS, inspect live robot topics, start Docker,
publish /cmd_vel, publish /dddmr_go2/dry_run_cmd_vel, publish
/api/sport/request, or publish /lowcmd.
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

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_file() {
  local path="$1"
  [[ -f "${WS_ROOT}/${path}" ]] || die "missing file: ${path}"
}

require_text() {
  local path="$1"
  local text="$2"
  local label="$3"

  require_file "${path}"
  if ! grep -Fq -- "${text}" "${WS_ROOT}/${path}"; then
    die "missing ${label} in ${path}: ${text}"
  fi
  echo "OK ${label}"
}

require_evidence_key() {
  local path="$1"
  local key="$2"

  require_file "${path}"
  if ! awk -v key="${key}" '
    BEGIN { FS = "=" }
    /^[[:space:]]*#/ { next }
    /^[[:space:]]*$/ { next }
    $1 == key { found = 1; exit }
    END { exit found ? 0 : 1 }
  ' "${WS_ROOT}/${path}"; then
    die "missing evidence key ${key} in ${path}"
  fi
  echo "OK ${path} has ${key}"
}

echo "=== static file presence"
shell_files=(
  scripts/dddmr_docker_go2_xt16.sh
  scripts/run_go2_xt16_navigation_supervised_live.sh
  scripts/check_go2_xt16_sport_live_readiness.sh
  scripts/run_go2_sport_adapter_supervised_probe.sh
  scripts/record_go2_xt16_nav_debug_bag.sh
  scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh
  scripts/build_go2_xt16_axis_tilt_gate_update.sh
  scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh
  scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh
  scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh
  scripts/build_go2_xt16_axis_tilt_manual_handoff.sh
  scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh
  scripts/sync_go2_xt16_axis_tilt_report_status.sh
  scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh
  scripts/check_go2_xt16_acceptance_pipeline_selftest.sh
  scripts/check_go2_xt16_report_completion.sh
  scripts/check_go2_xt16_no_motion_runtime_clean.sh
)
python_files=(
  scripts/check_go2_odom_axis_consistency.py
  scripts/check_go2_xt16_tf_health.py
  scripts/probe_go2_xt16_dry_run_goal.py
  scripts/probe_go2_xt16_plan_candidates.py
  scripts/summarize_go2_xt16_base_cloud.py
)
for path in "${shell_files[@]}" "${python_files[@]}"; do
  require_file "${path}"
done
require_file docs/go2_xt16_axis_tilt_acceptance_checklist.md
require_file docs/go2_xt16_axis_tilt_acceptance_evidence.env
require_file docs/go2_xt16_axis_tilt_acceptance_evidence.env.example
echo "OK all checked files exist"

echo "=== evidence schema"
required_evidence_keys=(
  REPORT_PATH
  RVIZ_BASE_LINK_FRONT_OBJECT_STATUS
  RVIZ_FIXED_FRAME
  RVIZ_FRONT_OBJECT_DIRECTION
  SCREENSHOT
  TF_HEALTH_MAP_ODOM_STATUS
  TF_HEALTH_MAP_BASE_STATUS
  TF_HEALTH_STATUS
  TF_HEALTH_LOG
  RAW_ODOM_AXIS_STATUS
  STANDARD_ODOM_AXIS_STATUS
  RAW_STANDARD_ODOM_MATERIAL_MATCH
  ODOM_AXIS_ROTATION_CHECK
  RAW_ODOM_AXIS_LOG
  STANDARD_ODOM_AXIS_LOG
  GO2_XT16_SPORT_LIVE_READINESS
  SPORT_LIVE_READINESS_LOG
  SPORT_PROBE_RESULT
  SPORT_PROBE_SUMMARY
  SPORT_PROBE_HAS_MOVE_1008
  SPORT_PROBE_HAS_STOPMOVE_1003
  LIVE_NAV_SHORT_GOAL_STATUS
  LIVE_NAV_SUMMARY
  LIVE_NAV_LOG
  LIVE_NAV_MAX_X_MPS
  NO_RESIDUAL_RUNTIME_STATUS
  NO_RESIDUAL_RUNTIME_LOG
)
for key in "${required_evidence_keys[@]}"; do
  require_evidence_key docs/go2_xt16_axis_tilt_acceptance_evidence.env "${key}"
  require_evidence_key docs/go2_xt16_axis_tilt_acceptance_evidence.env.example "${key}"
done
echo "GO2_XT16_EVIDENCE_SCHEMA_STATUS=PASS"

echo "=== shell syntax"
for path in "${shell_files[@]}"; do
  bash -n "${WS_ROOT}/${path}"
  echo "OK bash -n ${path}"
done
echo "GO2_XT16_STATIC_SHELL_STATUS=PASS"

echo "=== python syntax"
python3 - "${python_files[@]/#/${WS_ROOT}/}" <<'PY'
import ast
import pathlib
import sys

for raw_path in sys.argv[1:]:
    path = pathlib.Path(raw_path)
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print(f"OK ast.parse {path.relative_to(path.parents[1])}")
PY
echo "GO2_XT16_STATIC_PYTHON_STATUS=PASS"

echo "=== no-motion launch defaults"
require_text \
  src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch \
  '<arg name="start_go2_sport_adapter" default="false"/>' \
  "launch disables embedded Sport adapter by default"
require_text \
  src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch \
  '<arg name="go2_sport_enable_output" default="false"/>' \
  "launch disables Sport output by default"
require_text \
  src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch \
  '<arg name="go2_sport_allow_real_request_topic" default="false"/>' \
  "launch blocks real request topic by default"
require_text \
  src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch \
  '<remap from="/cmd_vel" to="/dddmr_go2/dry_run_cmd_vel"/>' \
  "p2p cmd_vel remaps to dry-run topic"
require_text \
  scripts/dddmr_docker_go2_xt16.sh \
  'start_sport_dry_run_adapter:=false' \
  "Docker live-source path disables dry-run adapter"
require_text \
  scripts/dddmr_docker_go2_xt16.sh \
  'start_go2_sport_adapter:=false' \
  "Docker live-source path disables embedded Sport adapter"
echo "GO2_XT16_STATIC_NO_MOTION_DEFAULTS=PASS"

echo "=== supervised live gates"
require_text \
  scripts/run_go2_sport_adapter_supervised_probe.sh \
  'live mode requires GO2_SPORT_LIVE_CONFIRM=${CONFIRM_PHRASE}' \
  "sport probe requires live confirmation"
require_text \
  scripts/run_go2_xt16_navigation_supervised_live.sh \
  'live navigation requires GO2_NAV_LIVE_CONFIRM=${CONFIRM_PHRASE}' \
  "navigation wrapper requires live confirmation"
require_text \
  scripts/run_go2_xt16_navigation_supervised_live.sh \
  'live navigation requires GO2_SPORT_PROBE_SUMMARY from the live adapter probe' \
  "navigation wrapper requires prior live probe summary"
require_text \
  scripts/run_go2_xt16_navigation_supervised_live.sh \
  'sport_max_x="${GO2_SPORT_MAX_X:-0.30}"' \
  "navigation wrapper default max x is 0.30"
require_text \
  scripts/run_go2_sport_adapter_supervised_probe.sh \
  '"x": (x, 0.10),' \
  "sport live probe hard-caps x at 0.10"
require_text \
  docs/go2_xt16_sport_live_runbook.md \
  'abs(yaw) <= 0.35' \
  "sport live runbook documents yaw hard cap"
require_text \
  docs/go2_xt16_axis_tilt_acceptance_checklist.md \
  'GO2_XT16_ACCEPTANCE_STATUS=PASS' \
  "acceptance checklist requires final evidence verifier PASS"
echo "GO2_XT16_STATIC_LIVE_GATES=PASS"

echo "=== acceptance evidence verifier"
"${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh" --self-test
echo "GO2_XT16_ACCEPTANCE_VERIFIER_SELFTEST_STATUS=PASS"
"${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh" \
  --evidence "${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env" \
  --allow-incomplete
echo "GO2_XT16_ACCEPTANCE_VERIFIER_STATUS=PASS"
"${WS_ROOT}/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh" \
  --evidence "${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env" \
  --status-report \
  --allow-incomplete
echo "GO2_XT16_STATUS_REPORT_STATUS=PASS"
"${WS_ROOT}/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh" --self-test
echo "GO2_XT16_GATE_GAPS_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh" \
  --evidence "${WS_ROOT}/docs/go2_xt16_axis_tilt_acceptance_evidence.env"
echo "GO2_XT16_GATE_GAPS_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh" --self-test
echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh"
echo "GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh" --self-test
echo "GO2_XT16_STATUS_SNAPSHOT_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/build_go2_xt16_axis_tilt_manual_handoff.sh" --self-test
echo "GO2_XT16_MANUAL_HANDOFF_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh" --self-test
echo "GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/sync_go2_xt16_axis_tilt_report_status.sh" --self-test
echo "GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh" --self-test
echo "GO2_XT16_EVIDENCE_UPDATER_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/build_go2_xt16_axis_tilt_gate_update.sh" --self-test
echo "GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/check_go2_xt16_acceptance_pipeline_selftest.sh"
echo "GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/check_go2_xt16_report_completion.sh" --self-test
echo "GO2_XT16_REPORT_COMPLETION_SELFTEST_CHECK_STATUS=PASS"
"${WS_ROOT}/scripts/check_go2_xt16_report_completion.sh" --allow-incomplete
echo "GO2_XT16_REPORT_COMPLETION_CHECK_STATUS=PASS"

echo "=== residual runtime"
"${WS_ROOT}/scripts/check_go2_xt16_no_motion_runtime_clean.sh"
echo "GO2_XT16_RUNTIME_CLEAN_CHECK_STATUS=PASS"

echo "=== velocity values"
planner_max_x="$(
  awk '/max_vel_x:/ {print $2; exit}' \
    "${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"
)"
planner_min_x="$(
  awk '/min_vel_x:/ {print $2; exit}' \
    "${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"
)"
[[ "${planner_max_x}" == "0.30" ]] || die "planner max_vel_x is ${planner_max_x}, expected 0.30"
[[ "${planner_min_x}" == "0.30" ]] || die "planner min_vel_x is ${planner_min_x}, expected 0.30"
echo "GO2_XT16_PLANNER_CONFIG_MAX_VEL_X=${planner_max_x}"
echo "GO2_XT16_PLANNER_CONFIG_MIN_VEL_X=${planner_min_x}"
echo "GO2_XT16_LIVE_WRAPPER_DEFAULT_MAX_X=0.30"
echo "GO2_XT16_LIVE_PROBE_HARD_CAP_X=0.10"

echo "GO2_XT16_MANUAL_GATE_STATUS=PASS"
echo "GO2_XT16_MANUAL_GATE_NOTE=static_audit_only_no_robot_or_motion_commands"
