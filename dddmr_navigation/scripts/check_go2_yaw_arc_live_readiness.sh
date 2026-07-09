#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/check_go2_yaw_arc_live_readiness.sh

Read-only gate before supervised Go2 yaw-arc live probes.

Checks:
  - no stale Go2/DDDMR Docker container or local nav/adapter/RViz process
  - yaw-arc no-motion verifier report passed and its referenced logs exist
  - live yaw probe script rejects missing supervision confirmation
  - live navigation yaw-arc mode rejects missing no-motion report
  - optionally, live Go2 ROS topic types are visible

This script does not publish /cmd_vel, /dddmr_go2/dry_run_cmd_vel, or
/api/sport/request.

Environment:
  GO2_YAW_ARC_NOMOTION_REPORT=<required, from check_go2_yaw_arc_shim_nomotion.sh>
  GO2_YAW_ARC_READINESS_SKIP_TOPIC_CHECK=false
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh
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
GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
YAW_PROBE="${WS_ROOT}/scripts/run_go2_yaw_feedback_probe.sh"
NAV_LIVE="${WS_ROOT}/scripts/run_go2_xt16_navigation_supervised_live.sh"
ANALYZER="${WS_ROOT}/scripts/analyze_go2_yaw_probe_result.py"
PAIR_RUNNER="${WS_ROOT}/scripts/run_go2_yaw_arc_supervised_probe_pair.sh"
NOMOTION_REPORT="${GO2_YAW_ARC_NOMOTION_REPORT:-}"
SKIP_TOPIC_CHECK="${GO2_YAW_ARC_READINESS_SKIP_TOPIC_CHECK:-false}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

expect_command_failure() {
  local expected="$1"
  shift
  local out
  local status

  set +e
  out="$("$@" 2>&1)"
  status=$?
  set -e

  if [[ "${status}" -eq 0 ]]; then
    echo "${out}" >&2
    die "expected command to fail: $*"
  fi
  if [[ "${out}" != *"${expected}"* ]]; then
    echo "${out}" >&2
    die "expected failure text not found: ${expected}"
  fi
}

check_no_conflicts() {
  local docker_matches
  docker_matches="$(docker ps --format '{{.Names}} {{.Image}}' 2>/dev/null | \
    rg 'go2_xt16|go2_yaw_probe|dddmr_go2_xt16|dddmr_navigation' || true)"
  if [[ -n "${docker_matches}" ]]; then
    echo "${docker_matches}" >&2
    die "stale Go2/DDDMR Docker container is still running"
  fi

  local proc_matches
  proc_matches="$(ps -eo pid,args | \
    rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | \
    rg -v 'check_go2_yaw_arc_live_readiness|bash -lc|py_compile|sed -n|nl -ba|rg |ps -eo' || true)"
  if [[ -n "${proc_matches}" ]]; then
    echo "${proc_matches}" >&2
    die "stale nav/adapter/RViz/ros2 pub/echo process is still running"
  fi
}

source_go2_ros() {
  [[ -f "${GO2_SETUP}" ]] || die "missing Go2 ROS setup: ${GO2_SETUP}"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
  set +u
  # shellcheck disable=SC1090
  source "${GO2_SETUP}"
  # shellcheck disable=SC1090
  source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
  set -u
}

check_topic_type() {
  local topic="$1"
  local expected="$2"
  local info
  info="$(timeout 10 ros2 topic info "${topic}" 2>&1)" || {
    echo "${info}" >&2
    die "topic ${topic} is not visible"
  }
  echo "=== ${topic}"
  echo "${info}"
  [[ "${info}" == *"Type: ${expected}"* ]] || die "${topic} type is not ${expected}"
}

validate_nomotion_report() {
  [[ -n "${NOMOTION_REPORT}" ]] || \
    die "GO2_YAW_ARC_NOMOTION_REPORT is required"
  [[ -f "${NOMOTION_REPORT}" ]] || \
    die "missing GO2_YAW_ARC_NOMOTION_REPORT: ${NOMOTION_REPORT}"

  # shellcheck disable=SC1090
  source "${NOMOTION_REPORT}"
  [[ "${RESULT:-}" == "GO2_YAW_ARC_SHIM_NOMOTION_PASS" ]] || \
    die "no-motion report did not pass: ${NOMOTION_REPORT}"

  [[ -f "${ALLOWED_LOG:-}" ]] || die "missing ALLOWED_LOG from no-motion report"
  [[ -f "${RECOVERY_LOG:-}" ]] || die "missing RECOVERY_LOG from no-motion report"
  [[ -f "${STALE_LOG:-}" ]] || die "missing STALE_LOG from no-motion report"

  rg -q --fixed-strings 'transformed_sport={"x":0.05,"y":0.0,"z":-0.15}' "${ALLOWED_LOG}" || \
    die "allowed log does not prove x=0.05,z=-0.15 transform"
  rg -q --fixed-strings 'shim=blocked_blocked_state=d_recovery_waitdone' "${RECOVERY_LOG}" || \
    die "recovery log does not prove recovery state block"
  rg -q --fixed-strings 'shim=blocked_stale_decision=d_align_heading' "${STALE_LOG}" || \
    die "stale log does not prove stale decision block"
}

check_nav_live_nomotion_report_gate() {
  local tmpdir
  local summary
  local echo_log
  tmpdir="$(mktemp -d /tmp/go2_yaw_arc_live_readiness_XXXXXX)"
  summary="${tmpdir}/fake_live_probe_summary.env"
  echo_log="${tmpdir}/fake_request_echo.log"

  cat >"${echo_log}" <<'EOF'
header:
  identity:
    id: 101
    api_id: 1008
parameter: '{"x":0.05,"y":0.0,"z":0.0}'
---
header:
  identity:
    id: 102
    api_id: 1003
parameter: ''
EOF

  cat >"${summary}" <<EOF
MODE=live
RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
REQUEST_ID_BASE=100
REQUEST_ECHO_LOG=${echo_log}
EOF

  expect_command_failure \
    "live yaw-arc shim requires GO2_YAW_ARC_NOMOTION_REPORT" \
    env -u GO2_YAW_ARC_NOMOTION_REPORT \
      GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
      GO2_SPORT_PROBE_SUMMARY="${summary}" \
      GO2_ENABLE_YAW_ARC_SHIM=true \
      GO2_YAW_ARC_SHIM_MODE=live \
      GO2_YAW_ARC_SHIM_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_SHIM \
      "${NAV_LIVE}" --live
}

case "${SKIP_TOPIC_CHECK}" in
  true|false) ;;
  *) die "GO2_YAW_ARC_READINESS_SKIP_TOPIC_CHECK must be true or false" ;;
esac

echo "=== runtime cleanup check"
check_no_conflicts
echo "OK no stale Go2/DDDMR container or nav/adapter/RViz process"

echo "=== script presence"
[[ -x "${YAW_PROBE}" ]] || die "missing executable: ${YAW_PROBE}"
[[ -x "${NAV_LIVE}" ]] || die "missing executable: ${NAV_LIVE}"
[[ -x "${ANALYZER}" ]] || die "missing executable: ${ANALYZER}"
[[ -x "${PAIR_RUNNER}" ]] || die "missing executable: ${PAIR_RUNNER}"
echo "OK yaw probe, analyzer, pair runner, and live navigation scripts are executable"

echo "=== yaw-arc no-motion report"
validate_nomotion_report
echo "OK no-motion yaw-arc report passed: ${NOMOTION_REPORT}"

echo "=== confirmation gates"
expect_command_failure \
  "GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE" \
  "${YAW_PROBE}" --live
echo "OK yaw feedback probe rejects missing confirmation"

check_nav_live_nomotion_report_gate
echo "OK live yaw-arc navigation rejects missing no-motion report"

if [[ "${SKIP_TOPIC_CHECK}" == "true" ]]; then
  echo "Skipping live Go2 topic checks by GO2_YAW_ARC_READINESS_SKIP_TOPIC_CHECK=true"
else
  echo "=== Go2 ROS read-only topics"
  source_go2_ros
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
  check_topic_type "/api/sport/request" "unitree_api/msg/Request"
  check_topic_type "/sportmodestate" "unitree_go/msg/SportModeState"
  check_topic_type "/lowstate" "unitree_go/msg/LowState"
fi

cat <<EOF
=== next supervised probe commands
GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE \\
GO2_YAW_PROBE_X=0.05 \\
GO2_YAW_PROBE_Y=0.0 \\
GO2_YAW_PROBE_YAW=-0.15 \\
GO2_YAW_PROBE_DURATION=0.6 \\
RVIZ=false \\
PUBLISH_STATIC_TF=true \\
scripts/run_go2_yaw_feedback_probe.sh --live

# The probe now runs the analyzer automatically and prints ANALYSIS_LOG.
# To re-check manually:
scripts/analyze_go2_yaw_probe_result.py <SUMMARY_LOG>

GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE \\
GO2_YAW_PROBE_X=0.05 \\
GO2_YAW_PROBE_Y=0.0 \\
GO2_YAW_PROBE_YAW=0.15 \\
GO2_YAW_PROBE_DURATION=0.6 \\
RVIZ=false \\
PUBLISH_STATIC_TF=true \\
scripts/run_go2_yaw_feedback_probe.sh --live

# The probe now runs the analyzer automatically and prints ANALYSIS_LOG.
# To re-check manually:
scripts/analyze_go2_yaw_probe_result.py <SUMMARY_LOG>
EOF

cat <<EOF
=== optional paired supervised command
GO2_YAW_ARC_PAIR_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_PAIR \\
GO2_YAW_ARC_NOMOTION_REPORT=${NOMOTION_REPORT} \\
GO2_YAW_ARC_PAIR_X=0.05 \\
GO2_YAW_ARC_PAIR_Y=0.0 \\
GO2_YAW_ARC_PAIR_NEG_YAW=-0.15 \\
GO2_YAW_ARC_PAIR_POS_YAW=0.15 \\
GO2_YAW_ARC_PAIR_DURATION=0.6 \\
GO2_YAW_ARC_PAIR_SKIP_TOPIC_CHECK=false \\
RVIZ=false \\
PUBLISH_STATIC_TF=true \\
scripts/run_go2_yaw_arc_supervised_probe_pair.sh --live
EOF

echo "RESULT: GO2_YAW_ARC_LIVE_READINESS_PASS"
