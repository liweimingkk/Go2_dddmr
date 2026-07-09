#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/run_go2_yaw_arc_supervised_probe_pair.sh --live

Run the two required supervised Go2 yaw-arc probes:
  1. x=0.05,z=-0.15
  2. x=0.05,z=+0.15

This can move the Go2 through /api/sport/request. It requires:
  GO2_YAW_ARC_PAIR_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_PAIR
  GO2_YAW_ARC_NOMOTION_REPORT=<passed report from check_go2_yaw_arc_shim_nomotion.sh>

The child probe script also runs the offline analyzer automatically. If either
direction fails analysis, this script stops and does not proceed to any RViz
navigation step.

Environment:
  GO2_YAW_ARC_PAIR_X=0.05
  GO2_YAW_ARC_PAIR_Y=0.0
  GO2_YAW_ARC_PAIR_NEG_YAW=-0.15
  GO2_YAW_ARC_PAIR_POS_YAW=0.15
  GO2_YAW_ARC_PAIR_DURATION=0.6
  GO2_YAW_ARC_PAIR_LOG_DIR=/tmp
  GO2_YAW_ARC_PAIR_SKIP_TOPIC_CHECK=false
  GO2_YAW_ARC_NOMOTION_REPORT=<required>
  GO2_SETUP=/home/lin/go2_workspace/unitree_ros2/setup.sh
  RVIZ=false
  PUBLISH_STATIC_TF=true

Live-mode hard caps are enforced by run_go2_yaw_feedback_probe.sh:
  abs(x) <= 0.05 m/s
  abs(y) <= 0.05 m/s
  abs(yaw) <= 0.30 rad/s
  duration <= 0.8 seconds
EOF
}

mode=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --live)
      mode="live"
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

[[ "${mode}" == "live" ]] || {
  usage >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
READINESS="${WS_ROOT}/scripts/check_go2_yaw_arc_live_readiness.sh"
YAW_PROBE="${WS_ROOT}/scripts/run_go2_yaw_feedback_probe.sh"
CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_YAW_ARC_PAIR"

pair_x="${GO2_YAW_ARC_PAIR_X:-0.05}"
pair_y="${GO2_YAW_ARC_PAIR_Y:-0.0}"
neg_yaw="${GO2_YAW_ARC_PAIR_NEG_YAW:--0.15}"
pos_yaw="${GO2_YAW_ARC_PAIR_POS_YAW:-0.15}"
pair_duration="${GO2_YAW_ARC_PAIR_DURATION:-0.6}"
log_dir="${GO2_YAW_ARC_PAIR_LOG_DIR:-/tmp}"
skip_topic_check="${GO2_YAW_ARC_PAIR_SKIP_TOPIC_CHECK:-false}"
nomotion_report="${GO2_YAW_ARC_NOMOTION_REPORT:-}"
stamp="$(date +%Y%m%d_%H%M%S)"
pair_report="${log_dir}/go2_yaw_arc_pair_${stamp}.env"
readiness_log="${log_dir}/go2_yaw_arc_pair_${stamp}_readiness.log"
neg_output_log="${log_dir}/go2_yaw_arc_pair_${stamp}_negative_probe.log"
pos_output_log="${log_dir}/go2_yaw_arc_pair_${stamp}_positive_probe.log"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

validate_settings() {
  [[ "${GO2_YAW_ARC_PAIR_CONFIRM:-}" == "${CONFIRM_PHRASE}" ]] || \
    die "live yaw-arc pair requires GO2_YAW_ARC_PAIR_CONFIRM=${CONFIRM_PHRASE}"
  [[ -n "${nomotion_report}" ]] || die "GO2_YAW_ARC_NOMOTION_REPORT is required"
  [[ -f "${nomotion_report}" ]] || die "missing GO2_YAW_ARC_NOMOTION_REPORT: ${nomotion_report}"
  [[ -x "${READINESS}" ]] || die "missing executable: ${READINESS}"
  [[ -x "${YAW_PROBE}" ]] || die "missing executable: ${YAW_PROBE}"
  case "${skip_topic_check}" in
    true|false) ;;
    *) die "GO2_YAW_ARC_PAIR_SKIP_TOPIC_CHECK must be true or false" ;;
  esac
}

run_readiness() {
  echo "=== readiness gate"
  GO2_YAW_ARC_NOMOTION_REPORT="${nomotion_report}" \
  GO2_YAW_ARC_READINESS_SKIP_TOPIC_CHECK="${skip_topic_check}" \
    "${READINESS}" >"${readiness_log}" 2>&1
  cat "${readiness_log}"
}

extract_value() {
  local key="$1"
  local file="$2"
  awk -F= -v key="${key}" '$1 == key {value=$2} END {print value}' "${file}"
}

run_one_probe() {
  local label="$1"
  local yaw="$2"
  local output_log="$3"
  local status
  local summary_log=""
  local analysis_log=""
  local analysis_result=""

  echo "=== ${label} yaw-arc probe yaw=${yaw}"
  set +e
  GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE \
  GO2_YAW_PROBE_X="${pair_x}" \
  GO2_YAW_PROBE_Y="${pair_y}" \
  GO2_YAW_PROBE_YAW="${yaw}" \
  GO2_YAW_PROBE_DURATION="${pair_duration}" \
  GO2_YAW_PROBE_ANALYZE=true \
  GO2_YAW_PROBE_LOG_DIR="${log_dir}" \
  RVIZ="${RVIZ:-false}" \
  PUBLISH_STATIC_TF="${PUBLISH_STATIC_TF:-true}" \
    "${YAW_PROBE}" --live >"${output_log}" 2>&1
  status=$?
  set -e

  cat "${output_log}"
  summary_log="$(awk -F= '$1 == "SUMMARY_LOG" {value=$2} END {print value}' "${output_log}")"
  if [[ -n "${summary_log}" && -f "${summary_log}" ]]; then
    analysis_log="$(extract_value "ANALYSIS_LOG" "${summary_log}")"
    analysis_result="$(extract_value "ANALYSIS_RESULT" "${summary_log}")"
  fi

  {
    echo "${label^^}_STATUS=${status}"
    echo "${label^^}_YAW=${yaw}"
    echo "${label^^}_OUTPUT_LOG=${output_log}"
    echo "${label^^}_SUMMARY_LOG=${summary_log}"
    echo "${label^^}_ANALYSIS_LOG=${analysis_log}"
    echo "${label^^}_ANALYSIS_RESULT=${analysis_result}"
  } >>"${pair_report}"

  if [[ "${status}" -ne 0 ]]; then
    die "${label} yaw-arc probe failed; output=${output_log}"
  fi
  [[ "${analysis_result}" == "GO2_YAW_PROBE_ANALYSIS_PASS" ]] || \
    die "${label} yaw-arc probe did not pass analysis; summary=${summary_log}"
}

validate_settings

cat >"${pair_report}" <<EOF
RESULT=GO2_YAW_ARC_PAIR_STARTED
STAMP=${stamp}
X=${pair_x}
Y=${pair_y}
NEG_YAW=${neg_yaw}
POS_YAW=${pos_yaw}
DURATION=${pair_duration}
NOMOTION_REPORT=${nomotion_report}
READINESS_LOG=${readiness_log}
NEG_OUTPUT_LOG=${neg_output_log}
POS_OUTPUT_LOG=${pos_output_log}
EOF

echo "PAIR_REPORT=${pair_report}"
echo "This supervised pair can move the Go2. Confirmation accepted."
run_readiness
run_one_probe "negative" "${neg_yaw}" "${neg_output_log}"
run_one_probe "positive" "${pos_yaw}" "${pos_output_log}"

{
  echo "RESULT=GO2_YAW_ARC_PAIR_PASS"
} >>"${pair_report}"

echo "PAIR_REPORT=${pair_report}"
echo "RESULT: GO2_YAW_ARC_PAIR_PASS"
