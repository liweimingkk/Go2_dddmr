#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_go2_xt16_scan_navigation.sh --live
  ./scripts/run_go2_xt16_scan_navigation.sh --dry-run

One-command launcher for the Go2 XT16 SCAN-Planner integration.

  --live     Start the existing supervised live-navigation workflow after an
             interactive onsite-supervision confirmation.
  --dry-run  Start SCAN-Planner with real Unitree Sport output disabled.

Optional speed overrides:
  GO2_SPORT_MAX_X=0.40
  GO2_SPORT_MAX_Y=0.0
  GO2_SPORT_MAX_YAW=0.40
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

if (( $# != 1 )); then
  usage >&2
  exit 2
fi

mode="$1"
case "${mode}" in
  --live|--dry-run) ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAV_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SUPERVISOR="${SCRIPT_DIR}/run_go2_xt16_navigation_supervised_live.sh"
ADAPTER="${NAV_ROOT}/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py"
LOG_DIR="${GO2_NAV_LOG_DIR:-${NAV_ROOT}/run_logs}"
GO2_DDS_IP="${GO2_DDS_IP:-192.168.123.18}"

[[ -x "${SUPERVISOR}" ]] || die "missing supervisor: ${SUPERVISOR}"
[[ -f "${ADAPTER}" ]] || die "missing Sport adapter: ${ADAPTER}"
mkdir -p -- "${LOG_DIR}"

detect_net_iface() {
  ip route get "${GO2_DDS_IP}" 2>/dev/null | awk '
    {
      for (i = 1; i <= NF; i++) {
        if ($i == "dev" && (i + 1) <= NF) {
          print $(i + 1)
          exit
        }
      }
    }
  '
}

if [[ -z "${GO2_NET_IFACE:-}" ]]; then
  GO2_NET_IFACE="$(detect_net_iface)"
fi
[[ -n "${GO2_NET_IFACE:-}" ]] || \
  die "cannot resolve the Go2 network interface for ${GO2_DDS_IP}"

GO2_SETUP="${GO2_SETUP:-${NAV_ROOT}/.unitree_msg_ws/install/setup.bash}"
[[ -f "${GO2_SETUP}" ]] || die "missing Go2 ROS setup: ${GO2_SETUP}"

export GO2_DDS_IP
export GO2_NET_IFACE
export GO2_SETUP
export GO2_NAV_LOG_DIR="${LOG_DIR}"
export GO2_NAV_DRY_RUN_COMMAND="scan-navigation-dry-run"
export GO2_NAV_DOCKER_COMMAND="scan-navigation-live-source"
export GO2_SPORT_MAX_X="${GO2_SPORT_MAX_X:-0.40}"
export GO2_SPORT_MAX_Y="${GO2_SPORT_MAX_Y:-0.0}"
export GO2_SPORT_MAX_YAW="${GO2_SPORT_MAX_YAW:-0.40}"
export GO2_ENABLE_YAW_ARC_SHIM="${GO2_ENABLE_YAW_ARC_SHIM:-false}"
export RVIZ="${RVIZ:-true}"
export PUBLISH_STATIC_TF="${PUBLISH_STATIC_TF:-true}"

find_latest_probe_summary() {
  local adapter_sha summary summary_sha summary_mtime
  local latest=""
  local latest_mtime=0
  adapter_sha="$(sha256sum -- "${ADAPTER}" | awk '{print $1}')"

  for summary in "${LOG_DIR}"/go2_sport_adapter_live_*_summary.env; do
    [[ -f "${summary}" ]] || continue
    grep -Fxq 'MODE=live' "${summary}" || continue
    grep -Fxq 'RESULT=GO2_SPORT_ADAPTER_live_COMPLETE' "${summary}" || continue
    summary_sha="$(awk -F= '$1 == "ADAPTER_SHA256" { print $2; exit }' "${summary}")"
    [[ "${summary_sha}" == "${adapter_sha}" ]] || continue
    summary_mtime="$(stat -c '%Y' -- "${summary}")"
    if (( summary_mtime > latest_mtime )); then
      latest="${summary}"
      latest_mtime="${summary_mtime}"
    fi
  done

  printf '%s\n' "${latest}"
}

if [[ "${mode}" == "--live" ]]; then
  if [[ -z "${GO2_SPORT_PROBE_SUMMARY:-}" ]]; then
    GO2_SPORT_PROBE_SUMMARY="$(find_latest_probe_summary)"
  fi
  [[ -n "${GO2_SPORT_PROBE_SUMMARY:-}" ]] || die \
    "no current successful Sport probe summary found under ${LOG_DIR}"
  [[ -f "${GO2_SPORT_PROBE_SUMMARY}" ]] || \
    die "probe summary does not exist: ${GO2_SPORT_PROBE_SUMMARY}"

  [[ -t 0 ]] || die "live mode requires an interactive terminal"
  printf '\nSCAN live navigation can move the Go2.\n'
  printf 'Confirm clear space, physical stop access, and onsite supervision. [y/N] '
  read -r answer
  [[ "${answer}" == "y" || "${answer}" == "Y" ]] || die "live launch cancelled"

  export GO2_NAV_LIVE_CONFIRM="I_AM_SUPERVISING_GO2_NAV"
  export GO2_SPORT_PROBE_SUMMARY
fi

printf 'SCAN_MODE=%s\n' "${mode#--}"
printf 'GO2_NET_IFACE=%s\n' "${GO2_NET_IFACE}"
printf 'SCAN_SPEED_LIMITS=x:%s y:%s yaw:%s\n' \
  "${GO2_SPORT_MAX_X}" "${GO2_SPORT_MAX_Y}" "${GO2_SPORT_MAX_YAW}"
if [[ "${mode}" == "--live" ]]; then
  printf 'GO2_SPORT_PROBE_SUMMARY=%s\n' "${GO2_SPORT_PROBE_SUMMARY}"
fi

exec "${SUPERVISOR}" "${mode}"
