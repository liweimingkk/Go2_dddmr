#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_go2_xt16_scan_navigation.sh --live
  ./scripts/run_go2_xt16_scan_navigation.sh --dry-run
  ./scripts/run_go2_xt16_scan_navigation.sh --record MISSION.json \
    --initial-pose INITIAL_POSE.json
  ./scripts/run_go2_xt16_scan_navigation.sh --multi-dry-run MISSION.json
  ./scripts/run_go2_xt16_scan_navigation.sh --multi-live MISSION.json

One-command launcher for the Go2 XT16 SCAN-Planner integration.

  --live     Start the existing supervised live-navigation workflow after an
             interactive onsite-supervision confirmation.
  --dry-run  Start SCAN-Planner with real Unitree Sport output disabled.
  --record   Start localization with real Sport output disabled. Save the
             fixed initial pose and current localized positions interactively.
  --multi-dry-run
             Load the fixed initial pose and execute one sequential mission
             through dry-run command topics.
  --multi-live
             Execute one sequential mission through the supervised Sport
             adapter. Requires both normal live evidence and a mission-specific
             `EXECUTE <mission_id>` confirmation after localization is ready.

Optional speed overrides:
  GO2_SPORT_MAX_X=0.40
  GO2_SPORT_MAX_Y=0.0
  GO2_SPORT_MAX_YAW=0.40

Live odom/XT16 preflight:
  At startup, select:
    1  Quick: two consistent 3-second windows, no retry windows.
    2  Full:  two consistent 8-second windows, up to four attempts.
  Set GO2_ODOM_PREFLIGHT_MODE=quick or full to preselect the mode.

Existing running Go2/DDDMR navigation containers are stopped and removed
before this launcher starts a new navigation runtime.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

if (( $# == 0 )); then
  usage >&2
  exit 2
fi

mode="$1"
mission_file=""
initial_pose_file=""
case "${mode}" in
  --live|--dry-run)
    (( $# == 1 )) || {
      usage >&2
      exit 2
    }
    ;;
  --multi-live|--multi-dry-run)
    (( $# == 2 )) || {
      usage >&2
      exit 2
    }
    mission_file="$2"
    ;;
  --record)
    (( $# == 4 )) && [[ "$3" == "--initial-pose" ]] || {
      usage >&2
      exit 2
    }
    mission_file="$2"
    initial_pose_file="$4"
    ;;
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
BAGS_DIR="${DDDMR_BAGS_DIR:-${NAV_ROOT}/../bags}"
MISSION_IO="${NAV_ROOT}/src/dddmr_scan_planner/scripts/scan_mission_io.py"
DOCKER_WRAPPER="${SCRIPT_DIR}/dddmr_docker_go2_xt16.sh"

[[ -x "${SUPERVISOR}" ]] || die "missing supervisor: ${SUPERVISOR}"
[[ -f "${ADAPTER}" ]] || die "missing Sport adapter: ${ADAPTER}"
[[ -x "${DOCKER_WRAPPER}" ]] || die "missing Docker wrapper: ${DOCKER_WRAPPER}"
[[ -f "${MISSION_IO}" ]] || die "missing mission validator: ${MISSION_IO}"
mkdir -p -- "${LOG_DIR}"

BAGS_DIR="$(realpath -m -- "${BAGS_DIR}")"

resolve_bags_path() {
  local requested="$1"
  local resolved
  resolved="$(realpath -m -- "${requested}")"
  if [[ "${resolved}" != "${BAGS_DIR}"/* && "${requested}" != /* ]]; then
    requested="${requested#./}"
    requested="${requested#bags/}"
    resolved="$(realpath -m -- "${BAGS_DIR}/${requested}")"
  fi
  [[ "${resolved}" == "${BAGS_DIR}"/* ]] || \
    die "mission data must stay under ${BAGS_DIR}: ${requested}"
  printf '%s\n' "${resolved}"
}

container_bags_path() {
  local host_path="$1"
  printf '/root/dddmr_bags/%s\n' "${host_path#"${BAGS_DIR}"/}"
}

remove_conflicting_navigation_containers() {
  local container_list id name image remove_error attempt
  local -a container_ids=()
  local -a container_descriptions=()

  if ! container_list="$(docker ps --format '{{.ID}}\t{{.Names}}\t{{.Image}}' 2>&1)"; then
    printf '%s\n' "${container_list}" >&2
    die "cannot inspect Docker containers"
  fi

  while IFS=$'\t' read -r id name image; do
    [[ -n "${id}" ]] || continue
    if [[ "${name} ${image}" =~ go2_xt16|dddmr_go2_xt16|dddmr_navigation ]]; then
      container_ids+=("${id}")
      container_descriptions+=("${name} (${image})")
    fi
  done <<<"${container_list}"

  if (( ${#container_ids[@]} == 0 )); then
    return
  fi

  printf 'Removing existing Go2/DDDMR navigation container(s):\n'
  printf '  %s\n' "${container_descriptions[@]}"
  docker stop --time 5 "${container_ids[@]}" >/dev/null || \
    die "failed to stop an existing Go2/DDDMR navigation container"

  for id in "${container_ids[@]}"; do
    if remove_error="$(docker rm "${id}" 2>&1)"; then
      continue
    fi

    # Containers launched with --rm can still be disappearing just after
    # docker stop returns. Treat that race as success once Docker confirms
    # that the container no longer exists.
    for (( attempt = 0; attempt < 50; attempt++ )); do
      if ! docker inspect "${id}" >/dev/null 2>&1; then
        remove_error=""
        break
      fi
      sleep 0.1
    done

    if [[ -n "${remove_error}" ]]; then
      printf '%s\n' "${remove_error}" >&2
      die "failed to remove existing Go2/DDDMR navigation container ${id}"
    fi
  done
}

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
unset GO2_SCAN_MISSION_FILE_CONTAINER GO2_SCAN_MISSION_ID

mission_id=""
mission_container=""
initial_pose_container=""
if [[ -n "${mission_file}" ]]; then
  mission_file="$(resolve_bags_path "${mission_file}")"
  mission_container="$(container_bags_path "${mission_file}")"
fi
if [[ -n "${initial_pose_file}" ]]; then
  initial_pose_file="$(resolve_bags_path "${initial_pose_file}")"
  initial_pose_container="$(container_bags_path "${initial_pose_file}")"
fi

if [[ "${mode}" == "--record" ]]; then
  [[ "${mission_file}" != "${initial_pose_file}" ]] || \
    die "mission and initial-pose files must be different"
  mkdir -p -- "$(dirname -- "${mission_file}")" "$(dirname -- "${initial_pose_file}")"
elif [[ "${mode}" == "--multi-live" || "${mode}" == "--multi-dry-run" ]]; then
  [[ -f "${mission_file}" ]] || die "mission file does not exist: ${mission_file}"
  /usr/bin/python3 "${MISSION_IO}" validate \
    --mission "${mission_file}" --root "${BAGS_DIR}"
  mission_id="$(
    /usr/bin/python3 "${MISSION_IO}" mission-id \
      --mission "${mission_file}" --root "${BAGS_DIR}"
  )"
  export GO2_SCAN_MISSION_FILE_CONTAINER="${mission_container}"
  export GO2_SCAN_MISSION_ID="${mission_id}"
fi

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

configure_live_odom_preflight() {
  local selection="${GO2_ODOM_PREFLIGHT_MODE:-}"

  if [[ -n "${ODOM_TIME_OFFSET_SEC:-}" ]]; then
    printf 'ODOM_PREFLIGHT_MODE=explicit-offset\n'
    printf 'Using caller-provided ODOM_TIME_OFFSET_SEC; interactive preflight selection skipped.\n'
    return
  fi

  if [[ -z "${selection}" ]]; then
    [[ -t 0 ]] || \
      die "live mode requires an interactive terminal or GO2_ODOM_PREFLIGHT_MODE=quick|full"
    printf '\nSelect the read-only odom/XT16 time-sync preflight:\n'
    printf '  1) Quick test: two consistent 3-second windows; fail closed on any problem.\n'
    printf '  2) Full test: two consistent 8-second windows; retry up to four times.\n'
    printf 'Enter 1 for the quick test; use 2 if the quick test reports a problem. [1/2] '
    read -r selection
  fi

  case "${selection}" in
    1|quick)
      export GO2_ODOM_PREFLIGHT_MODE="quick"
      export ODOM_OFFSET_MEASURE_SECONDS="3"
      export ODOM_OFFSET_MEASURE_ATTEMPTS="2"
      export ODOM_OFFSET_CONFIRMATIONS="2"
      ;;
    2|full)
      export GO2_ODOM_PREFLIGHT_MODE="full"
      export ODOM_OFFSET_MEASURE_SECONDS="8"
      export ODOM_OFFSET_MEASURE_ATTEMPTS="4"
      export ODOM_OFFSET_CONFIRMATIONS="2"
      ;;
    *)
      die "select odom/XT16 preflight 1 (quick) or 2 (full)"
      ;;
  esac

  printf 'ODOM_PREFLIGHT_MODE=%s\n' "${GO2_ODOM_PREFLIGHT_MODE}"
  printf 'ODOM_PREFLIGHT_SETTINGS=window:%ss attempts:%s confirmations:%s\n' \
    "${ODOM_OFFSET_MEASURE_SECONDS}" \
    "${ODOM_OFFSET_MEASURE_ATTEMPTS}" \
    "${ODOM_OFFSET_CONFIRMATIONS}"
}

if [[ "${mode}" == "--live" || "${mode}" == "--multi-live" ]]; then
  configure_live_odom_preflight

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
if [[ -n "${mission_file}" ]]; then
  printf 'SCAN_MISSION_FILE=%s\n' "${mission_file}"
fi
if [[ -n "${mission_id}" ]]; then
  printf 'SCAN_MISSION_ID=%s\n' "${mission_id}"
fi
if [[ "${mode}" == "--live" || "${mode}" == "--multi-live" ]]; then
  printf 'GO2_SPORT_PROBE_SUMMARY=%s\n' "${GO2_SPORT_PROBE_SUMMARY}"
fi

remove_conflicting_navigation_containers
case "${mode}" in
  --record)
    exec env DDDMR_BAGS_DIR="${BAGS_DIR}" \
      "${DOCKER_WRAPPER}" scan-navigation-record \
      "${mission_container}" "${initial_pose_container}"
    ;;
  --multi-dry-run)
    exec env DDDMR_BAGS_DIR="${BAGS_DIR}" \
      "${SUPERVISOR}" --dry-run
    ;;
  --multi-live)
    exec env DDDMR_BAGS_DIR="${BAGS_DIR}" \
      "${SUPERVISOR}" --live
    ;;
  *)
    exec "${SUPERVISOR}" "${mode}"
    ;;
esac
