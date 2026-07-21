#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/go2_path_tracking.sh <command> [route-id] [probe-summary]

Quick workflow:
  export ROUTE_ID=path_follow_YYYYMMDD_a
  ./scripts/go2_path_tracking.sh config
  ./scripts/go2_path_tracking.sh buffers
  ./scripts/go2_path_tracking.sh build        # once after code changes
  ./scripts/go2_path_tracking.sh record-new   # prepare, record, inspect, dry-start
  ./scripts/go2_path_tracking.sh dry-test     # after returning Go2 to route start
  ./scripts/go2_path_tracking.sh dry-disable
  ./scripts/go2_path_tracking.sh check
  ./scripts/go2_path_tracking.sh dry-stop
  ./scripts/go2_path_tracking.sh probe
  GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_..._summary.env \
    ./scripts/go2_path_tracking.sh live

Commands:
  config            Show the selected route file, map, and navigation config.
  buffers           Set the temporary host DDS receive buffers with sudo.
  build             Build the navigation stack in the supported Docker image.
  record-new        Prepare, record, inspect, and load one new route in dry-run.
  prepare-recording Start localization and a disabled dry-run controller.
  record            Record a new route while driving with the original remote.
  inspect           Inspect the saved route JSON without ROS or motion.
  dry-start         Start the selected route with physical output disconnected.
  dry-status        Show localization and controller status.
  dry-verify        Verify continuous local-observation updates for 30 seconds.
  dry-test          Run status + observation checks, then enable dry-run only.
  dry-enable        Enable calculation on dry-run topics only; no Sport output.
  dry-disable       Disable the controller and reset route progress.
  dry-stop          Stop the dry-run container and save its log.
  check             Run the offline route/map/live-policy acceptance gate.
  probe             Verify live Move/StopMove with a 0.05 m/s handshake probe.
  live              Run supervised route tracking on the physical Go2.
  help              Show this help.

Route selection:
  Set ROUTE_ID once, or pass the route id after commands that need a route.
  Route files default to <repo>/bags/routes/<route-id>.json.
  The map defaults to pose_graph_dir in go2_xt16_navigation.yaml.

Optional environment:
  ROUTE_MAP_DIR=<host map directory override>
  DDDMR_BAGS_DIR=<host bags directory override>
  NAV_CONFIG_FILE=<navigation YAML override>
  GO2_RECORDED_ROUTE_MAX_LENGTH_M=5.0
  GO2_SPORT_MAX_X=0.30
  GO2_RECORDED_ROUTE_LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC=0.35
  OVERWRITE_ROUTE=false

Safety:
  The default/help, build, inspect, check, and dry-run commands do not connect
  route velocity to the physical Go2. The probe and live commands require an
  interactive supervision phrase before delegating to the existing fail-closed
  launchers. The live launcher also requires `ENABLE <route-id>` later.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
NAV_CONFIG="${NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
DRY_RUNNER="${SCRIPT_DIR}/run_go2_xt16_recorded_route_dry_run.sh"
RECORDER="${SCRIPT_DIR}/record_go2_xt16_recorded_route.sh"
LIVE_RUNNER="${SCRIPT_DIR}/run_go2_xt16_recorded_route_supervised_live.sh"
SPORT_PROBE="${SCRIPT_DIR}/run_go2_sport_adapter_supervised_probe.sh"
DOCKER_WRAPPER="${SCRIPT_DIR}/dddmr_docker_go2_xt16.sh"
ROUTE_TOOL="${WS_ROOT}/src/dddmr_route_navigation/scripts/route_tool.py"
BUFFER_CHECK="${SCRIPT_DIR}/check_go2_dds_receive_buffers.sh"
SELF="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"

require_file() {
  [[ -f "$1" ]] || die "Missing file: $1"
}

require_executable() {
  [[ -x "$1" ]] || die "Missing executable: $1"
}

route_id_value() {
  local value="${1:-${ROUTE_ID:-}}"
  [[ -n "${value}" ]] || die "Set ROUTE_ID or pass a route id to this command."
  [[ "${value}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
    die "Route id may contain only letters, digits, dot, underscore, and hyphen."
  printf '%s\n' "${value}"
}

route_file_value() {
  printf '%s/routes/%s.json\n' "${BAGS_DIR}" "$1"
}

configured_map_dir() {
  if [[ -n "${ROUTE_MAP_DIR:-}" ]]; then
    realpath -m -- "${ROUTE_MAP_DIR}"
    return
  fi

  require_file "${NAV_CONFIG}"
  local container_path relative
  container_path="$(awk -F'"' '
    /^[[:space:]]*pose_graph_dir:[[:space:]]*"\/root\/dddmr_bags\// {
      print $2
      exit
    }
  ' "${NAV_CONFIG}")"
  [[ -n "${container_path}" ]] || \
    die "Could not find a /root/dddmr_bags pose_graph_dir in ${NAV_CONFIG}."
  relative="${container_path#/root/dddmr_bags/}"
  realpath -m -- "${BAGS_DIR}/${relative}"
}

validate_map() {
  local map_dir="$1"
  [[ "${map_dir}" == "$(realpath -m -- "${BAGS_DIR}")"/* ]] || \
    die "Route map must stay under ${BAGS_DIR}: ${map_dir}"
  require_file "${map_dir}/poses.pcd"
  require_file "${map_dir}/map.pcd"
  require_file "${map_dir}/ground.pcd"
  require_file "${map_dir}/edges.pcd"
}

run_dry_command() {
  local action="$1"
  local route_id="$2"
  local map_dir route_file
  map_dir="$(configured_map_dir)"
  validate_map "${map_dir}"
  route_file="$(route_file_value "${route_id}")"

  ROUTE_MAP_DIR="${map_dir}" \
  ROUTE_FILE="${route_file}" \
  ROUTE_ID="${route_id}" \
    "${DRY_RUNNER}" "${action}"
}

confirm_phrase() {
  local expected="$1"
  local purpose="$2"
  local answer
  [[ -t 0 ]] || die "${purpose} requires an interactive terminal."
  printf '%s\n' "${purpose}"
  printf 'Type exactly: %s\n> ' "${expected}"
  IFS= read -r answer
  [[ "${answer}" == "${expected}" ]] || die "Confirmation did not match; no live command was started."
}

command="${1:-help}"
if (( $# > 0 )); then
  shift
fi

require_executable "${DRY_RUNNER}"
require_executable "${RECORDER}"
require_executable "${LIVE_RUNNER}"
require_executable "${SPORT_PROBE}"
require_executable "${DOCKER_WRAPPER}"
require_executable "${ROUTE_TOOL}"
require_executable "${BUFFER_CHECK}"

case "${command}" in
  help|-h|--help)
    usage
    ;;

  config)
    (( $# <= 1 )) || die "Usage: $0 config [route-id]"
    route_id="$(route_id_value "${1:-}")"
    map_dir="$(configured_map_dir)"
    validate_map "${map_dir}"
    printf 'ROUTE_ID=%s\n' "${route_id}"
    printf 'ROUTE_FILE=%s\n' "$(route_file_value "${route_id}")"
    printf 'ROUTE_MAP_DIR=%s\n' "${map_dir}"
    printf 'NAV_CONFIG_FILE=%s\n' "${NAV_CONFIG}"
    ;;

  buffers)
    (( $# == 0 )) || die "buffers does not take arguments."
    log "Setting temporary host DDS receive buffers to 16777216 bytes."
    sudo sysctl -w net.core.rmem_max=16777216
    sudo sysctl -w net.core.rmem_default=16777216
    "${BUFFER_CHECK}"
    ;;

  build)
    (( $# == 0 )) || die "build does not take arguments."
    "${DOCKER_WRAPPER}" build-navigation
    ;;

  record-new)
    (( $# <= 1 )) || die "Usage: $0 record-new [route-id]"
    route_id="$(route_id_value "${1:-}")"
    route_file="$(route_file_value "${route_id}")"
    if [[ -e "${route_file}" && "${OVERWRITE_ROUTE:-false}" != "true" ]]; then
      die "Route already exists: ${route_file}. Choose another ROUTE_ID or explicitly set OVERWRITE_ROUTE=true."
    fi
    [[ -t 0 ]] || die "record-new requires an interactive terminal for route recording."
    "${SELF}" prepare-recording
    ROUTE_ID="${route_id}" "${SELF}" record
    ROUTE_ID="${route_id}" "${SELF}" inspect
    "${SELF}" dry-stop
    ROUTE_ID="${route_id}" "${SELF}" dry-start
    log "New route is loaded in dry-run and remains physically disconnected."
    log "Return the Go2 to the route start with the original remote, then run:"
    printf '  ROUTE_ID=%q %q dry-test\n' "${route_id}" "${SELF}"
    ;;

  prepare-recording)
    (( $# == 0 )) || die "prepare-recording does not take a route id."
    map_dir="$(configured_map_dir)"
    validate_map "${map_dir}"
    setup_id="recording_setup_current"
    setup_file="$(route_file_value "${setup_id}")"
    mkdir -p -- "${BAGS_DIR}/routes"
    log "Using navigation map: ${map_dir}"
    log "Starting localization with physical motion output disconnected."
    ROUTE_MAP_DIR="${map_dir}" \
    ROUTE_FILE="${setup_file}" \
    ROUTE_ID="${setup_id}" \
    REGENERATE_ROUTE=true \
    RVIZ="${RVIZ:-true}" \
      "${DRY_RUNNER}" --start
    ;;

  record)
    (( $# <= 1 )) || die "Usage: $0 record [route-id]"
    route_id="$(route_id_value "${1:-}")"
    map_dir="$(configured_map_dir)"
    validate_map "${map_dir}"
    route_file="$(route_file_value "${route_id}")"
    mkdir -p -- "${BAGS_DIR}/routes"
    log "Recording ${route_id} to ${route_file}."
    log "Drive only with the original remote; press Ctrl-C at the route end."
    ROUTE_ID="${route_id}" \
    ROUTE_FILE="${route_file}" \
    ROUTE_MAP_DIR="${map_dir}" \
    OVERWRITE_ROUTE="${OVERWRITE_ROUTE:-false}" \
      "${RECORDER}" --record
    ;;

  inspect)
    (( $# <= 1 )) || die "Usage: $0 inspect [route-id]"
    route_id="$(route_id_value "${1:-}")"
    route_file="$(route_file_value "${route_id}")"
    require_file "${route_file}"
    python3 "${ROUTE_TOOL}" inspect "${route_file}"
    ;;

  dry-start)
    (( $# <= 1 )) || die "Usage: $0 dry-start [route-id]"
    route_id="$(route_id_value "${1:-}")"
    route_file="$(route_file_value "${route_id}")"
    require_file "${route_file}"
    run_dry_command --start "${route_id}"
    ;;

  dry-status)
    (( $# == 0 )) || die "dry-status does not take arguments."
    "${DRY_RUNNER}" --status
    ;;

  dry-verify)
    (( $# == 0 )) || die "dry-verify does not take arguments."
    "${DRY_RUNNER}" --verify-observation
    ;;

  dry-test)
    (( $# == 0 )) || die "dry-test does not take arguments."
    "${DRY_RUNNER}" --status
    "${DRY_RUNNER}" --verify-observation
    "${DRY_RUNNER}" --enable
    log "Dry-run calculation is enabled, but physical Sport output is disconnected."
    log "Drive the route manually, inspect progress, then run dry-disable."
    ;;

  dry-enable)
    (( $# == 0 )) || die "dry-enable does not take arguments."
    "${DRY_RUNNER}" --enable
    ;;

  dry-disable)
    (( $# == 0 )) || die "dry-disable does not take arguments."
    "${DRY_RUNNER}" --disable
    ;;

  dry-stop|stop)
    (( $# == 0 )) || die "dry-stop does not take arguments."
    "${DRY_RUNNER}" --stop
    ;;

  check)
    (( $# <= 1 )) || die "Usage: $0 check [route-id]"
    route_id="$(route_id_value "${1:-}")"
    route_file="$(route_file_value "${route_id}")"
    map_dir="$(configured_map_dir)"
    validate_map "${map_dir}"
    require_file "${route_file}"
    GO2_RECORDED_ROUTE_EXPECTED_ID="${route_id}" \
    GO2_RECORDED_ROUTE_MAX_LENGTH_M="${GO2_RECORDED_ROUTE_MAX_LENGTH_M:-5.0}" \
    ROUTE_FILE="${route_file}" \
    ROUTE_MAP_DIR="${map_dir}" \
      "${LIVE_RUNNER}" --check
    ;;

  probe)
    (( $# == 0 )) || die "probe does not take arguments."
    confirm_phrase I_AM_SUPERVISING_GO2 \
      "This live probe can move the Go2; its 0.05 m/s command verifies Move/StopMove, not physical displacement."
    GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
    GO2_SPORT_PROBE_X=0.05 \
    GO2_SPORT_PROBE_Y=0.0 \
    GO2_SPORT_PROBE_YAW=0.0 \
    GO2_SPORT_MAX_X=0.10 \
    GO2_SPORT_MAX_Y=0.0 \
    GO2_SPORT_MAX_YAW=0.20 \
    GO2_SPORT_PROBE_DECISION=d_controlling \
    GO2_REQUIRE_MOTION_DECISION=true \
    GO2_MOTION_ALLOWED_DECISIONS=d_controlling,d_align_heading,d_align_goal_heading \
      "${SPORT_PROBE}" --live
    ;;

  live)
    (( $# <= 2 )) || die "Usage: $0 live [route-id] [probe-summary]"
    route_id="$(route_id_value "${1:-}")"
    probe_summary="${2:-${GO2_SPORT_PROBE_SUMMARY:-}}"
    [[ -n "${probe_summary}" ]] || \
      die "Set GO2_SPORT_PROBE_SUMMARY or pass the probe summary as the second argument."
    require_file "${probe_summary}"
    route_file="$(route_file_value "${route_id}")"
    map_dir="$(configured_map_dir)"
    validate_map "${map_dir}"
    require_file "${route_file}"
    confirm_phrase I_AM_SUPERVISING_GO2_RECORDED_ROUTE \
      "This command can move the physical Go2 along route ${route_id}."
    GO2_RECORDED_ROUTE_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_RECORDED_ROUTE \
    GO2_SPORT_PROBE_SUMMARY="${probe_summary}" \
    GO2_SPORT_MAX_X="${GO2_SPORT_MAX_X:-0.30}" \
    GO2_RECORDED_ROUTE_EXPECTED_ID="${route_id}" \
    GO2_RECORDED_ROUTE_MAX_LENGTH_M="${GO2_RECORDED_ROUTE_MAX_LENGTH_M:-5.0}" \
    ROUTE_FILE="${route_file}" \
    ROUTE_MAP_DIR="${map_dir}" \
    RVIZ="${RVIZ:-true}" \
      "${LIVE_RUNNER}" --live
    ;;

  *)
    usage >&2
    die "Unknown command: ${command}"
    ;;
esac
