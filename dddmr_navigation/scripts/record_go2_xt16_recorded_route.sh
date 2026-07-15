#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/record_go2_xt16_recorded_route.sh [--record|--status]

Record a route only while a recorded-route dry-run container is already
running.  The recorder subscribes to localized poses; it never publishes a
motion command.  Drive the robot only with the original remote controller
under normal on-site supervision.

Commands:
  --record       Confirm localization is TRACKING and record manual poses.
                 This is the default.  Press Ctrl-C after the manual traverse
                 to validate and save the JSON route atomically.
  --status       Show the localization and recorded-route controller status.
  -h, --help     Show this help.

Environment overrides:
  ROUTE_ID=recorded_route_YYYYMMDD_HHMMSS
  ROUTE_FILE=<host JSON path under DDDMR_BAGS_DIR>
  ROUTE_MAP_DIR=<host pose-graph map directory>
  DDDMR_BAGS_DIR=<repo-root>/bags
  DDDMR_INSTALL_BASE=.docker_go2_xt16_install
  OVERWRITE_ROUTE=false    Set true only to replace an existing route file.
  SAMPLE_DISTANCE=0.10
  SAMPLE_YAW=0.15
  MAX_SEGMENT_LENGTH=2.0

Safety:
  Start the dry-run stack first with run_go2_xt16_recorded_route_dry_run.sh.
  Do not run that stack with --enable while recording.  This script never
  publishes /cmd_vel, /api/sport/request, or /lowcmd.
EOF
}

mode="record"
if (( $# > 1 )); then
  usage >&2
  exit 2
fi
case "${1:---record}" in
  --record) mode="record" ;;
  --status) mode="status" ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown command: $1" >&2
    usage >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"

BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
DEFAULT_MAP_NAME="go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36"
MAP_DIR="${ROUTE_MAP_DIR:-${BAGS_DIR}/${DEFAULT_MAP_NAME}}"
ROUTE_ID_VALUE="${ROUTE_ID:-recorded_route_$(date +%Y%m%d_%H%M%S)}"
ROUTE_FILE_VALUE="${ROUTE_FILE:-${BAGS_DIR}/routes/${ROUTE_ID_VALUE}.json}"
NAV_CONFIG_FILE_VALUE="${NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
ROUTE_TOOL="${WS_ROOT}/src/dddmr_route_navigation/scripts/route_tool.py"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
OVERWRITE_ROUTE_VALUE="${OVERWRITE_ROUTE:-false}"
SAMPLE_DISTANCE_VALUE="${SAMPLE_DISTANCE:-0.10}"
SAMPLE_YAW_VALUE="${SAMPLE_YAW:-0.15}"
MAX_SEGMENT_LENGTH_VALUE="${MAX_SEGMENT_LENGTH:-2.0}"

CONTAINER_LABEL="dddmr.go2_xt16_recorded_route_dry_run=true"

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

validate_bool() {
  local name="$1"
  local value="$2"
  case "${value}" in
    true|false) ;;
    *) die "${name} must be true or false; got '${value}'." ;;
  esac
}

validate_positive_number() {
  local name="$1"
  local value="$2"
  awk -v value="${value}" 'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }' || \
    die "${name} must be a positive decimal number; got '${value}'."
}

path_inside() {
  local base target
  base="$(realpath -m -- "$1")"
  target="$(realpath -m -- "$2")"
  [[ "${target}" == "${base}"/* ]] || return 1
  printf '%s\n' "${target#"${base}"/}"
}

running_container() {
  local -a names=()
  mapfile -t names < <(
    docker ps --filter "label=${CONTAINER_LABEL}" --format '{{.Names}}' 2>/dev/null
  )
  if (( ${#names[@]} == 0 )); then
    die "No recorded-route dry-run container is running. Start it first with run_go2_xt16_recorded_route_dry_run.sh."
  fi
  if (( ${#names[@]} > 1 )); then
    printf '%s\n' "${names[@]}" >&2
    die "Multiple recorded-route dry-run containers are running; stop the extras first."
  fi
  printf '%s\n' "${names[0]}"
}

docker_ros() {
  local container="$1"
  shift
  docker exec "${container}" bash -lc "set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
cd /root/dddmr_navigation
$*"
}

localization_status() {
  local container="$1"
  docker_ros "${container}" \
    "timeout 8 ros2 topic echo --no-daemon --spin-time 3 /localization_status std_msgs/msg/String --once" \
    2>/dev/null || true
}

require_tracking() {
  local container="$1"
  local status
  status="$(localization_status "${container}")"
  case "${status}" in
    *"data: TRACKING"*|*"data: 'TRACKING'"*|*'data: "TRACKING"'*) ;;
    *)
      printf '%s\n' "${status}" >&2
      die "Localization is not TRACKING; keep the robot stationary until MCL is stable, then retry."
      ;;
  esac
}

require_controller_disabled() {
  local container="$1"
  local status
  status="$(docker_ros "${container}" \
    "timeout 8 ros2 topic echo --no-daemon --spin-time 3 /recorded_route_controller/status std_msgs/msg/String --once" \
    2>/dev/null || true)"
  case "${status}" in
    *"READY:"*|*"DISABLED:"*) ;;
    *)
      printf '%s\n' "${status}" >&2
      die "Recorded-route controller is not READY/DISABLED; run run_go2_xt16_recorded_route_dry_run.sh --disable before recording."
      ;;
  esac
}

show_status() {
  local container="$1"
  log "Localization status from ${container}:"
  localization_status "${container}"
  log "Recorded-route controller status from ${container}:"
  docker_ros "${container}" \
    "timeout 8 ros2 topic echo --no-daemon --spin-time 3 /recorded_route_controller/status std_msgs/msg/String --once" \
    || true
}

record_route() {
  local container route_relative map_relative container_route container_map
  validate_bool OVERWRITE_ROUTE "${OVERWRITE_ROUTE_VALUE}"
  validate_positive_number SAMPLE_DISTANCE "${SAMPLE_DISTANCE_VALUE}"
  validate_positive_number SAMPLE_YAW "${SAMPLE_YAW_VALUE}"
  validate_positive_number MAX_SEGMENT_LENGTH "${MAX_SEGMENT_LENGTH_VALUE}"
  [[ "${ROUTE_ID_VALUE}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
    die "ROUTE_ID may contain only letters, digits, dot, underscore, and hyphen."
  [[ -x "${ROUTE_TOOL}" ]] || die "Route tool is missing or not executable: ${ROUTE_TOOL}"
  [[ -f "${MAP_DIR}/poses.pcd" ]] || die "Missing pose graph: ${MAP_DIR}/poses.pcd"
  [[ -f "${NAV_CONFIG_FILE_VALUE}" ]] || die "Missing navigation config: ${NAV_CONFIG_FILE_VALUE}"

  route_relative="$(path_inside "${BAGS_DIR}" "${ROUTE_FILE_VALUE}")" || \
    die "ROUTE_FILE must be under DDDMR_BAGS_DIR so the dry-run container can save it."
  map_relative="$(path_inside "${BAGS_DIR}" "${MAP_DIR}")" || \
    die "ROUTE_MAP_DIR must be under DDDMR_BAGS_DIR."
  container_route="/root/dddmr_bags/${route_relative}"
  container_map="/root/dddmr_bags/${map_relative}"
  grep -Fq "pose_graph_dir: \"${container_map}\"" "${NAV_CONFIG_FILE_VALUE}" || \
    die "Navigation config does not select ROUTE_MAP_DIR: ${container_map}"
  if [[ -e "${ROUTE_FILE_VALUE}" && "${OVERWRITE_ROUTE_VALUE}" != "true" ]]; then
    die "Route file already exists: ${ROUTE_FILE_VALUE}. Choose another ROUTE_ID or set OVERWRITE_ROUTE=true deliberately."
  fi

  container="$(running_container)"
  require_tracking "${container}"
  require_controller_disabled "${container}"
  mkdir -p "$(dirname "${ROUTE_FILE_VALUE}")"

  log "Recording only localized manual poses into ${ROUTE_FILE_VALUE}"
  log "The dry-run controller stays disabled. Drive only with the original remote controller."
  log "Press Ctrl-C after the manual traverse to save and validate the route."

  docker exec -it "${container}" bash -lc '
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source "/root/dddmr_navigation/$DDDMR_INSTALL_BASE/setup.bash"
set -u
exec ros2 run dddmr_route_navigation route_tool.py record "$@"
' bash \
    "${container_route}" \
    --route-id "${ROUTE_ID_VALUE}" \
    --map-dir "${container_map}" \
    --pose-topic /mcl_pose \
    --status-topic /localization_status \
    --sample-distance "${SAMPLE_DISTANCE_VALUE}" \
    --sample-yaw "${SAMPLE_YAW_VALUE}" \
    --max-segment-length "${MAX_SEGMENT_LENGTH_VALUE}"

  [[ -f "${ROUTE_FILE_VALUE}" ]] || die "Recorder exited without creating ${ROUTE_FILE_VALUE}."
  log "Validating saved route..."
  python3 "${ROUTE_TOOL}" inspect "${ROUTE_FILE_VALUE}"
  log "Saved route: ${ROUTE_FILE_VALUE}"
  log "Restart dry-run with: ROUTE_FILE=${ROUTE_FILE_VALUE} ROUTE_MAP_DIR=${MAP_DIR} ${SCRIPT_DIR}/run_go2_xt16_recorded_route_dry_run.sh"
}

container="$(running_container)"
case "${mode}" in
  status) show_status "${container}" ;;
  record) record_route ;;
esac
