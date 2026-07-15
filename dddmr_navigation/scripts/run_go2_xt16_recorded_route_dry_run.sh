#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_recorded_route_dry_run.sh [command]

Commands:
  --start         Prepare the route, run the read-only XT16 preflight, and
                  start recorded-route navigation in detached dry-run mode.
                  This is the default.
  --prepare-only  Generate/validate the route without Docker or live ROS access.
  --status        Show the dry-run container and controller status.
  --enable        Explicitly enable recorded-route control after checking that
                  localization is TRACKING and no real Sport publisher exists.
                  Output remains on dry-run topics only.
  --disable       Disable recorded-route control and reset route progress.
  --stop          Stop the recorded-route dry-run container and save its log.
  -h, --help      Show this help.

Environment overrides:
  ROUTE_FILE=<host JSON path under DDDMR_BAGS_DIR>
  ROUTE_MAP_DIR=<host pose-graph map directory>
  ROUTE_ID=go2_xt16_indoor_reverse_test
  NAV_CONFIG_FILE=<host go2_xt16_navigation.yaml path>
  DDDMR_BAGS_DIR=<repo-parent>/bags
  DDDMR_IMAGE=dddmr_go2_xt16:x64
  ROS_DOMAIN_ID=0
  GO2_DDS_IP=192.168.123.18
  GO2_NET_IFACE=<auto>
  RVIZ=true
  PUBLISH_STATIC_TF=true
  BUILD=false              Set true to rebuild navigation before starting.
  REGENERATE_ROUTE=false  Set true to overwrite ROUTE_FILE from poses.pcd.
  PREFLIGHT_SAMPLES=3
  PREFLIGHT_TIMEOUT=10

Safety:
  This script has no live mode. The launch keeps the real Sport adapter off,
  remaps /cmd_vel to /dddmr_go2/dry_run_cmd_vel, and never publishes
  /api/sport/request.
EOF
}

mode="start"
if (( $# > 1 )); then
  usage >&2
  exit 2
fi
case "${1:---start}" in
  --start) mode="start" ;;
  --prepare-only) mode="prepare-only" ;;
  --status) mode="status" ;;
  --enable) mode="enable" ;;
  --disable) mode="disable" ;;
  --stop) mode="stop" ;;
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

IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
DEFAULT_MAP_NAME="go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36"
MAP_DIR="${ROUTE_MAP_DIR:-${BAGS_DIR}/${DEFAULT_MAP_NAME}}"
ROUTE_FILE_VALUE="${ROUTE_FILE:-${BAGS_DIR}/routes/go2_xt16_indoor_reverse_test.json}"
ROUTE_ID_VALUE="${ROUTE_ID:-go2_xt16_indoor_reverse_test}"
NAV_CONFIG_FILE_VALUE="${NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
ROUTE_TOOL="${WS_ROOT}/src/dddmr_route_navigation/scripts/route_tool.py"
DOCKER_WRAPPER="${SCRIPT_DIR}/dddmr_docker_go2_xt16.sh"
RUNTIME_CLEAN_CHECK="${SCRIPT_DIR}/check_go2_xt16_no_motion_runtime_clean.sh"

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-}"
BUILD_BASE_VALUE="${DDDMR_BUILD_BASE:-.docker_go2_xt16_build}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
LOG_BASE_VALUE="${DDDMR_LOG_BASE:-.docker_go2_xt16_log}"
RVIZ_VALUE="${RVIZ:-true}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
BUILD_VALUE="${BUILD:-false}"
REGENERATE_ROUTE_VALUE="${REGENERATE_ROUTE:-false}"
PREFLIGHT_SAMPLES_VALUE="${PREFLIGHT_SAMPLES:-3}"
PREFLIGHT_TIMEOUT_VALUE="${PREFLIGHT_TIMEOUT:-10}"
RUN_LOG_DIR="${RUN_LOG_DIR:-${WS_ROOT}/run_logs}"

CONTAINER_LABEL="dddmr.go2_xt16_recorded_route_dry_run=true"
CONTAINER_NAME="${RECORDED_ROUTE_CONTAINER_NAME:-go2_xt16_recorded_route_dry_run_$(date +%Y%m%d_%H%M%S)}"

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

require_positive_integer() {
  local name="$1"
  local value="$2"
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || \
    die "${name} must be a positive integer; got '${value}'."
}

recorded_route_containers() {
  docker ps -a --filter "label=${CONTAINER_LABEL}" --format '{{.Names}}' 2>/dev/null || true
}

running_container() {
  local -a names=()
  mapfile -t names < <(
    docker ps --filter "label=${CONTAINER_LABEL}" --format '{{.Names}}' 2>/dev/null
  )
  if (( ${#names[@]} == 0 )); then
    die "No recorded-route dry-run container is running. Start it with --start."
  fi
  if (( ${#names[@]} > 1 )); then
    printf '%s\n' "${names[@]}" >&2
    die "Multiple recorded-route containers are running; stop them before continuing."
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

save_and_remove_container() {
  local container="$1"
  mkdir -p "${RUN_LOG_DIR}"
  if docker inspect "${container}" >/dev/null 2>&1; then
    if docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -Fxq true; then
      log "Stopping ${container}..."
      docker stop -t 5 "${container}" >/dev/null || true
    fi
    local log_path="${RUN_LOG_DIR}/${container}_docker.log"
    if docker logs "${container}" >"${log_path}" 2>&1; then
      log "Saved log: ${log_path}"
    fi
    docker rm "${container}" >/dev/null 2>&1 || true
  fi
}

stop_all() {
  local names name
  names="$(recorded_route_containers)"
  if [[ -z "${names}" ]]; then
    log "No recorded-route dry-run container exists."
    return 0
  fi
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    save_and_remove_container "${name}"
  done <<<"${names}"
}

map_fingerprint() {
  local name path hash count=0
  for name in poses.pcd map.pcd ground.pcd edges.pcd; do
    if [[ -f "${MAP_DIR}/${name}" ]]; then
      count=$((count + 1))
    fi
  done
  (( count > 0 )) || die "Map directory has no fingerprintable PCD files: ${MAP_DIR}"

  {
    for name in poses.pcd map.pcd ground.pcd edges.pcd; do
      path="${MAP_DIR}/${name}"
      [[ -f "${path}" ]] || continue
      hash="$(sha256sum -- "${path}" | awk '{print $1}')"
      printf '%s%s' "${name}" "${hash}"
    done
  } | sha256sum | awk '{print $1}'
}

prepare_route() {
  [[ -x "${ROUTE_TOOL}" ]] || die "Route tool is missing or not executable: ${ROUTE_TOOL}"
  [[ -f "${MAP_DIR}/poses.pcd" ]] || die "Missing pose graph: ${MAP_DIR}/poses.pcd"

  if [[ ! -f "${ROUTE_FILE_VALUE}" || "${REGENERATE_ROUTE_VALUE}" == "true" ]]; then
    log "Generating route: ${ROUTE_FILE_VALUE}"
    mkdir -p "$(dirname "${ROUTE_FILE_VALUE}")"
    python3 "${ROUTE_TOOL}" convert-pose-graph \
      "${MAP_DIR}/poses.pcd" \
      "${ROUTE_FILE_VALUE}" \
      --route-id "${ROUTE_ID_VALUE}" \
      --map-dir "${MAP_DIR}"
  fi

  local summary expected_sha route_sha
  summary="$(python3 "${ROUTE_TOOL}" inspect "${ROUTE_FILE_VALUE}")"
  printf '%s\n' "${summary}"
  route_sha="$(
    sed -nE 's/^[[:space:]]*"map_sha256": "([0-9a-f]+)",?$/\1/p' <<<"${summary}"
  )"
  [[ -n "${route_sha}" ]] || die "Route does not contain a map fingerprint."
  expected_sha="$(map_fingerprint)"
  [[ "${route_sha}" == "${expected_sha}" ]] || \
    die "Route/map fingerprint mismatch. Set REGENERATE_ROUTE=true after verifying the map."
  log "Route and map fingerprints match: ${route_sha}"
}

path_inside() {
  local base target
  base="$(realpath -m -- "$1")"
  target="$(realpath -m -- "$2")"
  [[ "${target}" == "${base}"/* ]] || return 1
  printf '%s\n' "${target#"${base}"/}"
}

ensure_navigation_install() {
  local setup_file="${WS_ROOT}/${INSTALL_BASE_VALUE}/setup.bash"
  local route_node="${WS_ROOT}/${INSTALL_BASE_VALUE}/dddmr_route_navigation/lib/dddmr_route_navigation/recorded_route_move_base_node"
  if [[ "${BUILD_VALUE}" == "true" || ! -f "${setup_file}" || ! -x "${route_node}" ]]; then
    log "Building navigation in the authoritative Docker environment..."
    "${DOCKER_WRAPPER}" build-navigation
  fi
  [[ -f "${setup_file}" ]] || die "Navigation install is missing: ${setup_file}"
  [[ -x "${route_node}" ]] || die "Recorded-route node is missing: ${route_node}"
}

check_no_real_sport_publisher() {
  local container="$1"
  local nodes info publishers
  nodes="$(docker_ros "${container}" "timeout 8 ros2 node list" 2>&1)" || {
    printf '%s\n' "${nodes}" >&2
    die "Could not inspect ROS nodes."
  }
  if rg -q '(^|/)go2_sport_cmd_vel_adapter([^[:alnum:]]|$)' <<<"${nodes}"; then
    die "A real Sport adapter node is present; refusing dry-run enable."
  fi

  info="$(docker_ros "${container}" "timeout 8 ros2 topic info /api/sport/request" 2>&1 || true)"
  publishers="$(awk '/^Publisher count:/ {print $3; exit}' <<<"${info}")"
  if [[ -n "${publishers}" && "${publishers}" != "0" ]]; then
    printf '%s\n' "${info}" >&2
    die "/api/sport/request has ${publishers} publisher(s); refusing dry-run enable."
  fi
}

show_status() {
  local names name
  names="$(recorded_route_containers)"
  if [[ -z "${names}" ]]; then
    log "No recorded-route dry-run container exists."
    return 1
  fi
  docker ps -a --filter "label=${CONTAINER_LABEL}" \
    --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if docker inspect -f '{{.State.Running}}' "${name}" 2>/dev/null | grep -Fxq true; then
      log "Controller status from ${name}:"
      docker_ros "${name}" \
        "timeout 8 ros2 topic echo /recorded_route_controller/status std_msgs/msg/String --once" || true
      docker_ros "${name}" \
        "timeout 8 ros2 topic echo /recorded_route_controller/route_ready std_msgs/msg/Bool --once" || true
      docker_ros "${name}" \
        "timeout 8 ros2 topic echo /localization_status std_msgs/msg/String --once" || true
    fi
  done <<<"${names}"
}

enable_dry_run() {
  local container ready localization response
  container="$(running_container)"
  check_no_real_sport_publisher "${container}"
  ready="$(docker_ros "${container}" \
    "timeout 8 ros2 topic echo /recorded_route_controller/route_ready std_msgs/msg/Bool --once")"
  grep -Eq '^data: true$' <<<"${ready}" || die "Recorded route is not ready."
  localization="$(docker_ros "${container}" \
    "timeout 8 ros2 topic echo /localization_status std_msgs/msg/String --once")"
  case "${localization}" in
    *"data: TRACKING"*|*"data: 'TRACKING'"*|*'data: "TRACKING"'*) ;;
    *)
      printf '%s\n' "${localization}" >&2
      die "Localization is not TRACKING; refusing dry-run enable."
      ;;
  esac
  response="$(docker_ros "${container}" \
    "ros2 service call /recorded_route_controller/set_enabled std_srvs/srv/SetBool '{data: true}'")"
  printf '%s\n' "${response}"
  grep -Eq 'success=(True|true)|success: true' <<<"${response}" || \
    die "Controller refused enable; verify that the robot is inside the route start envelope."
  log "Recorded-route control enabled on dry-run topics only."
}

disable_dry_run() {
  local container
  container="$(running_container)"
  docker_ros "${container}" \
    "ros2 service call /recorded_route_controller/set_enabled std_srvs/srv/SetBool '{data: false}'"
  log "Recorded-route control disabled and progress reset."
}

wait_for_route_ready() {
  local container="$1"
  local deadline=$((SECONDS + 90))
  local output
  while (( SECONDS < deadline )); do
    if ! docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -Fxq true; then
      return 1
    fi
    output="$(docker_ros "${container}" \
      "timeout 5 ros2 topic echo /recorded_route_controller/route_ready std_msgs/msg/Bool --once" \
      2>/dev/null || true)"
    if grep -Eq '^data: true$' <<<"${output}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_dry_run() {
  validate_bool RVIZ "${RVIZ_VALUE}"
  validate_bool PUBLISH_STATIC_TF "${PUBLISH_STATIC_TF_VALUE}"
  validate_bool BUILD "${BUILD_VALUE}"
  validate_bool REGENERATE_ROUTE "${REGENERATE_ROUTE_VALUE}"
  require_positive_integer PREFLIGHT_SAMPLES "${PREFLIGHT_SAMPLES_VALUE}"
  require_positive_integer PREFLIGHT_TIMEOUT "${PREFLIGHT_TIMEOUT_VALUE}"

  docker image inspect "${IMAGE}" >/dev/null 2>&1 || \
    die "Docker image ${IMAGE} not found."
  [[ -z "$(recorded_route_containers)" ]] || \
    die "A recorded-route container already exists. Use --status or --stop."

  prepare_route
  ensure_navigation_install

  local route_relative map_relative config_relative
  route_relative="$(path_inside "${BAGS_DIR}" "${ROUTE_FILE_VALUE}")" || \
    die "ROUTE_FILE must be under DDDMR_BAGS_DIR so Docker can read it."
  map_relative="$(path_inside "${BAGS_DIR}" "${MAP_DIR}")" || \
    die "ROUTE_MAP_DIR must be under DDDMR_BAGS_DIR."
  config_relative="$(path_inside "${WS_ROOT}" "${NAV_CONFIG_FILE_VALUE}")" || \
    die "NAV_CONFIG_FILE must be inside the dddmr_navigation workspace."
  [[ -f "${NAV_CONFIG_FILE_VALUE}" ]] || die "Missing navigation config: ${NAV_CONFIG_FILE_VALUE}"

  local container_map_dir="/root/dddmr_bags/${map_relative}"
  rg -Fq "pose_graph_dir: \"${container_map_dir}\"" "${NAV_CONFIG_FILE_VALUE}" || \
    die "Navigation config does not select the route map: ${container_map_dir}"

  log "Checking for stale navigation/motion runtimes..."
  "${RUNTIME_CLEAN_CHECK}"
  log "Running read-only XT16 preflight..."
  "${DOCKER_WRAPPER}" preflight \
    --samples "${PREFLIGHT_SAMPLES_VALUE}" \
    --timeout "${PREFLIGHT_TIMEOUT_VALUE}"

  if [[ "${RVIZ_VALUE}" == "true" && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null || true
  fi

  local -a docker_args=(
    --name "${CONTAINER_NAME}"
    --label "${CONTAINER_LABEL}"
    --label "dddmr.go2_xt16_navigation=true"
    --network=host
    --privileged
    -e "DISPLAY=${DISPLAY:-:0}"
    -e "QT_X11_NO_MITSHM=1"
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
    -e "GO2_DDS_IP=${GO2_DDS_IP_VALUE}"
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    -e "DDDMR_BUILD_BASE=${BUILD_BASE_VALUE}"
    -e "DDDMR_INSTALL_BASE=${INSTALL_BASE_VALUE}"
    -e "DDDMR_LOG_BASE=${LOG_BASE_VALUE}"
    -v "/tmp:/tmp"
    -v "/dev:/dev"
    -v "${WS_ROOT}:/root/dddmr_navigation"
    -v "${BAGS_DIR}:/root/dddmr_bags"
  )
  if [[ -n "${GO2_NET_IFACE_VALUE}" ]]; then
    docker_args+=(-e "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}")
  fi

  local container_script
  container_script='set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source "/root/dddmr_navigation/${DDDMR_INSTALL_BASE}/setup.bash"
set -u
cd /root/dddmr_navigation
exec ros2 launch dddmr_beginner_guide go2_xt16_recorded_route_navigation.launch \
  route_file:="$1" \
  config_file:="$2" \
  start_route_file_publisher:=true \
  rviz:="$3" \
  publish_static_tf:="$4"'

  log "Starting fail-closed recorded-route dry-run: ${CONTAINER_NAME}"
  docker run -d "${docker_args[@]}" "${IMAGE}" \
    bash -lc "${container_script}" bash \
    "/root/dddmr_bags/${route_relative}" \
    "/root/dddmr_navigation/${config_relative}" \
    "${RVIZ_VALUE}" \
    "${PUBLISH_STATIC_TF_VALUE}" >/dev/null

  trap 'save_and_remove_container "${CONTAINER_NAME}"; exit 130' INT TERM
  if ! wait_for_route_ready "${CONTAINER_NAME}"; then
    docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
    save_and_remove_container "${CONTAINER_NAME}"
    die "Timed out waiting for a valid recorded route."
  fi
  trap - INT TERM

  check_no_real_sport_publisher "${CONTAINER_NAME}"
  log "Ready, but still DISABLED. No physical motion output is connected."
  log "Inspect: ${SCRIPT_DIR}/run_go2_xt16_recorded_route_dry_run.sh --status"
  log "Enable dry-run calculation: ${SCRIPT_DIR}/run_go2_xt16_recorded_route_dry_run.sh --enable"
  log "Stop: ${SCRIPT_DIR}/run_go2_xt16_recorded_route_dry_run.sh --stop"
}

case "${mode}" in
  prepare-only)
    validate_bool REGENERATE_ROUTE "${REGENERATE_ROUTE_VALUE}"
    prepare_route
    ;;
  status)
    show_status
    ;;
  enable)
    enable_dry_run
    ;;
  disable)
    disable_dry_run
    ;;
  stop)
    stop_all
    ;;
  start)
    start_dry_run
    ;;
esac
