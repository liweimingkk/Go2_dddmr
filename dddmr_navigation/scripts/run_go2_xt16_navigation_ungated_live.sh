#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${DDDMR_DOCKER_USE_SUDO:-0}" == "1" && "${EUID}" -ne 0 ]]; then
  exec sudo -E -- "$0" "$@"
fi

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_navigation_ungated_live.sh [--quick|--full] --check
  scripts/run_go2_xt16_navigation_ungated_live.sh [--quick|--full] --live
  scripts/run_go2_xt16_navigation_ungated_live.sh --stop

Starts a supervised Go2 XT16 navigation session without the localization,
perception, or planner-decision motion gates:

  /dddmr_go2/dry_run_cmd_vel -> Go2 Sport adapter -> /api/sport/request

The command clamp, stale-command StopMove, shutdown StopMove, MAX_Y=0 lateral
lockout, live confirmation, and bounded RUN_SECONDS remain active.

Required for --live:
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV
  MAX_Y=0

Common environment:
  DDDMR_PLATFORM=x64|orin-jp5
  MAP=/root/dddmr_bags/<pose_graph_map_directory>
  DDDMR_DOCKER_USE_SUDO=0|1
  STOP_EXISTING=false|true
  GO2_NET_IFACE=<primary robot interface>
  GO2_DDS_EXTRA_IFACES=<optional additional interfaces>
  RVIZ=false|true
  PUBLISH_STATIC_TF=true       Required by this launcher.
  MAX_X=0.50                  Maximum 0.50 m/s.
  MAX_Y=0                     Lateral motion is always disabled.
  MAX_YAW=0.50                Maximum 0.50 rad/s.
  RUN_SECONDS=300             Range 1..1800 seconds.
  ODOM_TIME_OFFSET_SEC=<optional explicit finite offset>

--quick uses a 3-sample read-only XT16 observation; --full uses 5 samples.
Read-only sensor/ROS checks are diagnostic and do not arm or block Sport output.
Invalid configuration, map data, confirmation, or speed limits still fail.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

warn() {
  printf 'WARNING: %s\n' "$*" >&2
}

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

is_number() {
  [[ "$1" =~ ^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$ ]]
}

is_nonnegative_number() {
  is_number "$1" && awk -v value="$1" 'BEGIN { exit !(value >= 0.0) }'
}

profile="quick"
case "${1:-}" in
  --quick)
    shift
    ;;
  --full)
    profile="full"
    shift
    ;;
esac

mode="${1:-}"
(( $# == 1 )) || {
  usage >&2
  exit 2
}
case "${mode}" in
  --check|--live|--stop)
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
DOCKER_WRAPPER="${SCRIPT_DIR}/dddmr_docker_go2_xt16.sh"
MAP_CONTRACT_TOOL="${SCRIPT_DIR}/go2_pose_graph_map_contract.py"
NAV_CONFIG_SOURCE="${GO2_NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"

PLATFORM_VALUE="${DDDMR_PLATFORM:-x64}"
case "${PLATFORM_VALUE}" in
  x64)
    DEFAULT_IMAGE="dddmr_go2_xt16:x64"
    DEFAULT_ROS_DISTRO="humble"
    DEFAULT_INSTALL_BASE=".docker_go2_xt16_install"
    DEFAULT_GO2_NET_IFACE="enp46s0"
    DEFAULT_GO2_DDS_RCVBUF_MIN="16MiB"
    DEFAULT_DOCKER_RUNTIME="none"
    ;;
  orin-jp5)
    DEFAULT_IMAGE="dddmr_go2_xt16:orin-jp5.1.1"
    DEFAULT_ROS_DISTRO="foxy"
    DEFAULT_INSTALL_BASE=".docker_go2_xt16_orin_install"
    DEFAULT_GO2_NET_IFACE="eth0"
    DEFAULT_GO2_DDS_RCVBUF_MIN="default"
    DEFAULT_DOCKER_RUNTIME="nvidia"
    ;;
  *)
    die "DDDMR_PLATFORM must be x64 or orin-jp5, got ${PLATFORM_VALUE}."
    ;;
esac

IMAGE="${DDDMR_IMAGE:-${DEFAULT_IMAGE}}"
ROS_DISTRO_VALUE="${DDDMR_ROS_DISTRO:-${DEFAULT_ROS_DISTRO}}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-${DEFAULT_INSTALL_BASE}}"
DOCKER_RUNTIME_VALUE="${DDDMR_DOCKER_RUNTIME:-${DEFAULT_DOCKER_RUNTIME}}"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-${DEFAULT_GO2_NET_IFACE}}"
GO2_DDS_EXTRA_IFACES_VALUE="${GO2_DDS_EXTRA_IFACES:-}"
GO2_DDS_PRIMARY_ADDRESS_VALUE="${GO2_DDS_PRIMARY_ADDRESS:-}"
GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE="${GO2_DDS_ROBOT_TOPIC_PATTERNS:-}"
GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX:-120}"
GO2_DDS_RCVBUF_MIN_VALUE="${GO2_DDS_RCVBUF_MIN:-${DEFAULT_GO2_DDS_RCVBUF_MIN}}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
RVIZ_VALUE="${RVIZ:-false}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
STOP_EXISTING_VALUE="${STOP_EXISTING:-false}"
MAX_X_VALUE="${MAX_X:-0.50}"
MAX_Y_VALUE="${MAX_Y:-0}"
MAX_YAW_VALUE="${MAX_YAW:-0.50}"
RUN_SECONDS_VALUE="${RUN_SECONDS:-300}"
MAP_REQUESTED="${MAP:-}"
LIVE_CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_NAV"
CONTAINER_NAME="${NAV_CONTAINER_NAME:-go2_xt16_nav_ungated_live}"
RUNTIME_NAV_CONFIG_HOST=""
RUNTIME_NAV_CONFIG_CONTAINER=""
runtime_active="false"

case "${DOCKER_RUNTIME_VALUE}" in
  none|nvidia)
    ;;
  *)
    die "DDDMR_DOCKER_RUNTIME must be none or nvidia."
    ;;
esac
[[ "${ROS_DISTRO_VALUE}" =~ ^[a-z0-9_]+$ ]] || \
  die "DDDMR_ROS_DISTRO contains unsupported characters."
[[ "${INSTALL_BASE_VALUE}" =~ ^[A-Za-z0-9._/-]+$ ]] || \
  die "DDDMR_INSTALL_BASE contains unsupported characters."
[[ "${CONTAINER_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || \
  die "NAV_CONTAINER_NAME contains unsupported characters."
[[ "${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}" =~ ^[1-9][0-9]*$ ]] || \
  die "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX must be a positive integer."
[[ "${RVIZ_VALUE}" == "true" || "${RVIZ_VALUE}" == "false" ]] || \
  die "RVIZ must be true or false."
[[ "${PUBLISH_STATIC_TF_VALUE}" == "true" ]] || \
  die "Ungated live navigation requires PUBLISH_STATIC_TF=true."
[[ "${STOP_EXISTING_VALUE}" == "true" || "${STOP_EXISTING_VALUE}" == "false" ]] || \
  die "STOP_EXISTING must be true or false."
if [[ "${mode}" != "--stop" ]]; then
  [[ -n "${MAX_Y+x}" ]] || \
    die "Ungated live navigation requires explicit MAX_Y=0."
  is_nonnegative_number "${MAX_X_VALUE}" || die "MAX_X must be finite and nonnegative."
  awk -v value="${MAX_X_VALUE}" 'BEGIN { exit !(value <= 0.50) }' || \
    die "MAX_X must not exceed 0.50m/s."
  is_nonnegative_number "${MAX_Y_VALUE}" || die "MAX_Y must be finite and nonnegative."
  awk -v value="${MAX_Y_VALUE}" 'BEGIN { exit !(value == 0.0) }' || \
    die "Ungated live navigation keeps lateral motion locked: MAX_Y must be 0."
  is_nonnegative_number "${MAX_YAW_VALUE}" || die "MAX_YAW must be finite and nonnegative."
  awk -v value="${MAX_YAW_VALUE}" 'BEGIN { exit !(value <= 0.50) }' || \
    die "MAX_YAW must not exceed 0.50rad/s."
  [[ "${RUN_SECONDS_VALUE}" =~ ^[1-9][0-9]*$ ]] && \
    (( RUN_SECONDS_VALUE <= 1800 )) || \
    die "RUN_SECONDS must be an integer from 1 through 1800."
fi

docker_cmd() {
  docker "$@"
}

nav_container_names() {
  {
    docker_cmd ps -a \
      --filter 'label=dddmr.go2_xt16_navigation=true' \
      --format '{{.Names}}'
    docker_cmd ps -a --format '{{.Names}}' | \
      awk '/^go2_xt16_nav/ || /^go2_xt16_navigation/ {print}'
  } | sort -u
}

stop_nav_containers() {
  local names name
  names="$(nav_container_names)"
  if [[ -z "${names}" ]]; then
    log "No Go2 XT16 navigation containers found."
    return 0
  fi
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if docker_cmd inspect -f '{{.State.Running}}' "${name}" 2>/dev/null | \
       grep -Fxq true; then
      log "Stopping Sport adapter in ${name}, if present."
      docker_cmd exec "${name}" bash -lc \
        "pkill -TERM -f '[g]o2_sport_cmd_vel_adapter.py' || true" || true
      sleep 1
      log "Stopping navigation container ${name}."
      docker_cmd stop -t 8 "${name}" >/dev/null || true
    fi
    docker_cmd rm "${name}" >/dev/null 2>&1 || true
  done <<<"${names}"
}

cleanup() {
  local status=$?
  if [[ "${runtime_active}" == "true" ]]; then
    warn "Stopping ungated navigation during exit cleanup."
    stop_nav_containers || true
    runtime_active="false"
  fi
  if [[ -n "${RUNTIME_NAV_CONFIG_HOST}" ]]; then
    rm -f -- "${RUNTIME_NAV_CONFIG_HOST}"
  fi
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${mode}" == "--stop" ]]; then
  docker_cmd version --format '{{.Server.Version}}' >/dev/null || \
    die "Cannot access the Docker daemon."
  stop_nav_containers
  exit 0
fi

resolve_map_path() {
  local requested="$1"
  local candidate relative
  if [[ "${requested}" == /root/dddmr_bags/* ]]; then
    relative="${requested#/root/dddmr_bags/}"
    candidate="${BAGS_DIR}/${relative}"
  elif [[ "${requested}" == /* ]]; then
    candidate="${requested}"
  else
    requested="${requested#./}"
    requested="${requested#bags/}"
    candidate="${BAGS_DIR}/${requested}"
  fi
  candidate="$(realpath -e -- "${candidate}" 2>/dev/null)" || \
    die "MAP directory does not exist: ${requested}"
  [[ -d "${candidate}" ]] || die "MAP is not a directory: ${candidate}"
  [[ "${candidate}" == "${BAGS_DIR}"/* ]] || \
    die "MAP must stay below DDDMR_BAGS_DIR (${BAGS_DIR})."
  printf '%s\n' "${candidate}"
}

prepare_map_override() {
  local map_host map_relative map_container report key_frame_count
  [[ -n "${MAP_REQUESTED}" ]] || return 0
  [[ -f "${NAV_CONFIG_SOURCE}" ]] || \
    die "Navigation config does not exist: ${NAV_CONFIG_SOURCE}"
  [[ -f "${MAP_CONTRACT_TOOL}" ]] || \
    die "Map contract helper does not exist: ${MAP_CONTRACT_TOOL}"
  map_host="$(resolve_map_path "${MAP_REQUESTED}")"
  map_relative="${map_host#"${BAGS_DIR}"/}"
  [[ "${map_relative}" =~ ^[A-Za-z0-9._/-]+$ ]] || \
    die "MAP contains unsupported characters."
  map_container="/root/dddmr_bags/${map_relative}"
  report="$(python3 "${MAP_CONTRACT_TOOL}" inspect --map-dir "${map_host}")" || \
    die "Selected MAP failed its immutable pose-graph contract."
  key_frame_count="$(awk -F= '$1 == "KEY_FRAME_COUNT" {print $2; exit}' <<<"${report}")"
  [[ "${key_frame_count}" =~ ^[1-9][0-9]*$ ]] || \
    die "MAP contract did not report a positive key-frame count."
  RUNTIME_NAV_CONFIG_HOST="$(mktemp /tmp/go2_xt16_ungated_nav.XXXXXX.yaml)"
  RUNTIME_NAV_CONFIG_CONTAINER="${RUNTIME_NAV_CONFIG_HOST}"
  python3 "${MAP_CONTRACT_TOOL}" render-config \
    --source "${NAV_CONFIG_SOURCE}" \
    --destination "${RUNTIME_NAV_CONFIG_HOST}" \
    --pose-graph-dir "${map_container}" \
    --expected-key-frame-count "${key_frame_count}"
  chmod 0644 "${RUNTIME_NAV_CONFIG_HOST}"
  log "MAP validated: ${map_host} (${key_frame_count} key frames)."
}

BAGS_DIR="$(realpath -m -- "${BAGS_DIR}")"
prepare_map_override

if [[ "${mode}" == "--check" ]]; then
  log "Ungated launch configuration is valid."
  printf 'PROFILE=%s\n' "${profile}"
  printf 'PLATFORM=%s\n' "${PLATFORM_VALUE}"
  printf 'IMAGE=%s\n' "${IMAGE}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION_VALUE}"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID_VALUE}"
  printf 'GO2_NET_IFACE=%s\n' "${GO2_NET_IFACE_VALUE}"
  printf 'GO2_DDS_EXTRA_IFACES=%s\n' "${GO2_DDS_EXTRA_IFACES_VALUE}"
  printf 'MAX_X=%s\nMAX_Y=%s\nMAX_YAW=%s\n' \
    "${MAX_X_VALUE}" "${MAX_Y_VALUE}" "${MAX_YAW_VALUE}"
  printf 'RUN_SECONDS=%s\n' "${RUN_SECONDS_VALUE}"
  printf 'COMMAND_PATH=/dddmr_go2/dry_run_cmd_vel->/api/sport/request\n'
  printf 'GO2_NAV_CMD_GATE=DISABLED\n'
  printf 'SPORT_DECISION_GATE=DISABLED\n'
  exit 0
fi

[[ "${GO2_NAV_LIVE_CONFIRM:-}" == "${LIVE_CONFIRM_PHRASE}" ]] || \
  die "--live requires GO2_NAV_LIVE_CONFIRM=${LIVE_CONFIRM_PHRASE}."
[[ -x "${DOCKER_WRAPPER}" ]] || die "Docker wrapper is not executable."
docker_cmd version --format '{{.Server.Version}}' >/dev/null || \
  die "Cannot access the Docker daemon."
docker_cmd image inspect "${IMAGE}" >/dev/null 2>&1 || \
  die "Docker image ${IMAGE} does not exist."

if [[ "${STOP_EXISTING_VALUE}" == "true" ]]; then
  stop_nav_containers
elif [[ -n "$(nav_container_names)" ]]; then
  nav_container_names >&2
  die "A Go2 XT16 navigation container already exists; use STOP_EXISTING=true."
fi

host_processes="$(
  pgrep -af \
    'go2_sport_cmd_vel_adapter|go2_xt16_navigation[.]launch|p2p_move_base_node' || true
)"
if [[ -n "${host_processes}" ]]; then
  printf '%s\n' "${host_processes}" >&2
  die "A host navigation or Sport adapter process is already running."
fi

mkdir -p "${BAGS_DIR}"
docker_base_args=(
  --rm
  --privileged
  --network=host
  --env "DISPLAY=${DISPLAY:-:0}"
  --env "QT_X11_NO_MITSHM=1"
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
  --env "GO2_DDS_IP=${GO2_DDS_IP_VALUE}"
  --env "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}"
  --env "GO2_DDS_EXTRA_IFACES=${GO2_DDS_EXTRA_IFACES_VALUE}"
  --env "GO2_DDS_PRIMARY_ADDRESS=${GO2_DDS_PRIMARY_ADDRESS_VALUE}"
  --env "GO2_DDS_ROBOT_TOPIC_PATTERNS=${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}"
  --env "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX=${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}"
  --env "GO2_DDS_RCVBUF_MIN=${GO2_DDS_RCVBUF_MIN_VALUE}"
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}"
  --env "DDDMR_INSTALL_BASE=${INSTALL_BASE_VALUE}"
  --volume "/tmp:/tmp"
  --volume "/dev:/dev"
  --volume "${WS_ROOT}:/root/dddmr_navigation"
  --volume "${BAGS_DIR}:/root/dddmr_bags"
  --volume "${BAGS_DIR}:${BAGS_DIR}"
)
if [[ "${DOCKER_RUNTIME_VALUE}" == "nvidia" ]]; then
  docker_base_args+=(
    --runtime nvidia
    --env "NVIDIA_VISIBLE_DEVICES=all"
    --env "NVIDIA_DRIVER_CAPABILITIES=all"
  )
fi

sample_count=3
[[ "${profile}" == "full" ]] && sample_count=5
log "Running the required read-only XT16 observation (${sample_count} samples)."
if ! DDDMR_PLATFORM="${PLATFORM_VALUE}" \
  DDDMR_IMAGE="${IMAGE}" \
  DDDMR_ROS_DISTRO="${ROS_DISTRO_VALUE}" \
  DDDMR_DOCKER_RUNTIME="${DOCKER_RUNTIME_VALUE}" \
  DDDMR_DOCKER_USE_SUDO=0 \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  GO2_DDS_IP="${GO2_DDS_IP_VALUE}" \
  GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
  GO2_DDS_EXTRA_IFACES="${GO2_DDS_EXTRA_IFACES_VALUE}" \
  GO2_DDS_PRIMARY_ADDRESS="${GO2_DDS_PRIMARY_ADDRESS_VALUE}" \
  GO2_DDS_ROBOT_TOPIC_PATTERNS="${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}" \
  GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}" \
  GO2_DDS_RCVBUF_MIN="${GO2_DDS_RCVBUF_MIN_VALUE}" \
    "${DOCKER_WRAPPER}" preflight --samples "${sample_count}" --timeout 12; then
  warn "XT16 read-only observation failed; continuing because this is the explicitly ungated launcher."
fi

log "Capturing the read-only ROS/DDS, odometry, and pre-launch TF snapshot."
graph_snapshot_command='set -eo pipefail
source "/opt/ros/'"${ROS_DISTRO_VALUE}"'/setup.bash"
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
printf "RMW=%s DOMAIN=%s PRIMARY_IFACE=%s EXTRA_IFACES=%s\n" \
  "${RMW_IMPLEMENTATION}" "${ROS_DOMAIN_ID:-0}" \
  "${GO2_NET_IFACE}" "${GO2_DDS_EXTRA_IFACES}"
if timeout 8 ros2 topic info /utlidar/robot_odom; then
  echo "ODOM_GRAPH_STATUS=PRESENT"
else
  echo "ODOM_GRAPH_STATUS=ABSENT"
fi
if timeout 8 ros2 topic info /tf; then
  echo "TF_GRAPH_STATUS=PRESENT"
else
  echo "TF_GRAPH_STATUS=ABSENT_BEFORE_LAUNCH"
fi
if timeout 8 ros2 topic info /tf_static; then
  echo "TF_STATIC_GRAPH_STATUS=PRESENT"
else
  echo "TF_STATIC_GRAPH_STATUS=ABSENT_BEFORE_LAUNCH_STATIC_PUBLISHERS_ENABLED"
fi'
if ! docker_cmd run "${docker_base_args[@]}" "${IMAGE}" \
  bash -lc "${graph_snapshot_command}"; then
  warn "ROS/DDS graph snapshot was incomplete; continuing because this is the explicitly ungated launcher."
fi

ODOM_TIME_OFFSET_SEC_VALUE="${ODOM_TIME_OFFSET_SEC:-}"
if [[ -z "${ODOM_TIME_OFFSET_SEC_VALUE}" ]]; then
  log "Attempting a read-only odom/XT16 time-offset measurement."
  if ODOM_TIME_OFFSET_SEC_VALUE="$(
    DDDMR_PLATFORM="${PLATFORM_VALUE}" \
    DDDMR_IMAGE="${IMAGE}" \
    DDDMR_ROS_DISTRO="${ROS_DISTRO_VALUE}" \
    DDDMR_DOCKER_RUNTIME="${DOCKER_RUNTIME_VALUE}" \
    DDDMR_DOCKER_USE_SUDO=0 \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
    GO2_DDS_IP="${GO2_DDS_IP_VALUE}" \
    GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    GO2_DDS_EXTRA_IFACES="${GO2_DDS_EXTRA_IFACES_VALUE}" \
    GO2_DDS_PRIMARY_ADDRESS="${GO2_DDS_PRIMARY_ADDRESS_VALUE}" \
    GO2_DDS_ROBOT_TOPIC_PATTERNS="${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}" \
    GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}" \
    GO2_DDS_RCVBUF_MIN="${GO2_DDS_RCVBUF_MIN_VALUE}" \
      "${SCRIPT_DIR}/resolve_go2_odom_time_offset.sh"
  )"; then
    log "Measured odom/XT16 time offset: ${ODOM_TIME_OFFSET_SEC_VALUE}s."
  else
    ODOM_TIME_OFFSET_SEC_VALUE="0.0"
    warn "Odom/XT16 measurement failed; continuing ungated with fallback offset 0.0s."
  fi
fi
is_number "${ODOM_TIME_OFFSET_SEC_VALUE}" || \
  die "ODOM_TIME_OFFSET_SEC must be finite."

log "UNGATED LIVE MODE: localization/perception command gate is not started."
log "UNGATED LIVE MODE: Sport decision gate is bypassed."
log "Limits remain max_x=${MAX_X_VALUE}, max_y=0, max_yaw=${MAX_YAW_VALUE}; timeout=${RUN_SECONDS_VALUE}s."
log "Do not send a goal until the map, localization, TF, and local perception look correct."

docker_args=(
  "${docker_base_args[@]}"
  --name "${CONTAINER_NAME}"
  --label "dddmr.go2_xt16_navigation=true"
  --label "dddmr.go2_xt16_ungated=true"
)

launch_args=(
  "rviz:=${RVIZ_VALUE}"
  "publish_static_tf:=true"
  "odom_sync_enabled:=true"
  "odom_sync_tolerance_sec:=${ODOM_SYNC_TOLERANCE_SEC:-0.05}"
  "odom_sync_wait_timeout_sec:=${ODOM_SYNC_WAIT_TIMEOUT_SEC:-0.1}"
  "odom_time_offset_sec:=${ODOM_TIME_OFFSET_SEC_VALUE}"
  "omni_min_vel_y:=0.0"
  "omni_max_vel_y:=0.0"
  "start_go2_nav_cmd_gate:=false"
  "sport_cmd_vel_topic:=/dddmr_go2/dry_run_cmd_vel"
  "start_sport_dry_run_adapter:=false"
  "start_go2_sport_adapter:=true"
  "go2_sport_enable_output:=true"
  "go2_sport_allow_real_request_topic:=true"
  "go2_sport_request_topic:=/api/sport/request"
  "go2_sport_axis_mode:=standard"
  "go2_sport_max_x:=${MAX_X_VALUE}"
  "go2_sport_max_y:=0.0"
  "go2_sport_max_yaw:=${MAX_YAW_VALUE}"
  "go2_sport_cmd_timeout_sec:=0.20"
  "go2_sport_stop_keepalive_hz:=2.0"
  "go2_sport_enable_yaw_arc_shim:=false"
  "go2_sport_decision_topic:=/dddmr_go2/ungated_unused_decision"
)
if [[ -n "${RUNTIME_NAV_CONFIG_CONTAINER}" ]]; then
  launch_args+=(
    "config_file:=${RUNTIME_NAV_CONFIG_CONTAINER}"
    "map_config_file:=${RUNTIME_NAV_CONFIG_CONTAINER}"
  )
fi

container_command='set -eo pipefail
set +u
source "/opt/ros/'"${ROS_DISTRO_VALUE}"'/setup.bash"
if [[ -f /opt/unitree_ros2/setup.bash ]]; then
  source /opt/unitree_ros2/setup.bash
fi
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source "/root/dddmr_navigation/${DDDMR_INSTALL_BASE}/setup.bash"
set -u
cd /root/dddmr_navigation
duration="$1"
shift
exec timeout -s TERM -k 8s "${duration}s" \
  ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch "$@"'

runtime_active="true"
set +e
docker_cmd run "${docker_args[@]}" "${IMAGE}" \
  bash -lc "${container_command}" bash "${RUN_SECONDS_VALUE}" "${launch_args[@]}"
status=$?
set -e
runtime_active="false"

case "${status}" in
  0|124)
    log "Ungated live session ended; the adapter shutdown path issued StopMove."
    ;;
  130|143)
    warn "Ungated live session was interrupted."
    exit "${status}"
    ;;
  *)
    die "Ungated live session exited with status ${status}."
    ;;
esac
