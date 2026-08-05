#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_navigation_test.sh [--quick|--full] --dry-run
  scripts/run_go2_xt16_navigation_test.sh [--quick|--full] --live
  scripts/run_go2_xt16_navigation_test.sh [--quick|--full] --record MISSION.json \
    --initial-pose INITIAL_POSE.json
  scripts/run_go2_xt16_navigation_test.sh [--quick|--full] --multi-dry-run MISSION.json
  scripts/run_go2_xt16_navigation_test.sh [--quick|--full] --multi-live MISSION.json
  scripts/run_go2_xt16_navigation_test.sh --stop

Starts Go2 XT16 navigation with the map configured in:
  src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
Set MAP for a one-run map override without editing that file.

Modes:
  --quick    Short fail-closed startup profile. Keeps all motion-safety gates,
             but uses shorter sustained-data windows and omits the slow status
             inventory. Use after a full run has already validated the setup.
  --full     Full diagnostic startup profile (default). Uses longer sustained
             windows and prints the complete runtime status before arming.
  --dry-run  Start navigation and RViz only. No real /api/sport/request output.
             This is the default.
  --live     Start navigation, RViz, and the live Sport adapter. This can move
             the Go2 after an RViz goal is sent. Requires onsite supervision
             and GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV.
  --record   Start the original DDDMR/P2P stack without real Sport output.
             Interactively save a fixed initial pose and localized waypoints.
  --multi-dry-run
             Load a mission and execute its waypoints in order through the
             dry-run command path.
  --multi-live
             Load a mission and execute it through the supervised Sport
             adapter. Also requires `EXECUTE <mission_id>` after READY.
  --stop     Stop running go2_xt16 navigation containers. If a live adapter is
             running, it is stopped first so it can publish StopMove. When the
             Docker socket is not writable, interactive stop auto-uses sudo.

Common environment overrides:
  DDDMR_PLATFORM=x64|orin-jp5
  MAX_X=0.50
  MAX_Y=0|0.20           Required explicitly for live; dry-run defaults to 0.20.
                         Use 0.20 only after a supervised lateral-direction probe.
  MAX_YAW=0.50
  RVIZ=true
  PUBLISH_STATIC_TF=true
  STOP_EXISTING=false      Set true to stop old nav containers before starting.
  RUN_SECONDS=             Live defaults to 300; maximum is 1800 seconds.
  GO2_NET_IFACE=enp46s0
  GO2_DDS_IP=192.168.123.18
  GO2_DDS_EXTRA_IFACES=wlan0 Additional DDS interface(s), for example Orin Wi-Fi.
  GO2_DDS_PRIMARY_ADDRESS=... Optional primary-interface IPv4 override.
  GO2_DDS_ROBOT_TOPIC_PATTERNS=...
                             DDS topics pinned to the primary interface.
  GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX=120
                             DDS participant capacity for multi-process navigation.
  DDDMR_DOCKER_USE_SUDO=1   Run Docker through sudo (required by the Orin user).
  ODOM_TIME_OFFSET_SEC=...  Explicit override; skips automatic measurement.
  AUTO_MEASURE_ODOM_TIME_OFFSET=true
  ODOM_SYNC_TOLERANCE_SEC=0.05
  ODOM_SYNC_WAIT_TIMEOUT_SEC=0.1
  GO2_NAV_ODOM_SYNC_WINDOW_SEC=8.0
  GO2_NAV_ODOM_SYNC_GATE_ATTEMPTS=3
  GO2_NAV_FEATURE_WINDOW_SEC=8.0
  GO2_NAV_FEATURE_GATE_ATTEMPTS=3
  LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC=0.35
  GO2_NAV_OBSERVATION_WINDOW_SEC=20.0
  GO2_NAV_OBSERVATION_GATE_ATTEMPTS=3
  GO2_NAV_OBSERVATION_STARTUP_AGE_MARGIN_SEC=0.05
  DDDMR_BAGS_DIR=../bags
  MAP=...                    Fast map override. Accepts a /root/dddmr_bags/...
                             container path, a host path below DDDMR_BAGS_DIR,
                             or a directory name below DDDMR_BAGS_DIR.
  NAV_CONTAINER_NAME=...

Examples:
  scripts/run_go2_xt16_navigation_test.sh --dry-run
  scripts/run_go2_xt16_navigation_test.sh --quick --dry-run
  MAP=/root/dddmr_bags/go2_xt16_mouth_mapping_20260723_153831_cleaned_20260724_130438 \
    scripts/run_go2_xt16_navigation_test.sh --quick --dry-run
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV MAX_Y=0.0 \
    scripts/run_go2_xt16_navigation_test.sh --live
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV MAX_Y=0.20 \
    scripts/run_go2_xt16_navigation_test.sh --live
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV STOP_EXISTING=true MAX_Y=0.0 \
    scripts/run_go2_xt16_navigation_test.sh --live
  scripts/run_go2_xt16_navigation_test.sh --record \
    bags/p2p_missions/route_a.json --initial-pose \
    bags/p2p_missions/go2_start.json
  scripts/run_go2_xt16_navigation_test.sh --multi-dry-run \
    bags/p2p_missions/route_a.json
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV MAX_Y=0.20 \
    scripts/run_go2_xt16_navigation_test.sh --quick --multi-live \
    bags/p2p_missions/route_a.json
  scripts/run_go2_xt16_navigation_test.sh --stop
EOF
}

STARTUP_PROFILE_VALUE="${GO2_NAV_STARTUP_PROFILE:-full}"
case "${1:-}" in
  --quick)
    STARTUP_PROFILE_VALUE="quick"
    shift
    ;;
  --full)
    STARTUP_PROFILE_VALUE="full"
    shift
    ;;
esac
[[ "${STARTUP_PROFILE_VALUE}" == "quick" || "${STARTUP_PROFILE_VALUE}" == "full" ]] || {
  echo "GO2_NAV_STARTUP_PROFILE must be quick or full, got: ${STARTUP_PROFILE_VALUE}" >&2
  exit 2
}

mode="dry-run"
mission_file_requested=""
initial_pose_file_requested=""
case "${1:---dry-run}" in
  --dry-run)
    (( $# <= 1 )) || {
      usage >&2
      exit 2
    }
    mode="dry-run"
    ;;
  --live)
    (( $# == 1 )) || {
      usage >&2
      exit 2
    }
    mode="live"
    ;;
  --multi-dry-run|--multi-live)
    (( $# == 2 )) || {
      usage >&2
      exit 2
    }
    mode="${1#--}"
    mission_file_requested="$2"
    ;;
  --record)
    (( $# == 4 )) && [[ "$3" == "--initial-pose" ]] || {
      usage >&2
      exit 2
    }
    mode="record"
    mission_file_requested="$2"
    initial_pose_file_requested="$4"
    ;;
  --stop)
    (( $# == 1 )) || {
      usage >&2
      exit 2
    }
    mode="stop"
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown or incomplete arguments: $*" >&2
    usage >&2
    exit 2
    ;;
esac

live_mode="false"
multi_mode="false"
record_mode="false"
[[ "${mode}" == "live" || "${mode}" == "multi-live" ]] && live_mode="true"
[[ "${mode}" == "multi-live" || "${mode}" == "multi-dry-run" ]] && multi_mode="true"
[[ "${mode}" == "record" ]] && record_mode="true"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"

PLATFORM_VALUE="${DDDMR_PLATFORM:-x64}"
case "${PLATFORM_VALUE}" in
  x64)
    DEFAULT_IMAGE="dddmr_go2_xt16:x64"
    DEFAULT_ROS_DISTRO="humble"
    DEFAULT_GO2_NET_IFACE="enp46s0"
    DEFAULT_BUILD_BASE=".docker_go2_xt16_build"
    DEFAULT_INSTALL_BASE=".docker_go2_xt16_install"
    DEFAULT_LOG_BASE=".docker_go2_xt16_log"
    DEFAULT_GO2_DDS_RCVBUF_MIN="16MiB"
    ;;
  orin-jp5)
    DEFAULT_IMAGE="dddmr_go2_xt16:orin-jp5.1.1"
    DEFAULT_ROS_DISTRO="foxy"
    DEFAULT_GO2_NET_IFACE="eth0"
    DEFAULT_BUILD_BASE=".docker_go2_xt16_orin_build"
    DEFAULT_INSTALL_BASE=".docker_go2_xt16_orin_install"
    DEFAULT_LOG_BASE=".docker_go2_xt16_orin_log"
    DEFAULT_GO2_DDS_RCVBUF_MIN="default"
    ;;
  *)
    echo "DDDMR_PLATFORM must be x64 or orin-jp5, got: ${PLATFORM_VALUE}" >&2
    exit 2
    ;;
esac

IMAGE="${DDDMR_IMAGE:-${DEFAULT_IMAGE}}"
ROS_DISTRO_VALUE="${DDDMR_ROS_DISTRO:-${DEFAULT_ROS_DISTRO}}"
[[ "${ROS_DISTRO_VALUE}" =~ ^[a-z0-9_]+$ ]] || {
  echo "DDDMR_ROS_DISTRO contains unsupported characters." >&2
  exit 2
}
ROS_SETUP_FILE_VALUE="/opt/ros/${ROS_DISTRO_VALUE}/setup.bash"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
MAP_REQUESTED="${MAP:-}"
NAV_CONFIG_SOURCE="${GO2_NAV_CONFIG_FILE:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-${DEFAULT_GO2_NET_IFACE}}"
GO2_DDS_EXTRA_IFACES_VALUE="${GO2_DDS_EXTRA_IFACES:-}"
GO2_DDS_PRIMARY_ADDRESS_VALUE="${GO2_DDS_PRIMARY_ADDRESS:-}"
GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE="${GO2_DDS_ROBOT_TOPIC_PATTERNS:-}"
GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX:-120}"
GO2_DDS_RCVBUF_MIN_VALUE="${GO2_DDS_RCVBUF_MIN:-${DEFAULT_GO2_DDS_RCVBUF_MIN}}"
[[ "${GO2_DDS_RCVBUF_MIN_VALUE}" == "default" || \
   "${GO2_DDS_RCVBUF_MIN_VALUE}" =~ ^[1-9][0-9]*(B|KiB|MiB|GiB)$ ]] || {
  echo "GO2_DDS_RCVBUF_MIN must be default or a positive byte size." >&2
  exit 2
}
DDDMR_DOCKER_USE_SUDO_VALUE="${DDDMR_DOCKER_USE_SUDO:-0}"
DOCKER_SOCKET_PATH_VALUE="${DOCKER_SOCKET_PATH:-/var/run/docker.sock}"
ODOM_TIME_OFFSET_SEC_VALUE="${ODOM_TIME_OFFSET_SEC:-}"
ODOM_SYNC_TOLERANCE_SEC_VALUE="${ODOM_SYNC_TOLERANCE_SEC:-0.05}"
ODOM_SYNC_WAIT_TIMEOUT_SEC_VALUE="${ODOM_SYNC_WAIT_TIMEOUT_SEC:-0.1}"
ODOM_OFFSET_RESOLVER="${GO2_ODOM_TIME_OFFSET_RESOLVER:-${SCRIPT_DIR}/resolve_go2_odom_time_offset.sh}"
ODOM_SYNC_CHECKER="${GO2_ODOM_SYNC_CHECKER:-${SCRIPT_DIR}/check_go2_odom_sync.py}"
DDS_BUFFER_CHECK="${SCRIPT_DIR}/check_go2_dds_receive_buffers.sh"
OBSERVATION_GATE_SOURCE="${WS_ROOT}/src/dddmr_beginner_guide/scripts/go2_pointcloud_stream_gate.py"
MISSION_IO_SOURCE="${WS_ROOT}/src/dddmr_route_navigation/scripts/waypoint_mission_io.py"
MAP_CONTRACT_TOOL="${SCRIPT_DIR}/go2_pose_graph_map_contract.py"
CURRENT_OBSERVATION_TOPIC="/perception_3d_local/lidar/current_observation"
MCL_FEATURE_TOPIC="/laser_cloud_less_sharp"
BUILD_BASE_VALUE="${DDDMR_BUILD_BASE:-${DEFAULT_BUILD_BASE}}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-${DEFAULT_INSTALL_BASE}}"
LOG_BASE_VALUE="${DDDMR_LOG_BASE:-${DEFAULT_LOG_BASE}}"
DOCKER_RUN_ARGS=()
if [[ "${PLATFORM_VALUE}" == "orin-jp5" ]]; then
  DOCKER_RUNTIME_VALUE="${DDDMR_DOCKER_RUNTIME:-nvidia}"
else
  DOCKER_RUNTIME_VALUE="${DDDMR_DOCKER_RUNTIME:-none}"
fi
[[ "${DOCKER_RUNTIME_VALUE}" == "nvidia" || "${DOCKER_RUNTIME_VALUE}" == "none" ]] || {
  echo "DDDMR_DOCKER_RUNTIME must be nvidia or none." >&2
  exit 2
}
if [[ "${DOCKER_RUNTIME_VALUE}" == "nvidia" ]]; then
  DOCKER_RUN_ARGS+=(--runtime nvidia)
fi
RVIZ_VALUE="${RVIZ:-true}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
MAX_X_VALUE="${MAX_X:-0.50}"
MAX_YAW_VALUE="${MAX_YAW:-0.50}"
if [[ -n "${MAX_Y+x}" ]]; then
  MAX_Y_VALUE="${MAX_Y}"
elif [[ "${live_mode}" == "true" ]]; then
  echo "Live navigation requires explicit MAX_Y=0 (no sidestep) or MAX_Y=0.20 (bounded lateral avoidance)." >&2
  exit 2
else
  MAX_Y_VALUE="0.20"
fi
OMNI_MIN_Y_VALUE=""
RUN_SECONDS_VALUE="${RUN_SECONDS:-}"
STOP_EXISTING_VALUE="${STOP_EXISTING:-false}"
LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC_VALUE="${LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC:-0.35}"
if [[ "${STARTUP_PROFILE_VALUE}" == "quick" ]]; then
  DEFAULT_ODOM_SYNC_WINDOW_SEC="3.0"
  DEFAULT_ODOM_SYNC_MIN_SAMPLES="20"
  DEFAULT_FEATURE_WINDOW_SEC="3.0"
  DEFAULT_FEATURE_TIMEOUT_SEC="10.0"
  DEFAULT_FEATURE_MIN_SAMPLES="20"
  DEFAULT_OBSERVATION_WINDOW_SEC="5.0"
  DEFAULT_OBSERVATION_TIMEOUT_SEC="12.0"
  DEFAULT_OBSERVATION_MIN_SAMPLES="35"
else
  DEFAULT_ODOM_SYNC_WINDOW_SEC="8.0"
  DEFAULT_ODOM_SYNC_MIN_SAMPLES="50"
  DEFAULT_FEATURE_WINDOW_SEC="8.0"
  DEFAULT_FEATURE_TIMEOUT_SEC="18.0"
  DEFAULT_FEATURE_MIN_SAMPLES="50"
  DEFAULT_OBSERVATION_WINDOW_SEC="20.0"
  DEFAULT_OBSERVATION_TIMEOUT_SEC="30.0"
  DEFAULT_OBSERVATION_MIN_SAMPLES="140"
fi
OBSERVATION_WINDOW_SEC_VALUE="${GO2_NAV_OBSERVATION_WINDOW_SEC:-${DEFAULT_OBSERVATION_WINDOW_SEC}}"
OBSERVATION_TIMEOUT_SEC_VALUE="${GO2_NAV_OBSERVATION_TIMEOUT_SEC:-${DEFAULT_OBSERVATION_TIMEOUT_SEC}}"
OBSERVATION_MIN_SAMPLES_VALUE="${GO2_NAV_OBSERVATION_MIN_SAMPLES:-${DEFAULT_OBSERVATION_MIN_SAMPLES}}"
OBSERVATION_MIN_RATE_HZ_VALUE="${GO2_NAV_OBSERVATION_MIN_RATE_HZ:-7.0}"
OBSERVATION_MAX_HEADER_GAP_SEC_VALUE="${GO2_NAV_OBSERVATION_MAX_HEADER_GAP_SEC:-0.35}"
OBSERVATION_MAX_RECEIVE_GAP_SEC_VALUE="${GO2_NAV_OBSERVATION_MAX_RECEIVE_GAP_SEC:-0.35}"
OBSERVATION_MAX_FUTURE_SKEW_SEC_VALUE="${GO2_NAV_OBSERVATION_MAX_FUTURE_SKEW_SEC:-0.05}"
OBSERVATION_GATE_ATTEMPTS_VALUE="${GO2_NAV_OBSERVATION_GATE_ATTEMPTS:-3}"
OBSERVATION_STARTUP_AGE_MARGIN_SEC_VALUE="${GO2_NAV_OBSERVATION_STARTUP_AGE_MARGIN_SEC:-0.05}"
ODOM_SYNC_WINDOW_SEC_VALUE="${GO2_NAV_ODOM_SYNC_WINDOW_SEC:-${DEFAULT_ODOM_SYNC_WINDOW_SEC}}"
ODOM_SYNC_MIN_SAMPLES_VALUE="${GO2_NAV_ODOM_SYNC_MIN_SAMPLES:-${DEFAULT_ODOM_SYNC_MIN_SAMPLES}}"
ODOM_SYNC_MAX_RECEIVE_GAP_SEC_VALUE="${GO2_NAV_ODOM_SYNC_MAX_RECEIVE_GAP_SEC:-0.30}"
ODOM_SYNC_GATE_ATTEMPTS_VALUE="${GO2_NAV_ODOM_SYNC_GATE_ATTEMPTS:-3}"
FEATURE_WINDOW_SEC_VALUE="${GO2_NAV_FEATURE_WINDOW_SEC:-${DEFAULT_FEATURE_WINDOW_SEC}}"
FEATURE_TIMEOUT_SEC_VALUE="${GO2_NAV_FEATURE_TIMEOUT_SEC:-${DEFAULT_FEATURE_TIMEOUT_SEC}}"
FEATURE_MIN_SAMPLES_VALUE="${GO2_NAV_FEATURE_MIN_SAMPLES:-${DEFAULT_FEATURE_MIN_SAMPLES}}"
FEATURE_MAX_HEADER_GAP_SEC_VALUE="${GO2_NAV_FEATURE_MAX_HEADER_GAP_SEC:-0.75}"
FEATURE_MAX_RECEIVE_GAP_SEC_VALUE="${GO2_NAV_FEATURE_MAX_RECEIVE_GAP_SEC:-0.75}"
FEATURE_MAX_HEADER_AGE_SEC_VALUE="${GO2_NAV_FEATURE_MAX_HEADER_AGE_SEC:-0.85}"
FEATURE_GATE_ATTEMPTS_VALUE="${GO2_NAV_FEATURE_GATE_ATTEMPTS:-3}"
LIVE_CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_NAV"
CONTAINER_NAME="${NAV_CONTAINER_NAME:-go2_xt16_nav_${mode//-/_}_x${MAX_X_VALUE//./}_y${MAX_Y_VALUE//./}_yaw${MAX_YAW_VALUE//./}_$(date +%Y%m%d_%H%M%S)}"
RUN_LOG_DIR="${RUN_LOG_DIR:-${WS_ROOT}/run_logs}"
ADAPTER_LOG_CONTAINER="/root/dddmr_navigation/run_logs/${CONTAINER_NAME}_adapter.log"
ADAPTER_LOG_HOST="${RUN_LOG_DIR}/${CONTAINER_NAME}_adapter.log"
PERCEPTION_GATE_LOG_HOST="${RUN_LOG_DIR}/${CONTAINER_NAME}_perception_gate.log"
FEATURE_GATE_LOG_HOST="${RUN_LOG_DIR}/${CONTAINER_NAME}_feature_gate.log"
LOCAL_LIDAR_RUNTIME_FRESHNESS_SEC_VALUE=""
LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE=""
runtime_started="false"
mission_file=""
mission_file_container=""
initial_pose_file=""
initial_pose_file_container=""
mission_id=""
MAP_HOST_DIR=""
MAP_CONTAINER_DIR=""
MAP_KEY_FRAME_COUNT=""
MAP_RAW_MAP_POINT_COUNT=""
MAP_RAW_GROUND_POINT_COUNT=""
MAP_RECOMMENDED_STATIC_LAYER_TIMEOUT_SEC=""
RUNTIME_NAV_CONFIG_HOST=""
RUNTIME_NAV_CONFIG_CONTAINER="/run/go2_xt16_navigation_runtime.yaml"
docker_sudo_auto_enabled="false"
if [[ -n "${GO2_NAV_STATIC_LAYER_TIMEOUT_SEC+x}" ]]; then
  STATIC_LAYER_TIMEOUT_SEC_VALUE="${GO2_NAV_STATIC_LAYER_TIMEOUT_SEC}"
  STATIC_LAYER_TIMEOUT_EXPLICIT="true"
else
  STATIC_LAYER_TIMEOUT_SEC_VALUE="90"
  STATIC_LAYER_TIMEOUT_EXPLICIT="false"
fi

if [[ "${mode}" == "stop" && -z "${DDDMR_DOCKER_USE_SUDO+x}" && \
      -S "${DOCKER_SOCKET_PATH_VALUE}" && ! -w "${DOCKER_SOCKET_PATH_VALUE}" ]]; then
  DDDMR_DOCKER_USE_SUDO_VALUE="1"
  docker_sudo_auto_enabled="true"
fi

case "${DDDMR_DOCKER_USE_SUDO_VALUE}" in
  0|false)
    ;;
  1|true)
    docker() {
      sudo docker "$@"
    }
    ;;
  *)
    printf 'ERROR: DDDMR_DOCKER_USE_SUDO must be 0, 1, false, or true.\n' >&2
    exit 2
    ;;
esac

if [[ "${live_mode}" == "true" && -z "${RUN_SECONDS_VALUE}" ]]; then
  RUN_SECONDS_VALUE="300"
fi

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

require_docker_access() {
  local output=""
  if ! output="$(docker version --format '{{.Server.Version}}' 2>&1)"; then
    [[ -n "${output}" ]] && printf '%s\n' "${output}" >&2
    die "Cannot access the Docker daemon. Set DDDMR_DOCKER_USE_SUDO=1 or run with Docker socket access."
  fi
}

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

resolve_map_path() {
  local requested="$1"
  local candidate=""
  local relative=""

  BAGS_DIR="$(realpath -m -- "${BAGS_DIR}")"
  if [[ "${requested}" == "/root/dddmr_bags" ]]; then
    die "MAP must name a map directory below /root/dddmr_bags."
  elif [[ "${requested}" == /root/dddmr_bags/* ]]; then
    relative="${requested#/root/dddmr_bags/}"
    candidate="${BAGS_DIR}/${relative}"
  elif [[ "${requested}" == /* ]]; then
    candidate="${requested}"
  else
    requested="${requested#./}"
    requested="${requested#bags/}"
    candidate="${BAGS_DIR}/${requested}"
  fi

  if ! candidate="$(realpath -e -- "${candidate}" 2>/dev/null)"; then
    die "MAP directory does not exist: ${requested}"
  fi
  [[ -d "${candidate}" ]] || die "MAP is not a directory: ${candidate}"
  [[ "${candidate}" == "${BAGS_DIR}"/* ]] || \
    die "MAP must stay below DDDMR_BAGS_DIR (${BAGS_DIR}): ${candidate}"

  relative="${candidate#"${BAGS_DIR}"/}"
  [[ "${relative}" =~ ^[A-Za-z0-9._/-]+$ ]] || \
    die "MAP contains unsupported characters: ${relative}"
  printf '%s\n' "${candidate}"
}

prepare_map_override() {
  local contract_report="" key="" value=""

  [[ -n "${MAP_REQUESTED}" ]] || return 0
  [[ -f "${NAV_CONFIG_SOURCE}" ]] || \
    die "Navigation config does not exist: ${NAV_CONFIG_SOURCE}"
  [[ -f "${MAP_CONTRACT_TOOL}" ]] || \
    die "Pose-graph map contract helper is missing: ${MAP_CONTRACT_TOOL}"

  BAGS_DIR="$(realpath -m -- "${BAGS_DIR}")"
  MAP_HOST_DIR="$(resolve_map_path "${MAP_REQUESTED}")"
  MAP_CONTAINER_DIR="$(container_bags_path "${MAP_HOST_DIR}")"
  if ! contract_report="$(
    python3 "${MAP_CONTRACT_TOOL}" inspect --map-dir "${MAP_HOST_DIR}"
  )"; then
    die "Selected MAP failed its immutable pose-graph contract."
  fi
  while IFS='=' read -r key value; do
    case "${key}" in
      KEY_FRAME_COUNT)
        MAP_KEY_FRAME_COUNT="${value}"
        ;;
      MAP_POINT_COUNT)
        MAP_RAW_MAP_POINT_COUNT="${value}"
        ;;
      GROUND_POINT_COUNT)
        MAP_RAW_GROUND_POINT_COUNT="${value}"
        ;;
      STATIC_LAYER_TIMEOUT_SEC)
        MAP_RECOMMENDED_STATIC_LAYER_TIMEOUT_SEC="${value}"
        ;;
    esac
  done <<<"${contract_report}"
  [[ "${MAP_KEY_FRAME_COUNT}" =~ ^[1-9][0-9]*$ ]] || \
    die "Pose-graph contract did not report a positive key-frame count."
  [[ "${MAP_RAW_MAP_POINT_COUNT}" =~ ^[1-9][0-9]*$ ]] || \
    die "Pose-graph contract did not report a positive map point count."
  [[ "${MAP_RAW_GROUND_POINT_COUNT}" =~ ^[1-9][0-9]*$ ]] || \
    die "Pose-graph contract did not report a positive ground point count."
  [[ "${MAP_RECOMMENDED_STATIC_LAYER_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || \
    die "Pose-graph contract did not report a static-layer timeout."

  mkdir -p -- "${RUN_LOG_DIR}"
  RUNTIME_NAV_CONFIG_HOST="$(
    mktemp --suffix=.yaml \
      "${RUN_LOG_DIR}/${CONTAINER_NAME}_map_override.XXXXXX"
  )"
  python3 "${MAP_CONTRACT_TOOL}" render-config \
    --source "${NAV_CONFIG_SOURCE}" \
    --destination "${RUNTIME_NAV_CONFIG_HOST}" \
    --pose-graph-dir "${MAP_CONTAINER_DIR}" \
    --expected-key-frame-count "${MAP_KEY_FRAME_COUNT}" || \
    die "Failed to pair the navigation config with the selected MAP contract."
  chmod 0644 "${RUNTIME_NAV_CONFIG_HOST}"

  log "Fast map override selected:"
  log "  host: ${MAP_HOST_DIR}"
  log "  container pose_graph_dir: ${MAP_CONTAINER_DIR}"
  log "  key frames: ${MAP_KEY_FRAME_COUNT} (validated feature/surface/ground sets)"
  log "  raw points: map=${MAP_RAW_MAP_POINT_COUNT} ground=${MAP_RAW_GROUND_POINT_COUNT}"
  log "  runtime expected_key_frame_count: ${MAP_KEY_FRAME_COUNT}"
}

configure_static_layer_timeout() {
  if [[ "${STATIC_LAYER_TIMEOUT_EXPLICIT}" == "false" && \
        -n "${MAP_RECOMMENDED_STATIC_LAYER_TIMEOUT_SEC}" ]]; then
    if (( MAP_RECOMMENDED_STATIC_LAYER_TIMEOUT_SEC > \
          STATIC_LAYER_TIMEOUT_SEC_VALUE )); then
      STATIC_LAYER_TIMEOUT_SEC_VALUE="${MAP_RECOMMENDED_STATIC_LAYER_TIMEOUT_SEC}"
      log "Dense MAP detected; raised only the static-layer readiness timeout to ${STATIC_LAYER_TIMEOUT_SEC_VALUE}s."
    fi
  fi

  [[ "${STATIC_LAYER_TIMEOUT_SEC_VALUE}" =~ ^[1-9][0-9]*$ ]] && \
    (( STATIC_LAYER_TIMEOUT_SEC_VALUE >= 30 )) && \
    (( STATIC_LAYER_TIMEOUT_SEC_VALUE <= 1800 )) || \
    die "GO2_NAV_STATIC_LAYER_TIMEOUT_SEC must be an integer from 30 through 1800."
  log "Static-layer readiness timeout: ${STATIC_LAYER_TIMEOUT_SEC_VALUE}s (map geometry unchanged)"
}

weighted_ground_timeout_error() {
  if [[ -n "${MAP_HOST_DIR}" ]]; then
    die "Timed out after ${STATIC_LAYER_TIMEOUT_SEC_VALUE}s waiting for weighted planning-ground publication (MAP key_frames=${MAP_KEY_FRAME_COUNT}, raw_ground_points=${MAP_RAW_GROUND_POINT_COUNT})."
  fi
  die "Timed out after ${STATIC_LAYER_TIMEOUT_SEC_VALUE}s waiting for weighted planning-ground publication."
}

validate_mission_request() {
  [[ -f "${MISSION_IO_SOURCE}" ]] || \
    die "P2P mission validator is missing: ${MISSION_IO_SOURCE}"
  BAGS_DIR="$(realpath -m -- "${BAGS_DIR}")"
  if [[ -n "${mission_file_requested}" ]]; then
    mission_file="$(resolve_bags_path "${mission_file_requested}")"
    mission_file_container="$(container_bags_path "${mission_file}")"
  fi
  if [[ -n "${initial_pose_file_requested}" ]]; then
    initial_pose_file="$(resolve_bags_path "${initial_pose_file_requested}")"
    initial_pose_file_container="$(container_bags_path "${initial_pose_file}")"
  fi
  if [[ "${record_mode}" == "true" ]]; then
    [[ "${mission_file}" != "${initial_pose_file}" ]] || \
      die "mission and initial-pose files must be different"
    mkdir -p -- \
      "$(dirname -- "${mission_file}")" \
      "$(dirname -- "${initial_pose_file}")"
  elif [[ "${multi_mode}" == "true" ]]; then
    [[ -f "${mission_file}" ]] || \
      die "mission file does not exist: ${mission_file}"
    /usr/bin/python3 "${MISSION_IO_SOURCE}" validate \
      --mission "${mission_file}" --root "${BAGS_DIR}"
    mission_id="$(
      /usr/bin/python3 "${MISSION_IO_SOURCE}" mission-id \
        --mission "${mission_file}" --root "${BAGS_DIR}"
    )"
  fi
}

is_number() {
  [[ "$1" =~ ^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$ ]]
}

is_nonnegative_number() {
  is_number "$1" && awk -v value="$1" 'BEGIN { exit !(value >= 0) }'
}

is_positive_number() {
  is_number "$1" && awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

validate_lateral_limit() {
  is_nonnegative_number "${MAX_Y_VALUE}" || \
    die "MAX_Y must be a finite nonnegative number."
  awk -v value="${MAX_Y_VALUE}" \
    'BEGIN { exit !(value == 0.0 || value == 0.20) }' || \
    die "MAX_Y must be exactly 0 (disabled) or 0.20m/s (bounded lateral avoidance)."
  MAX_Y_VALUE="$(
    awk -v value="${MAX_Y_VALUE}" 'BEGIN { printf "%.6f", value }'
  )"
  OMNI_MIN_Y_VALUE="$(
    awk -v value="${MAX_Y_VALUE}" 'BEGIN { printf "%.6f", -value }'
  )"
}

validate_perception_settings() {
  [[ -x "${DDS_BUFFER_CHECK}" ]] || \
    die "Go2 DDS receive-buffer check is not executable: ${DDS_BUFFER_CHECK}"
  [[ -f "${OBSERVATION_GATE_SOURCE}" ]] || \
    die "Local perception stream gate is missing: ${OBSERVATION_GATE_SOURCE}"
  is_positive_number "${LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC_VALUE}" || \
    die "LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC must be a finite positive number."
  awk -v value="${LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC_VALUE}" \
    'BEGIN { exit !(value <= 0.35) }' || \
    die "LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC must not exceed the 0.35s safety cap."
  is_positive_number "${OBSERVATION_WINDOW_SEC_VALUE}" || \
    die "GO2_NAV_OBSERVATION_WINDOW_SEC must be a finite positive number."
  is_positive_number "${OBSERVATION_TIMEOUT_SEC_VALUE}" || \
    die "GO2_NAV_OBSERVATION_TIMEOUT_SEC must be a finite positive number."
  [[ "${OBSERVATION_MIN_SAMPLES_VALUE}" =~ ^[1-9][0-9]*$ ]] && \
    (( OBSERVATION_MIN_SAMPLES_VALUE >= 2 )) || \
    die "GO2_NAV_OBSERVATION_MIN_SAMPLES must be an integer of at least 2."
  [[ "${OBSERVATION_GATE_ATTEMPTS_VALUE}" =~ ^[1-5]$ ]] || \
    die "GO2_NAV_OBSERVATION_GATE_ATTEMPTS must be an integer from 1 through 5."
  is_nonnegative_number "${OBSERVATION_STARTUP_AGE_MARGIN_SEC_VALUE}" || \
    die "GO2_NAV_OBSERVATION_STARTUP_AGE_MARGIN_SEC must be finite and nonnegative."
  awk -v value="${OBSERVATION_STARTUP_AGE_MARGIN_SEC_VALUE}" \
    'BEGIN { exit !(value <= 0.05) }' || \
    die "GO2_NAV_OBSERVATION_STARTUP_AGE_MARGIN_SEC must not exceed 0.05s."
  is_positive_number "${OBSERVATION_MIN_RATE_HZ_VALUE}" || \
    die "GO2_NAV_OBSERVATION_MIN_RATE_HZ must be a finite positive number."
  is_positive_number "${OBSERVATION_MAX_HEADER_GAP_SEC_VALUE}" || \
    die "GO2_NAV_OBSERVATION_MAX_HEADER_GAP_SEC must be a finite positive number."
  is_positive_number "${OBSERVATION_MAX_RECEIVE_GAP_SEC_VALUE}" || \
    die "GO2_NAV_OBSERVATION_MAX_RECEIVE_GAP_SEC must be a finite positive number."
  is_nonnegative_number "${OBSERVATION_MAX_FUTURE_SKEW_SEC_VALUE}" || \
    die "GO2_NAV_OBSERVATION_MAX_FUTURE_SKEW_SEC must be finite and nonnegative."
  awk \
    -v timeout="${OBSERVATION_TIMEOUT_SEC_VALUE}" \
    -v window="${OBSERVATION_WINDOW_SEC_VALUE}" \
    -v gap="${OBSERVATION_MAX_RECEIVE_GAP_SEC_VALUE}" \
    'BEGIN { exit !(timeout > window + gap && timeout <= 45.0) }' || \
    die "Observation timeout must exceed window plus receive gap and be at most 45s."
}

resolve_odom_time_offset() {
  [[ -x "${ODOM_OFFSET_RESOLVER}" ]] || \
    die "Odom time-offset resolver is not executable: ${ODOM_OFFSET_RESOLVER}"
  [[ -f "${ODOM_SYNC_CHECKER}" ]] || \
    die "Odom synchronization checker is missing: ${ODOM_SYNC_CHECKER}"
  is_nonnegative_number "${ODOM_SYNC_TOLERANCE_SEC_VALUE}" || \
    die "ODOM_SYNC_TOLERANCE_SEC must be a finite nonnegative number."
  is_nonnegative_number "${ODOM_SYNC_WAIT_TIMEOUT_SEC_VALUE}" || \
    die "ODOM_SYNC_WAIT_TIMEOUT_SEC must be a finite nonnegative number."
  is_positive_number "${ODOM_SYNC_WINDOW_SEC_VALUE}" || \
    die "GO2_NAV_ODOM_SYNC_WINDOW_SEC must be a finite positive number."
  [[ "${ODOM_SYNC_MIN_SAMPLES_VALUE}" =~ ^[1-9][0-9]*$ ]] && \
    (( ODOM_SYNC_MIN_SAMPLES_VALUE >= 2 )) || \
    die "GO2_NAV_ODOM_SYNC_MIN_SAMPLES must be an integer of at least 2."
  [[ "${ODOM_SYNC_GATE_ATTEMPTS_VALUE}" =~ ^[1-5]$ ]] || \
    die "GO2_NAV_ODOM_SYNC_GATE_ATTEMPTS must be an integer from 1 through 5."
  is_positive_number "${ODOM_SYNC_MAX_RECEIVE_GAP_SEC_VALUE}" || \
    die "GO2_NAV_ODOM_SYNC_MAX_RECEIVE_GAP_SEC must be a finite positive number."
  is_positive_number "${FEATURE_WINDOW_SEC_VALUE}" || \
    die "GO2_NAV_FEATURE_WINDOW_SEC must be a finite positive number."
  is_positive_number "${FEATURE_TIMEOUT_SEC_VALUE}" || \
    die "GO2_NAV_FEATURE_TIMEOUT_SEC must be a finite positive number."
  [[ "${FEATURE_MIN_SAMPLES_VALUE}" =~ ^[1-9][0-9]*$ ]] && \
    (( FEATURE_MIN_SAMPLES_VALUE >= 2 )) || \
    die "GO2_NAV_FEATURE_MIN_SAMPLES must be an integer of at least 2."
  [[ "${FEATURE_GATE_ATTEMPTS_VALUE}" =~ ^[1-5]$ ]] || \
    die "GO2_NAV_FEATURE_GATE_ATTEMPTS must be an integer from 1 through 5."
  is_positive_number "${FEATURE_MAX_HEADER_GAP_SEC_VALUE}" || \
    die "GO2_NAV_FEATURE_MAX_HEADER_GAP_SEC must be a finite positive number."
  is_positive_number "${FEATURE_MAX_RECEIVE_GAP_SEC_VALUE}" || \
    die "GO2_NAV_FEATURE_MAX_RECEIVE_GAP_SEC must be a finite positive number."
  is_positive_number "${FEATURE_MAX_HEADER_AGE_SEC_VALUE}" || \
    die "GO2_NAV_FEATURE_MAX_HEADER_AGE_SEC must be a finite positive number."
  awk \
    -v header_gap="${FEATURE_MAX_HEADER_GAP_SEC_VALUE}" \
    -v receive_gap="${FEATURE_MAX_RECEIVE_GAP_SEC_VALUE}" \
    -v header_age="${FEATURE_MAX_HEADER_AGE_SEC_VALUE}" \
    'BEGIN { exit !(header_gap <= 0.75 && receive_gap <= 0.75 && header_age <= 0.85) }' || \
    die "MCL feature freshness limits exceed the supervised startup safety caps."
  awk -v timeout="${FEATURE_TIMEOUT_SEC_VALUE}" -v window="${FEATURE_WINDOW_SEC_VALUE}" \
    'BEGIN { exit !(timeout > window) }' || \
    die "GO2_NAV_FEATURE_TIMEOUT_SEC must be greater than GO2_NAV_FEATURE_WINDOW_SEC."

  log "Running read-only odom/XT16 time-sync preflight..."
  ODOM_TIME_OFFSET_SEC_VALUE="$(
    DDDMR_IMAGE="${IMAGE}" \
    DDDMR_PLATFORM="${PLATFORM_VALUE}" \
    DDDMR_ROS_DISTRO="${ROS_DISTRO_VALUE}" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
    GO2_DDS_IP="${GO2_DDS_IP_VALUE}" \
    GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    GO2_DDS_EXTRA_IFACES="${GO2_DDS_EXTRA_IFACES_VALUE}" \
    GO2_DDS_PRIMARY_ADDRESS="${GO2_DDS_PRIMARY_ADDRESS_VALUE}" \
    GO2_DDS_ROBOT_TOPIC_PATTERNS="${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}" \
    GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}" \
    GO2_DDS_RCVBUF_MIN="${GO2_DDS_RCVBUF_MIN_VALUE}" \
      "${ODOM_OFFSET_RESOLVER}"
  )" || die "Odom/XT16 time-sync preflight failed."
  is_number "${ODOM_TIME_OFFSET_SEC_VALUE}" || \
    die "Resolved odom time offset is not finite: ${ODOM_TIME_OFFSET_SEC_VALUE}"
  export ODOM_TIME_OFFSET_SEC="${ODOM_TIME_OFFSET_SEC_VALUE}"
  log "Confirmed odom/XT16 time offset: ${ODOM_TIME_OFFSET_SEC_VALUE}s"
}

validate_live_request() {
  [[ "${GO2_NAV_LIVE_CONFIRM:-}" == "${LIVE_CONFIRM_PHRASE}" ]] || \
    die "Live mode requires GO2_NAV_LIVE_CONFIRM=${LIVE_CONFIRM_PHRASE}."
  [[ "${RUN_SECONDS_VALUE}" =~ ^[1-9][0-9]*$ ]] || \
    die "Live RUN_SECONDS must be a positive integer."
  (( RUN_SECONDS_VALUE <= 1800 )) || \
    die "Live RUN_SECONDS must not exceed 1800 seconds."
}

cleanup_runtime_on_exit() {
  local status=$?
  if [[ "${runtime_started}" == "true" ]] && \
     { (( status != 0 )) || [[ "${live_mode}" == "true" ]] || \
       [[ "${multi_mode}" == "true" ]] || [[ "${record_mode}" == "true" ]]; }; then
    log "Stopping supervised navigation runtime during exit cleanup..."
    stop_nav_containers || true
    runtime_started="false"
  fi
  return "${status}"
}

require_docker_image() {
  docker image inspect "${IMAGE}" >/dev/null 2>&1 || die "Docker image ${IMAGE} not found."
}

nav_container_names() {
  local current_name=""
  if [[ -f /tmp/go2_xt16_nav_current_name.txt ]]; then
    current_name="$(< /tmp/go2_xt16_nav_current_name.txt)"
  fi
  {
    docker ps -a --filter 'label=dddmr.go2_xt16_navigation=true' --format '{{.Names}}'
    docker ps --format '{{.Names}}' | grep -E '^go2_xt16_nav|^go2_xt16_navigation' || true
    if [[ -n "${current_name}" ]] && \
       docker ps -a --format '{{.Names}}' | grep -Fxq "${current_name}"; then
      printf '%s\n' "${current_name}"
    fi
  } | sort -u
}

host_nav_processes() {
  ps -eo pid,args | \
    grep -E '[g]o2_sport_cmd_vel_adapter|[g]o2_xt16_navigation[.]launch|[p]2p_move_base_node|[r]os2 topic pub /api/sport/request|[r]viz2' || true
}

stop_adapter_in_container() {
  local container="$1"
  docker exec "${container}" bash -lc 'set +e
pids="$(ps -eo pid,args | awk '"'"'/[g]o2_sport_cmd_vel_adapter.py/ {print $1}'"'"')"
if [ -n "${pids}" ]; then
  echo "stopping adapter pids: ${pids}"
  kill -TERM ${pids}
  sleep 1.5
fi
ps -eo pid,args | awk '"'"'/[g]o2_sport_cmd_vel_adapter.py/ {print}'"'"'
' || true
}

stop_nav_containers() {
  local names name
  names="$(nav_container_names)"
  if [[ -z "${names}" ]]; then
    log "No go2_xt16 navigation containers were found."
    return 0
  fi

  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if docker inspect -f '{{.State.Running}}' "${name}" 2>/dev/null | grep -Fxq true; then
      log "Stopping live adapter in ${name}, if present..."
      stop_adapter_in_container "${name}"
      log "Stopping navigation container: ${name}"
      docker stop -t 5 "${name}" >/dev/null
    else
      log "Removing stopped navigation container: ${name}"
    fi
    mkdir -p "${RUN_LOG_DIR}"
    local docker_log="${RUN_LOG_DIR}/${name}_docker.log"
    if docker logs "${name}" >"${docker_log}" 2>&1; then
      log "Saved navigation log: ${docker_log}"
    else
      log "WARNING: failed to save navigation log for ${name}"
    fi
    docker rm "${name}" >/dev/null
  done <<< "${names}"
  rm -f /tmp/go2_xt16_nav_current_name.txt
}

assert_clean_runtime() {
  local containers processes
  containers="$(nav_container_names)"
  if [[ -n "${containers}" ]]; then
    if [[ "${STOP_EXISTING_VALUE}" == "true" ]]; then
      stop_nav_containers
    else
      echo "${containers}" >&2
      die "A navigation container already exists. Use --stop first or STOP_EXISTING=true."
    fi
  fi

  processes="$(host_nav_processes)"
  if [[ -n "${processes}" ]]; then
    echo "${processes}" >&2
    die "Navigation/RViz/Sport process is still running. Stop it before starting a new test."
  fi
}

docker_ros() {
  docker exec "${CONTAINER_NAME}" bash -lc "set -eo pipefail
set +u
source ${ROS_SETUP_FILE_VALUE}
if [[ -f /opt/unitree_ros2/setup.bash ]]; then
  source /opt/unitree_ros2/setup.bash
fi
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
cd /root/dddmr_navigation
$*"
}

wait_for_node() {
  local node="$1"
  local timeout_sec="${2:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local next_progress=$((SECONDS + 15))

  log "Waiting for node ${node} (timeout ${timeout_sec}s)..."
  while (( SECONDS < deadline )); do
    if ! docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null | grep -Fxq true; then
      log "Container ${CONTAINER_NAME} exited while waiting for node ${node}."
      docker logs --tail 80 "${CONTAINER_NAME}" 2>&1 || true
      return 1
    fi
    if docker_ros "timeout 5 ros2 node list | grep -Fx '${node}'" >/dev/null 2>&1; then
      log "Ready node: ${node}"
      return 0
    fi
    if (( SECONDS >= next_progress )); then
      log "Still waiting for node ${node}..."
      next_progress=$((SECONDS + 15))
    fi
    sleep 1
  done
  return 1
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local deadline=$((SECONDS + timeout_sec))
  local next_progress=$((SECONDS + 15))

  log "Waiting for topic ${topic} (timeout ${timeout_sec}s)..."
  while (( SECONDS < deadline )); do
    if ! docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null | grep -Fxq true; then
      log "Container ${CONTAINER_NAME} exited while waiting for topic ${topic}."
      docker logs --tail 80 "${CONTAINER_NAME}" 2>&1 || true
      return 1
    fi
    if docker_ros "timeout 5 ros2 topic list | grep -Fx '${topic}'" >/dev/null 2>&1; then
      log "Ready topic: ${topic}"
      return 0
    fi
    if (( SECONDS >= next_progress )); then
      log "Still waiting for topic ${topic}..."
      next_progress=$((SECONDS + 15))
    fi
    sleep 1
  done
  return 1
}

wait_for_pointcloud_sample() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local durability="${3:-volatile}"
  local deadline=$((SECONDS + timeout_sec))
  local report width rc

  log "Waiting for a non-empty ${topic} point-cloud sample (timeout ${timeout_sec}s)..."
  while (( SECONDS < deadline )); do
    set +e
    report="$(docker_ros "set +o pipefail
timeout 6 ros2 topic echo '${topic}' sensor_msgs/msg/PointCloud2 \
  --qos-reliability reliable --qos-durability '${durability}' \
  --no-arr --no-str 2>/dev/null |
awk '\$1 == \"width:\" {print \$2; exit}'" 2>&1)"
    rc=$?
    set -e
    if (( rc == 0 )); then
      width="$(awk '/^[[:space:]]*[0-9]+[[:space:]]*$/ {print $1; exit}' <<<"${report}")"
      if [[ "${width}" =~ ^[0-9]+$ ]] && (( 10#${width} > 0 )); then
        log "Received ${topic} with width=${width}."
        return 0
      fi
    fi
    sleep 1
  done
  printf '%s\n' "${report:-no sample output}" >&2
  return 1
}

wait_for_container_log_pattern() {
  local description="$1"
  local pattern="$2"
  local timeout_sec="${3:-90}"
  local deadline=$((SECONDS + timeout_sec))
  local report=""

  log "Waiting for ${description} in the current navigation log (timeout ${timeout_sec}s)..."
  # The readiness record can precede this gate and can be pushed out of any
  # fixed-size tail by noisy startup diagnostics. Scan this unique container
  # from its first record. Stop reading at the first match so early readiness
  # remains cheap to verify even if later diagnostics are noisy.
  while (( SECONDS < deadline )); do
    if ! docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null | grep -Fxq true; then
      log "Container ${CONTAINER_NAME} exited while waiting for ${description}."
      docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
      return 1
    fi
    report="$(
      docker logs "${CONTAINER_NAME}" 2>&1 |
        grep -m 1 -E "${pattern}" ||
        true
    )"
    if [[ -n "${report}" ]]; then
      printf '%s\n' "${report}"
      log "Confirmed ${description}."
      return 0
    fi
    sleep 1
  done
  return 1
}

check_odom_sync_runtime() {
  local report rc standardizer_report standardizer_offset attempt
  standardizer_report="$(docker_ros "timeout 10 ros2 param get /go2_odom_standardizer stamp_time_offset_sec" 2>&1)" || {
    printf '%s\n' "${standardizer_report}" >&2
    die "Could not read the standardized odometry time offset."
  }
  standardizer_offset="$(awk -F': ' '/Double value is:/ {print $2; exit}' <<<"${standardizer_report}")"
  is_number "${standardizer_offset}" || \
    die "Invalid stamp_time_offset_sec parameter response: ${standardizer_report}"
  awk -v actual="${standardizer_offset}" -v expected="${ODOM_TIME_OFFSET_SEC_VALUE}" \
    'BEGIN {delta=actual-expected; if (delta<0) delta=-delta; exit !(delta <= 1e-6)}' || \
    die "Standardized odometry offset ${standardizer_offset}s does not match measured ${ODOM_TIME_OFFSET_SEC_VALUE}s."
  log "Standardized odometry is applying ${standardizer_offset}s before all navigation consumers."

  for (( attempt = 1; attempt <= ODOM_SYNC_GATE_ATTEMPTS_VALUE; attempt++ )); do
    log "Requiring ${ODOM_SYNC_WINDOW_SEC_VALUE}s of continuous valid /odom_sync_diagnostics (attempt ${attempt}/${ODOM_SYNC_GATE_ATTEMPTS_VALUE})..."
    set +e
    report="$(docker_ros "timeout 50 python3 /root/dddmr_navigation/scripts/check_go2_odom_sync.py \
      --timeout 40 \
      --max-error '${ODOM_SYNC_TOLERANCE_SEC_VALUE}' \
      --expected-offset 0.0 \
      --window-sec '${ODOM_SYNC_WINDOW_SEC_VALUE}' \
      --min-samples '${ODOM_SYNC_MIN_SAMPLES_VALUE}' \
      --max-receive-gap-sec '${ODOM_SYNC_MAX_RECEIVE_GAP_SEC_VALUE}'" 2>&1)"
    rc=$?
    set -e
    printf 'ODOM_SYNC_GATE_ATTEMPT=%d/%d\n' \
      "${attempt}" "${ODOM_SYNC_GATE_ATTEMPTS_VALUE}"
    printf '%s\n' "${report}"
    if (( rc == 0 )); then
      log "Live odom/XT16 synchronization passed on attempt ${attempt}."
      return 0
    fi
    if (( attempt < ODOM_SYNC_GATE_ATTEMPTS_VALUE )); then
      log "Odom synchronization was not yet stable; keeping motion blocked and retrying."
      sleep 2
    fi
  done

  docker logs --tail 100 "${CONTAINER_NAME}" 2>&1 || true
  die "Live odom/XT16 synchronization did not pass after ${ODOM_SYNC_GATE_ATTEMPTS_VALUE} attempts."
}

check_topic_contract() {
  local topic="$1"
  local expected="$2"
  local minimum_publishers="${3:-0}"
  local minimum_subscriptions="${4:-0}"
  local info
  local publisher_count
  local subscription_count

  info="$(docker_ros "timeout 10 ros2 topic info '${topic}'" 2>&1)" || {
    echo "${info}" >&2
    die "Topic ${topic} is not readable."
  }
  printf '%s\n' "${info}"
  [[ "${info}" == *"Type: ${expected}"* ]] || \
    die "Topic ${topic} is not ${expected}."

  publisher_count="$(awk '/^Publisher count:/ {print $3; exit}' <<< "${info}")"
  subscription_count="$(awk '/^Subscription count:/ {print $3; exit}' <<< "${info}")"
  [[ "${publisher_count}" =~ ^[0-9]+$ ]] || \
    die "Topic ${topic} did not report a valid publisher count."
  [[ "${subscription_count}" =~ ^[0-9]+$ ]] || \
    die "Topic ${topic} did not report a valid subscription count."
  (( publisher_count >= minimum_publishers )) || \
    die "Topic ${topic} requires at least ${minimum_publishers} publisher(s); found ${publisher_count}."
  (( subscription_count >= minimum_subscriptions )) || \
    die "Topic ${topic} requires at least ${minimum_subscriptions} subscriber(s); found ${subscription_count}."
}

read_runtime_double_parameter() {
  local node="$1"
  local parameter="$2"
  local label="$3"
  local report="" value="" attempt
  for (( attempt = 1; attempt <= 6; attempt++ )); do
    if report="$(docker_ros \
      "timeout 10 ros2 param get '${node}' '${parameter}'" \
      2>&1)"; then
      value="$(awk -F': ' '/Double value is:/ {print $2; exit}' <<<"${report}")"
      if is_number "${value}"; then
        printf '%s\n' "${value}"
        return 0
      fi
    fi
    if (( attempt < 6 )); then
      log "Parameter ${node}/${parameter} is not readable yet (attempt ${attempt}/6); keeping navigation blocked and retrying." >&2
      sleep 2
    fi
  done
  printf '%s\n' "${report}" >&2
  die "Could not read ${label} from ${node} after 6 attempts."
}

read_local_lidar_freshness_limit() {
  local value
  value="$(
    read_runtime_double_parameter \
      /perception_3d_local \
      lidar.expected_sensor_time \
      "the local LiDAR freshness limit"
  )"
  is_positive_number "${value}" || \
    die "Local LiDAR freshness limit must be positive, got: ${value}"
  awk \
    -v actual="${value}" \
    -v requested="${LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC_VALUE}" \
    'BEGIN { delta=actual-requested; if(delta<0) delta=-delta; exit !(delta <= 1e-6) }' || \
    die "Local LiDAR freshness limit ${value}s does not match requested ${LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC_VALUE}s."
  LOCAL_LIDAR_RUNTIME_FRESHNESS_SEC_VALUE="${value}"
  log "Local LiDAR hard freshness limit: ${value}s"
  LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE="$(
    awk \
      -v runtime_limit="${value}" \
      -v margin="${OBSERVATION_STARTUP_AGE_MARGIN_SEC_VALUE}" \
      'BEGIN { printf "%.6f", runtime_limit + margin }'
  )"
  awk -v value="${LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE}" \
    'BEGIN { exit !(value <= 0.40) }' || \
    die "Local LiDAR startup age ceiling must not exceed 0.40s."
  log "Startup observation age ceiling: ${LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE}s (runtime remains ${value}s)"
}

require_planner_lateral_limits() {
  local minimum_y maximum_y lateral_samples expected_samples
  minimum_y="$(
    read_runtime_double_parameter \
      /trajectory_generators \
      omni_drive_simple.min_vel_y \
      "the planner minimum lateral velocity"
  )"
  maximum_y="$(
    read_runtime_double_parameter \
      /trajectory_generators \
      omni_drive_simple.max_vel_y \
      "the planner maximum lateral velocity"
  )"
  lateral_samples="$(
    read_runtime_double_parameter \
      /trajectory_generators \
      omni_drive_simple.linear_y_sample \
      "the planner lateral sample count"
  )"

  if awk -v value="${MAX_Y_VALUE}" 'BEGIN { exit !(value == 0.0) }'; then
    expected_samples="1.0"
  else
    expected_samples="3.0"
  fi
  awk \
    -v actual="${minimum_y}" \
    -v requested="${OMNI_MIN_Y_VALUE}" \
    'BEGIN { delta=actual-requested; if(delta<0) delta=-delta; exit !(delta <= 1e-6) }' || \
    die "Planner minimum lateral limit ${minimum_y}m/s does not match requested ${OMNI_MIN_Y_VALUE}m/s."
  awk \
    -v actual="${maximum_y}" \
    -v requested="${MAX_Y_VALUE}" \
    'BEGIN { delta=actual-requested; if(delta<0) delta=-delta; exit !(delta <= 1e-6) }' || \
    die "Planner maximum lateral limit ${maximum_y}m/s does not match requested ${MAX_Y_VALUE}m/s."
  awk \
    -v actual="${lateral_samples}" \
    -v expected="${expected_samples}" \
    'BEGIN { exit !(actual == expected) }' || \
    die "Planner lateral sample count must be ${expected_samples}, got ${lateral_samples}."
  log "P2P lateral envelope: min_y=${minimum_y}m/s max_y=${maximum_y}m/s samples=${lateral_samples}"
}

require_lateral_parameter() {
  local node="$1"
  local value
  value="$(
    read_runtime_double_parameter \
      "${node}" \
      max_y \
      "the lateral command limit"
  )"
  awk \
    -v actual="${value}" \
    -v requested="${MAX_Y_VALUE}" \
    'BEGIN { delta=actual-requested; if(delta<0) delta=-delta; exit !(delta <= 1e-6) }' || \
    die "${node} lateral command limit ${value}m/s does not match requested ${MAX_Y_VALUE}m/s."
  log "${node} lateral command envelope: max_y=${value}m/s"
}

require_sport_axis_contract() {
  local node="$1"
  local report="" axis_mode="" sign="" parameter="" attempt
  for (( attempt = 1; attempt <= 6; attempt++ )); do
    if report="$(docker_ros \
      "timeout 15 ros2 param dump '${node}' --print" \
      2>&1)"; then
      axis_mode="$(
        awk '$1 == "axis_mode:" {print $2; exit}' <<<"${report}"
      )"
      if [[ -n "${axis_mode}" ]]; then
        break
      fi
    fi
    if (( attempt < 6 )); then
      log "Sport axis contract for ${node} is not readable yet (attempt ${attempt}/6); keeping navigation blocked and retrying." >&2
      sleep 2
    fi
  done
  axis_mode="${axis_mode//\'/}"
  axis_mode="${axis_mode//\"/}"
  [[ "${axis_mode}" == "standard" ]] || \
    die "${node} axis_mode must be standard, got ${axis_mode:-unreadable}."

  for parameter in x_sign y_sign yaw_sign; do
    sign="$(
      awk -v key="${parameter}:" '$1 == key {print $2; exit}' <<<"${report}"
    )"
    is_number "${sign}" || \
      die "${node} ${parameter} is not a readable number."
    awk -v actual="${sign}" \
      'BEGIN { delta=actual-1.0; if(delta<0) delta=-delta; exit !(delta <= 1e-6) }' || \
      die "${node} ${parameter} must be +1.0, got ${sign}."
  done
  log "${node} axis contract: standard, x/y/yaw signs=+1"
}

require_navigation_runtime_parameters() {
  log "Reading back exact P2P runtime safety parameters..."
  read_local_lidar_freshness_limit
  require_planner_lateral_limits
  require_lateral_parameter /go2_nav_cmd_gate
  require_lateral_parameter /go2_sport_cmd_vel_dry_run
  require_sport_axis_contract /go2_sport_cmd_vel_dry_run
  log "Exact P2P runtime safety parameters match the requested launch limits."
}

require_mcl_feature_stream() {
  local output status gate_path attempt
  gate_path="/root/dddmr_navigation/src/dddmr_beginner_guide/scripts/go2_pointcloud_stream_gate.py"

  : >"${FEATURE_GATE_LOG_HOST}"
  for (( attempt = 1; attempt <= FEATURE_GATE_ATTEMPTS_VALUE; attempt++ )); do
    log "Validating ${FEATURE_WINDOW_SEC_VALUE}s of continuous MCL feature input (attempt ${attempt}/${FEATURE_GATE_ATTEMPTS_VALUE})..."
    set +e
    output="$(docker_ros \
      "timeout -s INT -k 1s 30s python3 '${gate_path}' \
        --topic '${MCL_FEATURE_TOPIC}' \
        --window-sec '${FEATURE_WINDOW_SEC_VALUE}' \
        --timeout-sec '${FEATURE_TIMEOUT_SEC_VALUE}' \
        --min-samples '${FEATURE_MIN_SAMPLES_VALUE}' \
        --min-rate-hz '${OBSERVATION_MIN_RATE_HZ_VALUE}' \
        --max-header-gap-sec '${FEATURE_MAX_HEADER_GAP_SEC_VALUE}' \
        --max-receive-gap-sec '${FEATURE_MAX_RECEIVE_GAP_SEC_VALUE}' \
        --max-header-age-sec '${FEATURE_MAX_HEADER_AGE_SEC_VALUE}' \
        --max-future-skew-sec '${OBSERVATION_MAX_FUTURE_SKEW_SEC_VALUE}' \
        --expected-publishers 1" 2>&1)"
    status=$?
    set -e
    {
      printf 'FEATURE_GATE_ATTEMPT=%d/%d\n' \
        "${attempt}" "${FEATURE_GATE_ATTEMPTS_VALUE}"
      printf '%s\n' "${output}"
    } | tee -a "${FEATURE_GATE_LOG_HOST}"

    if (( status == 0 )) && \
       grep -Fxq 'CURRENT_OBSERVATION_GATE=PASS' <<<"${output}"; then
      log "Sustained MCL feature-stream gate passed on attempt ${attempt}."
      return 0
    fi
    if (( attempt < FEATURE_GATE_ATTEMPTS_VALUE )); then
      log "MCL feature stream was not yet stable; keeping motion blocked and retrying."
      sleep 2
    fi
  done

  docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
  die "MCL feature input did not remain fresh after ${FEATURE_GATE_ATTEMPTS_VALUE} attempts; navigation was not armed."
}

require_current_observation_stream() {
  local output status gate_path attempt
  gate_path="/root/dddmr_navigation/src/dddmr_beginner_guide/scripts/go2_pointcloud_stream_gate.py"

  [[ -n "${LOCAL_LIDAR_RUNTIME_FRESHNESS_SEC_VALUE}" ]] || \
    die "Local LiDAR runtime freshness was not validated before the observation gate."
  [[ -n "${LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE}" ]] || \
    die "Local LiDAR startup age ceiling was not derived before the observation gate."

  : >"${PERCEPTION_GATE_LOG_HOST}"
  for (( attempt = 1; attempt <= OBSERVATION_GATE_ATTEMPTS_VALUE; attempt++ )); do
    log "Validating ${OBSERVATION_WINDOW_SEC_VALUE}s of fresh local LiDAR observations (attempt ${attempt}/${OBSERVATION_GATE_ATTEMPTS_VALUE})..."
    set +e
    output="$(docker_ros \
      "timeout -s INT -k 1s 50s python3 '${gate_path}' \
        --topic '${CURRENT_OBSERVATION_TOPIC}' \
        --window-sec '${OBSERVATION_WINDOW_SEC_VALUE}' \
        --timeout-sec '${OBSERVATION_TIMEOUT_SEC_VALUE}' \
        --min-samples '${OBSERVATION_MIN_SAMPLES_VALUE}' \
        --min-rate-hz '${OBSERVATION_MIN_RATE_HZ_VALUE}' \
        --max-header-gap-sec '${OBSERVATION_MAX_HEADER_GAP_SEC_VALUE}' \
        --max-receive-gap-sec '${OBSERVATION_MAX_RECEIVE_GAP_SEC_VALUE}' \
        --max-header-age-sec '${LOCAL_LIDAR_STARTUP_MAX_HEADER_AGE_SEC_VALUE}' \
        --max-future-skew-sec '${OBSERVATION_MAX_FUTURE_SKEW_SEC_VALUE}' \
        --reliability reliable \
        --expected-publishers 1" 2>&1)"
    status=$?
    set -e
    {
      printf 'CURRENT_OBSERVATION_GATE_ATTEMPT=%d/%d\n' \
        "${attempt}" "${OBSERVATION_GATE_ATTEMPTS_VALUE}"
      printf '%s\n' "${output}"
    } | tee -a "${PERCEPTION_GATE_LOG_HOST}"

    if (( status == 0 )) && \
       grep -Fxq 'CURRENT_OBSERVATION_GATE=PASS' <<<"${output}"; then
      log "Sustained local LiDAR freshness gate passed on attempt ${attempt}."
      return 0
    fi
    if (( attempt < OBSERVATION_GATE_ATTEMPTS_VALUE )); then
      log "Local LiDAR stream was not yet stable; keeping motion blocked and retrying."
      sleep 2
    fi
  done

  docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
  die "Local LiDAR observations did not remain fresh after ${OBSERVATION_GATE_ATTEMPTS_VALUE} attempts; Sport output was not enabled."
}

start_container() {
  local start_clicked_to_goal="true"
  local p2p_goals_enabled="true"
  local start_p2p_mission="false"
  local p2p_mission_file=""
  local p2p_mission_launch_command=""
  local map_config_launch_command=""
  local -a map_config_mount=()
  if [[ -n "${RUNTIME_NAV_CONFIG_HOST}" ]]; then
    map_config_mount=(
      -v
      "${RUNTIME_NAV_CONFIG_HOST}:${RUNTIME_NAV_CONFIG_CONTAINER}:ro"
    )
    printf -v map_config_launch_command \
      'launch_args+=(%q %q)' \
      "config_file:=${RUNTIME_NAV_CONFIG_CONTAINER}" \
      "map_config_file:=${RUNTIME_NAV_CONFIG_CONTAINER}"
  fi
  if [[ "${multi_mode}" == "true" ]]; then
    start_clicked_to_goal="false"
    p2p_goals_enabled="false"
    start_p2p_mission="true"
    p2p_mission_file="${mission_file_container}"
    # ROS 2 rejects an explicit empty `name:=` launch override. Append this
    # argument only when a validated multi-point mission actually exists.
    printf -v p2p_mission_launch_command \
      'launch_args+=(%q %q)' \
      "p2p_mission_file:=${p2p_mission_file}" \
      "p2p_mission_planner_ready_timeout_sec:=240.0"
  elif [[ "${record_mode}" == "true" ]]; then
    start_clicked_to_goal="false"
    p2p_goals_enabled="false"
  fi
  mkdir -p "${BAGS_DIR}" "${RUN_LOG_DIR}"

  if [[ "${RVIZ_VALUE}" == "true" && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null || true
  fi

  log "Starting navigation container: ${CONTAINER_NAME}"
  if [[ "${live_mode}" == "true" && "${multi_mode}" != "true" ]]; then
    log "Do not send an RViz goal until all readiness checks have passed."
  elif [[ "${multi_mode}" == "true" ]]; then
    log "RViz manual goals are disabled; the P2P mission remains locked until READY."
  fi
  docker run -d \
    "${DOCKER_RUN_ARGS[@]}" \
    --name "${CONTAINER_NAME}" \
    --label "dddmr.go2_xt16_navigation=true" \
    --network=host \
    --privileged \
    -e "DISPLAY=${DISPLAY:-:0}" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}" \
    -e "GO2_DDS_IP=${GO2_DDS_IP_VALUE}" \
    -e "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}" \
    -e "GO2_DDS_EXTRA_IFACES=${GO2_DDS_EXTRA_IFACES_VALUE}" \
    -e "GO2_DDS_PRIMARY_ADDRESS=${GO2_DDS_PRIMARY_ADDRESS_VALUE}" \
    -e "GO2_DDS_ROBOT_TOPIC_PATTERNS=${GO2_DDS_ROBOT_TOPIC_PATTERNS_VALUE}" \
    -e "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX=${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX_VALUE}" \
    -e "GO2_DDS_RCVBUF_MIN=${GO2_DDS_RCVBUF_MIN_VALUE}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e "DDDMR_PLATFORM=${PLATFORM_VALUE}" \
    -e "DDDMR_ROS_DISTRO=${ROS_DISTRO_VALUE}" \
    -e "DDDMR_BUILD_BASE=${BUILD_BASE_VALUE}" \
    -e "DDDMR_INSTALL_BASE=${INSTALL_BASE_VALUE}" \
    -e "DDDMR_LOG_BASE=${LOG_BASE_VALUE}" \
    -v "${BAGS_DIR}:${BAGS_DIR}" \
    -v "/tmp:/tmp" \
    -v "/dev:/dev" \
    -v "${WS_ROOT}:/root/dddmr_navigation" \
    -v "${BAGS_DIR}:/root/dddmr_bags" \
    "${map_config_mount[@]}" \
    "${IMAGE}" \
    bash -lc "set -eo pipefail
set +u
source ${ROS_SETUP_FILE_VALUE}
if [[ -f /opt/unitree_ros2/setup.bash ]]; then
  source /opt/unitree_ros2/setup.bash
fi
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
cd /root/dddmr_navigation
launch_args=(
  \"rviz:=${RVIZ_VALUE}\"
  \"publish_static_tf:=${PUBLISH_STATIC_TF_VALUE}\"
  \"odom_sync_enabled:=true\"
  \"odom_sync_tolerance_sec:=${ODOM_SYNC_TOLERANCE_SEC_VALUE}\"
  \"odom_sync_wait_timeout_sec:=${ODOM_SYNC_WAIT_TIMEOUT_SEC_VALUE}\"
  \"odom_time_offset_sec:=${ODOM_TIME_OFFSET_SEC_VALUE}\"
  \"local_lidar_expected_sensor_time_sec:=${LOCAL_LIDAR_EXPECTED_SENSOR_TIME_SEC_VALUE}\"
  \"omni_min_vel_y:=${OMNI_MIN_Y_VALUE}\"
  \"omni_max_vel_y:=${MAX_Y_VALUE}\"
  \"go2_nav_cmd_gate_max_x:=${MAX_X_VALUE}\"
  \"go2_nav_cmd_gate_max_y:=${MAX_Y_VALUE}\"
  \"sport_dry_run_max_x:=${MAX_X_VALUE}\"
  \"sport_dry_run_max_y:=${MAX_Y_VALUE}\"
  \"go2_sport_max_x:=${MAX_X_VALUE}\"
  \"go2_sport_max_y:=${MAX_Y_VALUE}\"
  \"go2_nav_cmd_gate_max_yaw:=${MAX_YAW_VALUE}\"
  \"sport_dry_run_max_yaw:=${MAX_YAW_VALUE}\"
  \"start_clicked_to_goal:=${start_clicked_to_goal}\"
  \"p2p_goals_enabled:=${p2p_goals_enabled}\"
  \"start_p2p_mission:=${start_p2p_mission}\"
  \"start_sport_dry_run_adapter:=true\"
  \"start_go2_sport_adapter:=false\"
  \"go2_sport_enable_output:=false\"
  \"go2_sport_allow_real_request_topic:=false\"
  \"go2_sport_max_yaw:=${MAX_YAW_VALUE}\"
)
${map_config_launch_command}
${p2p_mission_launch_command}
exec ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch \
  \"\${launch_args[@]}\"" >/dev/null

  printf '%s\n' "${CONTAINER_NAME}" >/tmp/go2_xt16_nav_current_name.txt
}

run_waypoint_recorder() {
  log "Starting interactive P2P waypoint recorder."
  docker exec -it "${CONTAINER_NAME}" bash -lc "set -eo pipefail
set +u
source ${ROS_SETUP_FILE_VALUE}
if [[ -f /opt/unitree_ros2/setup.bash ]]; then
  source /opt/unitree_ros2/setup.bash
fi
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
exec ros2 run dddmr_route_navigation p2p_waypoint_recorder.py \
  --mission-file '${mission_file_container}' \
  --initial-pose-file '${initial_pose_file_container}'"
}

read_mission_state() {
  local output
  output="$(
    docker_ros \
      "timeout -s INT -k 1s 3s ros2 topic echo \
        --qos-reliability reliable \
        --qos-durability transient_local \
        /p2p_multi_point/state std_msgs/msg/String" \
      2>/dev/null || true
  )"
  awk -F': ' '
    $1 == "data" {
      gsub(/^["'\''"]|["'\''"]$/, "", $2)
      if ($2 == "FAILED") {
        print "FAILED"
        printed = 1
        exit
      }
      if ($2 == "READY")
        saw_ready = 1
      last = $2
    }
    END {
      if (!printed && saw_ready)
        print "READY"
      else if (!printed && last != "")
        print last
    }
  ' <<<"${output}"
}

wait_for_mission_ready() {
  local timeout_sec="${1:-120}"
  local deadline=$((SECONDS + timeout_sec))
  local state=""
  log "Waiting for P2P mission READY (timeout ${timeout_sec}s)..."
  while (( SECONDS < deadline )); do
    state="$(read_mission_state)"
    case "${state}" in
      READY)
        log "Mission ${mission_id} is localized and READY."
        return 0
        ;;
      FAILED)
        docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
        die "P2P mission entered FAILED before arming."
        ;;
    esac
    sleep 1
  done
  docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
  die "Timed out waiting for P2P mission READY (last state=${state:-missing})."
}

arm_multi_mission() {
  local expected="EXECUTE ${mission_id}"
  local answer=""
  local tty_fd
  if ! exec {tty_fd}<>/dev/tty; then
    die "multi-point execution requires an interactive controlling terminal"
  fi
  if [[ ! -t "${tty_fd}" ]]; then
    exec {tty_fd}>&-
    die "multi-point execution requires an interactive controlling terminal"
  fi

  # A newline or VEOF entered during the long readiness checks must not count
  # as the post-READY motion confirmation. Drain queued terminal input before
  # displaying the prompt, then require a newly entered exact phrase.
  while IFS= read -r -t 0.05 -u "${tty_fd}" answer; do
    :
  done
  while true; do
    answer=""
    printf 'Type exactly `%s` to submit waypoint 1: ' "${expected}" >&"${tty_fd}"
    if ! IFS= read -r -u "${tty_fd}" answer; then
      printf '\n' >&"${tty_fd}"
      exec {tty_fd}>&-
      die "mission confirmation input closed before a response was received"
    fi
    [[ -n "${answer//[[:space:]]/}" ]] && break
    printf 'Empty confirmation ignored; enter the exact phrase after the prompt.\n' \
      >&"${tty_fd}"
  done
  exec {tty_fd}>&-
  [[ "${answer}" == "${expected}" ]] || die "mission execution cancelled"

  # READY is a live condition, not a promise. Recheck immediately before the
  # service call so a brief localization gap cannot turn confirmation into a
  # misleading rejected arm request.
  wait_for_mission_ready 15
  local response
  response="$(
    docker_ros \
      "timeout 10 ros2 service call /p2p_multi_point/arm std_srvs/srv/Trigger '{}'" \
      2>&1
  )" || {
    printf '%s\n' "${response}" >&2
    die "P2P mission arm service call failed"
  }
  printf '%s\n' "${response}"
  if grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${response}"; then
    return 0
  fi
  local state
  state="$(read_mission_state)"
  if [[ "${state}" == "FAILED" ]] || grep -q 'state=FAILED' <<<"${response}"; then
    die "P2P mission entered terminal FAILED before arm"
  fi
  log "Mission readiness changed before arm (state=${state:-missing}); waiting again."
  return 1
}

monitor_multi_mission() {
  local timeout_sec="${P2P_MISSION_MONITOR_TIMEOUT_SEC:-${RUN_SECONDS_VALUE:-3600}}"
  [[ "${timeout_sec}" =~ ^[1-9][0-9]*$ ]] || \
    die "P2P_MISSION_MONITOR_TIMEOUT_SEC must be a positive integer"
  local deadline=$((SECONDS + timeout_sec))
  local state=""
  log "Monitoring P2P mission for up to ${timeout_sec}s..."
  while (( SECONDS < deadline )); do
    state="$(read_mission_state)"
    case "${state}" in
      COMPLETE)
        log "P2P mission ${mission_id} completed."
        return 0
        ;;
      FAILED)
        docker logs --tail 120 "${CONTAINER_NAME}" 2>&1 || true
        die "P2P mission ${mission_id} failed."
        ;;
    esac
    sleep 1
  done
  docker_ros \
    "timeout 10 ros2 service call /p2p_multi_point/cancel std_srvs/srv/Trigger '{}'" \
    >/dev/null 2>&1 || true
  die "P2P mission monitor timed out (last state=${state:-missing})."
}

start_live_adapter() {
  log "Starting live Sport adapter: max_x=${MAX_X_VALUE}, max_y=${MAX_Y_VALUE}, max_yaw=${MAX_YAW_VALUE}"
  docker exec "${CONTAINER_NAME}" bash -lc "set -e
mkdir -p /root/dddmr_navigation/run_logs
source ${ROS_SETUP_FILE_VALUE}
if [[ -f /opt/unitree_ros2/setup.bash ]]; then
  source /opt/unitree_ros2/setup.bash
elif [[ -f /root/dddmr_navigation/.unitree_msg_ws/install/setup.bash ]]; then
  source /root/dddmr_navigation/.unitree_msg_ws/install/setup.bash
else
  echo 'Missing Unitree ROS 2 message overlay: expected /opt/unitree_ros2/setup.bash or /root/dddmr_navigation/.unitree_msg_ws/install/setup.bash.' >&2
  exit 1
fi
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
python3 -c 'from unitree_api.msg import Request' || {
  echo 'The selected Unitree ROS 2 overlay does not provide unitree_api.msg.Request.' >&2
  exit 1
}
nohup python3 /root/dddmr_navigation/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py \
  --ros-args \
  -r __node:=go2_sport_cmd_vel_adapter_live \
  -p cmd_vel_topic:=/dddmr_go2/safe_cmd_vel \
  -p request_topic:=/api/sport/request \
  -p enable_sport_output:=true \
  -p allow_real_request_topic:=true \
  -p axis_mode:=standard \
  -p max_x:=${MAX_X_VALUE} \
  -p max_y:=${MAX_Y_VALUE} \
  -p max_yaw:=${MAX_YAW_VALUE} \
  -p x_sign:=1.0 \
  -p y_sign:=1.0 \
  -p yaw_sign:=1.0 \
  -p publish_rate_hz:=50.0 \
  -p cmd_timeout_sec:=0.20 \
  -p zero_epsilon:=0.001 \
  -p stop_keepalive_hz:=2.0 \
  -p require_motion_decision:=true \
  -p motion_allowed_decisions:="'d_controlling,d_align_heading,d_align_goal_heading,d_recovery_waitdone'" \
  -p enable_yaw_arc_shim:=false \
  > '${ADAPTER_LOG_CONTAINER}' 2>&1 &
echo \$!" >/tmp/go2_xt16_nav_adapter_pid.txt

  printf '%s\n' "${ADAPTER_LOG_CONTAINER}" >/tmp/go2_xt16_nav_current_adapter_log.txt
}

print_status() {
  log "Container status:"
  docker ps --format '{{.Names}}\t{{.Status}}\t{{.Image}}' | grep -E "^${CONTAINER_NAME}\b" || true

  log "ROS parameters:"
  docker_ros "ros2 param get /go2_nav_cmd_gate max_x; ros2 param get /go2_nav_cmd_gate max_y; ros2 param get /go2_nav_cmd_gate max_yaw; ros2 param get /trajectory_generators omni_drive_simple.min_vel_y; ros2 param get /trajectory_generators omni_drive_simple.max_vel_y; ros2 param get /trajectory_generators differential_drive_rotate_shortest_angle.rotation_speed; ros2 param get /trajectory_generators differential_drive_rotate_inplace.rotation_speed" || true

  log "Map and command topics:"
  docker_ros "ros2 topic list | sort | grep -E 'map1/mapcloud|map1/mapground|sub_map|safe_cmd_vel|dry_run_cmd_vel|lidar_points|utlidar/robot_odom' || true" || true

  if [[ "${live_mode}" == "true" ]]; then
    log "Live adapter log: ${ADAPTER_LOG_HOST}"
    if [[ -f "${ADAPTER_LOG_HOST}" ]]; then
      tail -n 20 "${ADAPTER_LOG_HOST}" || true
    fi
    log "Current safe_cmd_vel sample:"
    docker_ros "timeout 4 ros2 topic echo /dddmr_go2/safe_cmd_vel geometry_msgs/msg/Twist --once" || true
  fi
}

main() {
  if [[ "${docker_sudo_auto_enabled}" == "true" ]]; then
    log "Docker socket is not writable; using sudo for --stop."
  fi
  require_docker_access

  if [[ "${mode}" == "stop" ]]; then
    stop_nav_containers
    return 0
  fi

  validate_mission_request

  if [[ "${live_mode}" == "true" ]]; then
    validate_live_request
  fi
  trap cleanup_runtime_on_exit EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  validate_lateral_limit
  log "Startup profile: ${STARTUP_PROFILE_VALUE}"
  prepare_map_override
  configure_static_layer_timeout
  require_docker_image
  assert_clean_runtime
  validate_perception_settings
  if [[ "${GO2_DDS_RCVBUF_MIN_VALUE}" == "default" ]]; then
    log "Using the platform kernel-default DDS receive-buffer minimum."
  else
    log "Checking host DDS receive buffers before starting any ROS node..."
    "${DDS_BUFFER_CHECK}" || die "Host DDS receive-buffer check failed."
  fi
  resolve_odom_time_offset
  start_container
  runtime_started="true"

  log "Starting navigation and Go2 DDS readiness checks..."
  wait_for_node /go2_nav_cmd_gate 90 || die "Timed out waiting for /go2_nav_cmd_gate"
  wait_for_node /perception_3d_local 90 || die "Timed out waiting for /perception_3d_local"
  wait_for_node /trajectory_generators 90 || die "Timed out waiting for /trajectory_generators"
  require_navigation_runtime_parameters
  wait_for_topic /odom_sync_diagnostics 90 || die "Timed out waiting for /odom_sync_diagnostics"
  check_odom_sync_runtime
  wait_for_topic "${MCL_FEATURE_TOPIC}" 90 || die "Timed out waiting for ${MCL_FEATURE_TOPIC}"
  require_mcl_feature_stream
  wait_for_topic /map1/mapcloud 90 || die "Timed out waiting for /map1/mapcloud"
  wait_for_topic /map1/mapground 90 || die "Timed out waiting for /map1/mapground"
  if [[ "${ROS_DISTRO_VALUE}" == "foxy" ]]; then
    wait_for_container_log_pattern \
      "non-empty /map1/mapground consumption" \
      '\[perception_3d_local\.map\].*Ground.*size: [1-9][0-9]*' 90 || \
      die "Timed out waiting for downstream consumption of non-empty /map1/mapground"
  else
    wait_for_pointcloud_sample /map1/mapground 90 transient_local || \
      die "Timed out waiting for a non-empty /map1/mapground sample"
  fi
  wait_for_topic /map1/planning_ground 90 || die "Timed out waiting for /map1/planning_ground"
  if [[ "${ROS_DISTRO_VALUE}" == "foxy" ]]; then
    wait_for_container_log_pattern \
      "non-empty /map1/planning_ground consumption" \
      '\[perception_3d_global\.map\].*Ground.*size: [1-9][0-9]*' 90 || \
      die "Timed out waiting for downstream consumption of non-empty /map1/planning_ground"
  else
    wait_for_pointcloud_sample /map1/planning_ground 90 transient_local || \
      die "Timed out waiting for a non-empty /map1/planning_ground sample"
  fi
  wait_for_topic /weighted_ground 90 || die "Timed out waiting for /weighted_ground"
  if [[ "${ROS_DISTRO_VALUE}" == "foxy" ]]; then
    wait_for_container_log_pattern \
      "weighted planning-ground publication" \
      '\[global_planner\].*Publish weighted ground point cloud\.' \
      "${STATIC_LAYER_TIMEOUT_SEC_VALUE}" || weighted_ground_timeout_error
  else
    wait_for_pointcloud_sample \
      /weighted_ground "${STATIC_LAYER_TIMEOUT_SEC_VALUE}" transient_local || \
      weighted_ground_timeout_error
  fi
  wait_for_topic /dddmr_go2/safe_cmd_vel 90 || die "Timed out waiting for /dddmr_go2/safe_cmd_vel"
  require_current_observation_stream

  if [[ "${live_mode}" == "true" ]]; then
    # The bare-DDS Go2 endpoints can take longer than the navigation graph to
    # appear in a newly started container's ROS CLI discovery cache. Require
    # the continuously published state topics before enabling Sport output.
    # Do not gate startup on the subscriber-only /api/sport/request topic: the
    # live adapter creates its publisher, while the state topics prove that the
    # Go2 DDS participant is online.
    wait_for_topic /lowstate 60 || die "Timed out waiting for /lowstate"
    check_topic_contract /lowstate unitree_go/msg/LowState 1 0
    wait_for_topic /sportmodestate 60 || die "Timed out waiting for /sportmodestate"
    check_topic_contract /sportmodestate unitree_go/msg/SportModeState 1 0
    start_live_adapter
    sleep 2
    require_lateral_parameter /go2_sport_cmd_vel_adapter_live
    require_sport_axis_contract /go2_sport_cmd_vel_adapter_live
  fi

  if [[ "${STARTUP_PROFILE_VALUE}" == "full" ]]; then
    print_status
  else
    log "Quick profile passed all fail-closed readiness gates; skipping full status inventory."
  fi

  if [[ "${record_mode}" == "true" ]]; then
    run_waypoint_recorder
    stop_nav_containers
    runtime_started="false"
  elif [[ "${multi_mode}" == "true" ]]; then
    while true; do
      wait_for_mission_ready 120
      if arm_multi_mission; then
        break
      fi
    done
    monitor_multi_mission
    stop_nav_containers
    runtime_started="false"
  elif [[ -n "${RUN_SECONDS_VALUE}" ]]; then
    log "RUN_SECONDS=${RUN_SECONDS_VALUE}; stopping after timeout..."
    sleep "${RUN_SECONDS_VALUE}"
    stop_nav_containers
    runtime_started="false"
  else
    log "Started. Stop with: ${SCRIPT_DIR}/run_go2_xt16_navigation_test.sh --stop"
  fi
}

main "$@"
