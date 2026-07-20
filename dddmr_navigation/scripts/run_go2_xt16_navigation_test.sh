#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_navigation_test.sh [--dry-run|--live|--stop]

Starts Go2 XT16 navigation with the map configured in:
  src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml

Modes:
  --dry-run  Start navigation and RViz only. No real /api/sport/request output.
             This is the default.
  --live     Start navigation, RViz, and the live Sport adapter. This can move
             the Go2 after an RViz goal is sent. Requires onsite supervision
             and GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV.
  --stop     Stop running go2_xt16 navigation containers. If a live adapter is
             running, it is stopped first so it can publish StopMove.

Common environment overrides:
  MAX_X=0.50
  MAX_Y=0.0
  MAX_YAW=0.50
  RVIZ=true
  PUBLISH_STATIC_TF=true
  STOP_EXISTING=false      Set true to stop old nav containers before starting.
  RUN_SECONDS=             Live defaults to 300; maximum is 1800 seconds.
  GO2_NET_IFACE=enp46s0
  GO2_DDS_IP=192.168.123.18
  DDDMR_BAGS_DIR=../bags
  NAV_CONTAINER_NAME=...

Examples:
  scripts/run_go2_xt16_navigation_test.sh --dry-run
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
    scripts/run_go2_xt16_navigation_test.sh --live
  GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV STOP_EXISTING=true \
    scripts/run_go2_xt16_navigation_test.sh --live
  scripts/run_go2_xt16_navigation_test.sh --stop
EOF
}

mode="dry-run"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      mode="dry-run"
      shift
      ;;
    --live)
      mode="live"
      shift
      ;;
    --stop)
      mode="stop"
      shift
      ;;
    -h|--help|help)
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"

IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-enp46s0}"
BUILD_BASE_VALUE="${DDDMR_BUILD_BASE:-.docker_go2_xt16_build}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
LOG_BASE_VALUE="${DDDMR_LOG_BASE:-.docker_go2_xt16_log}"
RVIZ_VALUE="${RVIZ:-true}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
MAX_X_VALUE="${MAX_X:-0.50}"
MAX_YAW_VALUE="${MAX_YAW:-0.50}"
MAX_Y_VALUE="${MAX_Y:-0.0}"
RUN_SECONDS_VALUE="${RUN_SECONDS:-}"
STOP_EXISTING_VALUE="${STOP_EXISTING:-false}"
LIVE_CONFIRM_PHRASE="I_AM_SUPERVISING_GO2_NAV"
CONTAINER_NAME="${NAV_CONTAINER_NAME:-go2_xt16_nav_${mode//-/_}_x${MAX_X_VALUE//./}_yaw${MAX_YAW_VALUE//./}_$(date +%Y%m%d_%H%M%S)}"
RUN_LOG_DIR="${RUN_LOG_DIR:-${WS_ROOT}/run_logs}"
ADAPTER_LOG_CONTAINER="/root/dddmr_navigation/run_logs/${CONTAINER_NAME}_adapter.log"
ADAPTER_LOG_HOST="${RUN_LOG_DIR}/${CONTAINER_NAME}_adapter.log"
runtime_started="false"

if [[ "${mode}" == "live" && -z "${RUN_SECONDS_VALUE}" ]]; then
  RUN_SECONDS_VALUE="300"
fi

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

validate_live_request() {
  [[ "${GO2_NAV_LIVE_CONFIRM:-}" == "${LIVE_CONFIRM_PHRASE}" ]] || \
    die "Live mode requires GO2_NAV_LIVE_CONFIRM=${LIVE_CONFIRM_PHRASE}."
  [[ "${RUN_SECONDS_VALUE}" =~ ^[1-9][0-9]*$ ]] || \
    die "Live RUN_SECONDS must be a positive integer."
  (( RUN_SECONDS_VALUE <= 1800 )) || \
    die "Live RUN_SECONDS must not exceed 1800 seconds."
}

cleanup_live_runtime() {
  local status=$?
  if [[ "${mode}" == "live" && "${runtime_started}" == "true" ]]; then
    log "Stopping supervised live runtime during exit cleanup..."
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
    log "No go2_xt16 navigation containers are running."
    return 0
  fi

  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    log "Stopping live adapter in ${name}, if present..."
    stop_adapter_in_container "${name}"
    log "Stopping navigation container: ${name}"
    docker stop -t 5 "${name}" >/dev/null || true
    mkdir -p "${RUN_LOG_DIR}"
    local docker_log="${RUN_LOG_DIR}/${name}_docker.log"
    if docker logs "${name}" >"${docker_log}" 2>&1; then
      log "Saved navigation log: ${docker_log}"
    else
      log "WARNING: failed to save navigation log for ${name}"
    fi
    docker rm "${name}" >/dev/null 2>&1 || true
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
      die "Navigation container is already running. Use --stop first or STOP_EXISTING=true."
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
source /opt/ros/humble/setup.bash
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

start_container() {
  mkdir -p "${BAGS_DIR}" "${RUN_LOG_DIR}"

  if [[ "${RVIZ_VALUE}" == "true" && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null || true
  fi

  log "Starting navigation container: ${CONTAINER_NAME}"
  docker run -d \
    --name "${CONTAINER_NAME}" \
    --label "dddmr.go2_xt16_navigation=true" \
    --network=host \
    --privileged \
    -e "DISPLAY=${DISPLAY:-:0}" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}" \
    -e "GO2_DDS_IP=${GO2_DDS_IP_VALUE}" \
    -e "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e "DDDMR_BUILD_BASE=${BUILD_BASE_VALUE}" \
    -e "DDDMR_INSTALL_BASE=${INSTALL_BASE_VALUE}" \
    -e "DDDMR_LOG_BASE=${LOG_BASE_VALUE}" \
    -v "${BAGS_DIR}:${BAGS_DIR}" \
    -v "/tmp:/tmp" \
    -v "/dev:/dev" \
    -v "${WS_ROOT}:/root/dddmr_navigation" \
    -v "${BAGS_DIR}:/root/dddmr_bags" \
    "${IMAGE}" \
    bash -lc "set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
cd /root/dddmr_navigation
exec ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch \
  rviz:=${RVIZ_VALUE} \
  publish_static_tf:=${PUBLISH_STATIC_TF_VALUE} \
  go2_nav_cmd_gate_max_x:=${MAX_X_VALUE} \
  go2_nav_cmd_gate_max_y:=${MAX_Y_VALUE} \
  sport_dry_run_max_x:=${MAX_X_VALUE} \
  sport_dry_run_max_y:=${MAX_Y_VALUE} \
  go2_sport_max_x:=${MAX_X_VALUE} \
  go2_sport_max_y:=${MAX_Y_VALUE} \
  go2_nav_cmd_gate_max_yaw:=${MAX_YAW_VALUE} \
  sport_dry_run_max_yaw:=${MAX_YAW_VALUE} \
  go2_sport_max_yaw:=${MAX_YAW_VALUE}" >/dev/null

  printf '%s\n' "${CONTAINER_NAME}" >/tmp/go2_xt16_nav_current_name.txt
}

start_live_adapter() {
  log "Starting live Sport adapter: max_x=${MAX_X_VALUE}, max_yaw=${MAX_YAW_VALUE}"
  docker exec "${CONTAINER_NAME}" bash -lc "set -e
test -f /root/dddmr_navigation/.unitree_msg_ws/install/setup.bash || {
  echo 'Missing /root/dddmr_navigation/.unitree_msg_ws/install/setup.bash; build/copy unitree_api first.' >&2
  exit 1
}
mkdir -p /root/dddmr_navigation/run_logs
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
source /root/dddmr_navigation/.unitree_msg_ws/install/setup.bash
nohup python3 /root/dddmr_navigation/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py \
  --ros-args \
  -r __node:=go2_sport_cmd_vel_adapter_live \
  -p cmd_vel_topic:=/dddmr_go2/safe_cmd_vel \
  -p request_topic:=/api/sport/request \
  -p enable_sport_output:=true \
  -p allow_real_request_topic:=true \
  -p max_x:=${MAX_X_VALUE} \
  -p max_y:=${MAX_Y_VALUE} \
  -p max_yaw:=${MAX_YAW_VALUE} \
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
  docker_ros "ros2 param get /go2_nav_cmd_gate max_x; ros2 param get /go2_nav_cmd_gate max_yaw; ros2 param get /trajectory_generators differential_drive_rotate_shortest_angle.rotation_speed; ros2 param get /trajectory_generators differential_drive_rotate_inplace.rotation_speed" || true

  log "Map and command topics:"
  docker_ros "ros2 topic list | sort | grep -E 'map1/mapcloud|map1/mapground|sub_map|safe_cmd_vel|dry_run_cmd_vel|lidar_points|utlidar/robot_odom' || true" || true

  if [[ "${mode}" == "live" ]]; then
    log "Live adapter log: ${ADAPTER_LOG_HOST}"
    if [[ -f "${ADAPTER_LOG_HOST}" ]]; then
      tail -n 20 "${ADAPTER_LOG_HOST}" || true
    fi
    log "Current safe_cmd_vel sample:"
    docker_ros "timeout 4 ros2 topic echo /dddmr_go2/safe_cmd_vel geometry_msgs/msg/Twist --once" || true
  fi
}

main() {
  if [[ "${mode}" == "stop" ]]; then
    stop_nav_containers
    return 0
  fi

  if [[ "${mode}" == "live" ]]; then
    validate_live_request
    trap cleanup_live_runtime EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
  fi

  require_docker_image
  assert_clean_runtime
  start_container
  runtime_started="true"

  log "Starting navigation and Go2 DDS readiness checks..."
  wait_for_node /go2_nav_cmd_gate 90 || die "Timed out waiting for /go2_nav_cmd_gate"
  wait_for_topic /map1/mapcloud 90 || die "Timed out waiting for /map1/mapcloud"
  wait_for_topic /map1/mapground 90 || die "Timed out waiting for /map1/mapground"
  wait_for_topic /dddmr_go2/safe_cmd_vel 90 || die "Timed out waiting for /dddmr_go2/safe_cmd_vel"

  if [[ "${mode}" == "live" ]]; then
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
  fi

  print_status

  if [[ -n "${RUN_SECONDS_VALUE}" ]]; then
    log "RUN_SECONDS=${RUN_SECONDS_VALUE}; stopping after timeout..."
    sleep "${RUN_SECONDS_VALUE}"
    stop_nav_containers
    runtime_started="false"
  else
    log "Started. Stop with: ${SCRIPT_DIR}/run_go2_xt16_navigation_test.sh --stop"
  fi
}

main "$@"
