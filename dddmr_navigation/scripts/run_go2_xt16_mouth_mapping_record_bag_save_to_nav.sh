#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_mouth_mapping_record_bag_save_to_nav.sh

Run the supervised Go2 XT16 + mouth-LiDAR mapping/save workflow while recording
the three raw inputs required to replay the same mapping run:

  /lidar_points
  /utlidar/cloud_base
  /utlidar/robot_odom

The script first performs a read-only XT16 readiness check, then starts the
recorder and delegates all mapping, save, and navigation-config update work to
run_go2_xt16_mouth_mapping_save_to_nav.sh.  When mapping finishes or is
interrupted, the recorder receives SIGINT so rosbag2 flushes its cache and
writes metadata.yaml before the recorder container is removed.

The mapping controls from run_go2_xt16_mouth_mapping_save_to_nav.sh are passed
through unchanged. In particular, set MAPPING_SECONDS=N for automatic save, or
leave it empty to press Enter in the mapping workflow.

Recording overrides:
  DDDMR_BAGS_DIR=...                 Shared host output directory. Default: ../bags.
  MAPPING_BAG_PREFIX=...             Default: go2_xt16_mouth_mapping_input.
  MAPPING_BAG_DOCKER_NAME=...        Explicit recorder container name.
  RECORD_XT16_TOPIC=...              Default: XT16_TOPIC or /lidar_points.
  RECORD_MOUTH_CLOUD_TOPIC=...       Default: MOUTH_CLOUD_TOPIC or /utlidar/cloud_base.
  RECORD_ODOM_TOPIC=...              Default: ODOM_TOPIC or /utlidar/robot_odom.
  BAG_STOP_TIMEOUT_SEC=30            Graceful recorder shutdown deadline.
  SKIP_LIDAR_PREFLIGHT=true          Skip the read-only live readiness check.

Safety:
  This wrapper publishes no robot motion command. Its delegated mapping script
  retains the navigation-stop safety checks before mapping starts.
EOF
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]] &&
   [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
MAPPING_SCRIPT="${SCRIPT_DIR}/run_go2_xt16_mouth_mapping_save_to_nav.sh"

IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-enp46s0}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
MAPPING_BAG_PREFIX="${MAPPING_BAG_PREFIX:-go2_xt16_mouth_mapping_input}"
RECORD_XT16_TOPIC="${RECORD_XT16_TOPIC:-${XT16_TOPIC:-/lidar_points}}"
RECORD_MOUTH_CLOUD_TOPIC="${RECORD_MOUTH_CLOUD_TOPIC:-${MOUTH_CLOUD_TOPIC:-/utlidar/cloud_base}}"
RECORD_ODOM_TOPIC="${RECORD_ODOM_TOPIC:-${ODOM_TOPIC:-/utlidar/robot_odom}}"
BAG_STOP_TIMEOUT_SEC="${BAG_STOP_TIMEOUT_SEC:-30}"
SKIP_LIDAR_PREFLIGHT="${SKIP_LIDAR_PREFLIGHT:-false}"

stamp="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="${MAPPING_BAG_PREFIX}_${stamp}"
HOST_BAG_DIR="${BAGS_DIR}/${BAG_NAME}"
CONTAINER_BAG_DIR="/root/dddmr_bags/${BAG_NAME}"
RECORDER_CONTAINER="${MAPPING_BAG_DOCKER_NAME:-go2_xt16_mouth_mapping_bag_${stamp}}"
RECORDER_STARTED=false

log() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "Required file not found: $1"
}

require_docker_image() {
  docker image inspect "${IMAGE}" >/dev/null 2>&1 || die "Docker image ${IMAGE} not found."
}

require_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]] || die "Expected a positive integer, got: $1"
}

run_live_readiness_check() {
  case "${SKIP_LIDAR_PREFLIGHT}" in
    false)
      ;;
    true)
      log 'Skipping live XT16 readiness check because SKIP_LIDAR_PREFLIGHT=true.'
      return 0
      ;;
    *)
      die 'SKIP_LIDAR_PREFLIGHT must be true or false.'
      ;;
  esac

  log "Read-only readiness: ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE} GO2_NET_IFACE=${GO2_NET_IFACE_VALUE} RMW=${RMW_IMPLEMENTATION_VALUE}"
  docker run --rm \
    --network=host \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}" \
    -e "GO2_DDS_IP=${GO2_DDS_IP_VALUE}" \
    -e "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}" \
    -v "${WS_ROOT}:/root/dddmr_navigation:ro" \
    "${IMAGE}" \
    bash -lc 'set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
set -u
python3 /root/dddmr_navigation/scripts/go2_xt16_lidar_preflight.py --samples 3 --timeout 15
if ros2 topic list | grep -Fxq /tf; then
  echo "TF_TOPIC=present"
else
  echo "TF_TOPIC=absent (mapping launch may provide configured static TF)"
fi
if ros2 topic list | grep -Fxq /tf_static; then
  echo "TF_STATIC_TOPIC=present"
else
  echo "TF_STATIC_TOPIC=absent (mapping launch may provide configured static TF)"
fi'
}

start_recorder() {
  mkdir -p "${BAGS_DIR}"
  if docker ps -a --format '{{.Names}}' | grep -Fxq "${RECORDER_CONTAINER}"; then
    die "Recorder container already exists: ${RECORDER_CONTAINER}"
  fi

  log "Starting mapping-input recorder: ${RECORDER_CONTAINER}"
  docker run -d \
    --name "${RECORDER_CONTAINER}" \
    --network=host \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}" \
    -e "GO2_DDS_IP=${GO2_DDS_IP_VALUE}" \
    -e "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}" \
    -v "${WS_ROOT}:/root/dddmr_navigation:ro" \
    -v "${BAGS_DIR}:/root/dddmr_bags" \
    "${IMAGE}" \
    bash -lc "set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
set -u
exec ros2 bag record -o '${CONTAINER_BAG_DIR}' \\
  '${RECORD_XT16_TOPIC}' \\
  '${RECORD_MOUTH_CLOUD_TOPIC}' \\
  '${RECORD_ODOM_TOPIC}'" >/dev/null

  RECORDER_STARTED=true
  sleep 1
  docker ps --filter "name=^/${RECORDER_CONTAINER}$" --format '{{.Status}}' | \
    grep -q '^Up' || {
      docker logs "${RECORDER_CONTAINER}" >&2 || true
      die "Recorder container failed to stay running: ${RECORDER_CONTAINER}"
    }
  log "Recording raw mapping inputs to: ${HOST_BAG_DIR}"
}

stop_recorder() {
  [[ "${RECORDER_STARTED}" == true ]] || return 0
  RECORDER_STARTED=false

  local wait_file="/tmp/${RECORDER_CONTAINER}.wait"
  if docker ps --format '{{.Names}}' | grep -Fxq "${RECORDER_CONTAINER}"; then
    log "Stopping mapping-input recorder with SIGINT: ${RECORDER_CONTAINER}"
    docker kill --signal=SIGINT "${RECORDER_CONTAINER}" >/dev/null || true
  fi

  if docker ps -a --format '{{.Names}}' | grep -Fxq "${RECORDER_CONTAINER}"; then
    if timeout "${BAG_STOP_TIMEOUT_SEC}" docker wait "${RECORDER_CONTAINER}" >"${wait_file}"; then
      log "Recorder exit code: $(<"${wait_file}")"
    else
      log "Recorder did not stop within ${BAG_STOP_TIMEOUT_SEC}s; forcing container removal."
      docker rm -f "${RECORDER_CONTAINER}" >/dev/null || true
    fi
  fi
  docker rm "${RECORDER_CONTAINER}" >/dev/null 2>&1 || true

  if [[ -f "${HOST_BAG_DIR}/metadata.yaml" ]]; then
    log "Saved mapping-input bag: ${HOST_BAG_DIR}"
  else
    log "Recorder stopped before metadata.yaml was created: ${HOST_BAG_DIR}"
  fi
}

on_exit() {
  local rc=$?
  trap - EXIT
  stop_recorder
  exit "${rc}"
}

main() {
  require_file "${MAPPING_SCRIPT}"
  require_docker_image
  require_positive_integer "${BAG_STOP_TIMEOUT_SEC}"
  run_live_readiness_check
  start_recorder

  export DDDMR_BAGS_DIR="${BAGS_DIR}"
  log 'Delegating mapping, save, and navigation-config update to the existing mapping workflow.'
  "${MAPPING_SCRIPT}" "$@"
}

trap on_exit EXIT
main "$@"
