#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh

Starts Go2 XT16 + mouth lidar mapping, saves the current pose graph, copies it to
the host bags directory, and updates the navigation config used by
go2_xt16_navigation.launch.

Common environment overrides:
  MAPPING_SECONDS=120        Map for N seconds, then save. If empty, wait for Enter.
  RVIZ=true                  Open mouth/ground fusion RViz while mapping.
  MAP_RVIZ=true              Open a second RViz showing the accumulated map.
  STOP_AFTER_SAVE=true       Stop the mapping container after saving.
  CLEAN_STALE_MAPPING=true   Remove stale containers left by this script first.
  STOP_NAV_BEFORE_MAPPING=true
                             Stop running go2_xt16 navigation before mapping.
  MAPPING_CONTAINER=name     Save from an already-running mapping container.
  DDDMR_BAGS_DIR=...         Host bags directory. Default: ../bags.
  GO2_NET_IFACE=enp46s0      Network interface used for Go2 DDS.
  MOUTH_MAX_TIME_DIFF=0.15
  MOUTH_TIME_OFFSET_SEC=0.03424
  NAV_CONFIG=...             Navigation YAML to update.

Safety:
  This script stops active go2_xt16 navigation/live adapter runtime before
  mapping. Set STOP_NAV_BEFORE_MAPPING=false to refuse instead, or
  ALLOW_NAV_RUNNING=true to skip this safety check.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
  exit 0
fi

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
MAP_RVIZ_VALUE="${MAP_RVIZ:-true}"
MAP_RVIZ_CONFIG="${MAP_RVIZ_CONFIG:-/root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/rviz/go2_xt16_map_result.rviz}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
MAPPING_SECONDS_VALUE="${MAPPING_SECONDS:-}"
MOUTH_MAX_TIME_DIFF_VALUE="${MOUTH_MAX_TIME_DIFF:-0.15}"
MOUTH_TIME_OFFSET_SEC_VALUE="${MOUTH_TIME_OFFSET_SEC:-0.03424}"
NAV_CONFIG="${NAV_CONFIG:-${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml}"
NAV_MAP_ROOT="${NAV_MAP_ROOT:-/root/dddmr_bags}"
MAP_PREFIX="${MAP_PREFIX:-go2_xt16_mouth_mapping}"
UPDATE_NAV_CONFIG="${UPDATE_NAV_CONFIG:-true}"
ALLOW_NAV_RUNNING="${ALLOW_NAV_RUNNING:-false}"
STOP_NAV_BEFORE_MAPPING="${STOP_NAV_BEFORE_MAPPING:-true}"
CLEAN_STALE_MAPPING="${CLEAN_STALE_MAPPING:-true}"

MAPPING_CONTAINER_VALUE="${MAPPING_CONTAINER:-}"
OWN_CONTAINER=false
if [[ -n "${MAPPING_CONTAINER_VALUE}" ]]; then
  CONTAINER_NAME="${MAPPING_CONTAINER_VALUE}"
  STOP_AFTER_SAVE="${STOP_AFTER_SAVE:-false}"
else
  CONTAINER_NAME="${DDDMR_DOCKER_NAME:-go2_xt16_mouth_mapping_save_$(date +%Y%m%d_%H%M%S)}"
  STOP_AFTER_SAVE="${STOP_AFTER_SAVE:-true}"
  OWN_CONTAINER=true
fi

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

nav_container_names() {
  docker ps --format '{{.Names}}' | grep -E 'go2_xt16_nav|go2_xt16_navigation' || true
}

host_navigation_processes() {
  ps -eo pid,args | awk '
    $0 ~ /run_go2_xt16_navigation_supervised_live[.]sh/ ||
    $0 ~ /run_go2_xt16_navigation_test[.]sh/ ||
    $0 ~ /go2_sport_cmd_vel_adapter[.]py/ ||
    $0 ~ /go2_xt16_navigation[.]launch/ ||
    $0 ~ /ros2 topic pub \/api\/sport\/request/ {
      if ($0 !~ /run_go2_xt16_mouth_mapping_save_to_nav[.]sh/ && $0 !~ /awk/) {
        print
      }
    }
  '
}

host_navigation_pids() {
  host_navigation_processes | awk '{print $1}'
}

wait_until_pids_exit() {
  local timeout_sec="$1"
  shift
  local deadline=$((SECONDS + timeout_sec))
  local pid

  while (( SECONDS < deadline )); do
    local any_alive="false"
    for pid in "$@"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        any_alive="true"
        break
      fi
    done
    [[ "${any_alive}" == "false" ]] && return 0
    sleep 1
  done
  return 1
}

stop_adapter_in_nav_container() {
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

stop_live_navigation_before_mapping() {
  [[ "${ALLOW_NAV_RUNNING}" == "true" ]] && return 0
  [[ "${STOP_NAV_BEFORE_MAPPING}" == "true" ]] || return 0

  local pids=()
  mapfile -t pids < <(host_navigation_pids)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    log "Stopping host navigation/live process(es): ${pids[*]}"
    kill -INT "${pids[@]}" >/dev/null 2>&1 || true
    if ! wait_until_pids_exit 20 "${pids[@]}"; then
      log "Host navigation process(es) still running; sending SIGTERM: ${pids[*]}"
      kill -TERM "${pids[@]}" >/dev/null 2>&1 || true
      wait_until_pids_exit 10 "${pids[@]}" || true
    fi
  fi

  local nav_names=()
  local name
  mapfile -t nav_names < <(nav_container_names)
  if [[ "${#nav_names[@]}" -gt 0 ]]; then
    log "Stopping go2_xt16 navigation container(s): ${nav_names[*]}"
    for name in "${nav_names[@]}"; do
      [[ -n "${name}" ]] || continue
      stop_adapter_in_nav_container "${name}"
      docker stop -t 5 "${name}" >/dev/null || true
      docker rm "${name}" >/dev/null 2>&1 || true
    done
  fi
}

check_no_live_navigation() {
  [[ "${ALLOW_NAV_RUNNING}" == "true" ]] && return 0

  local containers processes
  containers="$(nav_container_names)"
  if [[ -n "${containers}" ]]; then
    echo "${containers}" >&2
    die "A go2_xt16 navigation container is still running. Stop navigation before mapping."
  fi

  processes="$(host_navigation_processes)"
  if [[ -n "${processes}" ]]; then
    echo "${processes}" >&2
    die "A navigation/live sport publisher process is still running. Stop it before mapping."
  fi
}

cleanup_stale_mapping_runtime() {
  [[ "${OWN_CONTAINER}" == "true" ]] || return 0
  [[ "${CLEAN_STALE_MAPPING}" == "true" ]] || return 0

  local stale_names=()
  local name
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if [[ "${name}" == "${CONTAINER_NAME}" || "${name}" =~ ^go2_xt16_mouth_mapping_save(_|$) ]]; then
      stale_names+=("${name}")
    fi
  done < <(docker ps -a --format '{{.Names}}')

  [[ "${#stale_names[@]}" -gt 0 ]] || return 0

  log "Cleaning stale mapping container(s): ${stale_names[*]}"
  for name in "${stale_names[@]}"; do
    docker rm -f "${name}" >/dev/null 2>&1 || true
  done
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

wait_for_service() {
  local service="$1"
  local timeout_sec="${2:-60}"
  local i
  for i in $(seq 1 "${timeout_sec}"); do
    if docker_ros "ros2 service list | grep -Fxq '${service}'" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local i
  for i in $(seq 1 "${timeout_sec}"); do
    if docker_ros "ros2 topic list | grep -Fxq '${topic}'" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

newest_tmp_map_dir() {
  docker exec "${CONTAINER_NAME}" bash -lc \
    "find /tmp -maxdepth 1 -type d -name '20??_*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | awk '{print \$2}'"
}

start_map_result_rviz() {
  [[ "${MAP_RVIZ_VALUE}" == "true" ]] || return 0

  log "Starting map-result RViz: ${MAP_RVIZ_CONFIG}"
  docker exec "${CONTAINER_NAME}" bash -lc "set -e
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
if ros2 node list 2>/dev/null | grep -Fxq /rviz_map_result; then
  echo 'rviz_map_result already running'
  exit 0
fi
mkdir -p /root/dddmr_navigation/run_logs
nohup rviz2 -d '${MAP_RVIZ_CONFIG}' --ros-args -r __node:=rviz_map_result \
  > '/root/dddmr_navigation/run_logs/${CONTAINER_NAME}_rviz_map.log' 2>&1 &
echo \$!"
}

update_nav_config() {
  local config_file="$1"
  local pose_graph_dir="$2"

  python3 - "$config_file" "$pose_graph_dir" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
pose_graph_dir = sys.argv[2]
lines = path.read_text().splitlines()

in_map1 = False
replaced = False
out = []
for line in lines:
    stripped = line.strip()
    if line and not line.startswith((" ", "\t")) and stripped.endswith(":"):
        in_map1 = stripped == "map1:"
    if in_map1 and stripped.startswith("pose_graph_dir:"):
        indent = line[: len(line) - len(line.lstrip())]
        out.append(f'{indent}pose_graph_dir: "{pose_graph_dir}"')
        replaced = True
    else:
        out.append(line)

if not replaced:
    raise SystemExit("pose_graph_dir under map1 was not found")

path.write_text("\n".join(out) + "\n")
PY
}

print_summary() {
  local host_dir="$1"
  log "Saved map directory: ${host_dir}"
  for f in map.pcd ground.pcd poses.pcd edges.pcd; do
    if [[ -f "${host_dir}/${f}" ]]; then
      printf '  %-10s %s\n' "${f}" "$(grep -m1 '^POINTS' "${host_dir}/${f}" || true)"
    else
      printf '  %-10s MISSING\n' "${f}"
    fi
  done
  if [[ -d "${host_dir}/pcd" ]]; then
    printf '  %-10s %s files\n' "pcd/" "$(find "${host_dir}/pcd" -maxdepth 1 -type f | wc -l)"
  fi
}

start_mapping_container() {
  mkdir -p "${BAGS_DIR}"

  if [[ ( "${RVIZ_VALUE}" == "true" || "${MAP_RVIZ_VALUE}" == "true" ) && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null || true
  fi

  log "Starting mapping container: ${CONTAINER_NAME}"
  docker run -d \
    --name "${CONTAINER_NAME}" \
    --privileged \
    --network=host \
    -e "DISPLAY=${DISPLAY:-:0}" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}" \
    -e "GO2_DDS_IP=${GO2_DDS_IP_VALUE}" \
    -e "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e "DDDMR_BUILD_BASE=${BUILD_BASE_VALUE}" \
    -e "DDDMR_INSTALL_BASE=${INSTALL_BASE_VALUE}" \
    -e "DDDMR_LOG_BASE=${LOG_BASE_VALUE}" \
    -v "/tmp:/tmp" \
    -v "/dev:/dev" \
    -v "${WS_ROOT}:/root/dddmr_navigation" \
    -v "${BAGS_DIR}:${NAV_MAP_ROOT}" \
    -v "${BAGS_DIR}:${BAGS_DIR}" \
    "${IMAGE}" \
    bash -lc "set -eo pipefail
cd /root/dddmr_navigation
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/${INSTALL_BASE_VALUE}/setup.bash
set -u
echo 'MOUTH_MAPPING_CONTRACT mouth_cloud_topic=/utlidar/cloud_base mouth_filter_frame=base_link'
echo 'MOUTH_MAPPING_TIMING mouth_max_time_diff=${MOUTH_MAX_TIME_DIFF_VALUE} mouth_time_offset_sec=${MOUTH_TIME_OFFSET_SEC_VALUE}'
exec ros2 launch lego_loam_bor lego_loam_go2_xt16_mouth.launch \
  rviz:=${RVIZ_VALUE} \
  rviz_config:=/root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/rviz/go2_xt16_mouth_validation.rviz \
  publish_static_tf:=${PUBLISH_STATIC_TF_VALUE} \
  mouth_cloud_topic:=/utlidar/cloud_base \
  mouth_filter_frame:=base_link \
  mouth_max_time_diff:=${MOUTH_MAX_TIME_DIFF_VALUE} \
  mouth_time_offset_sec:=${MOUTH_TIME_OFFSET_SEC_VALUE}" >/dev/null
}

main() {
  require_file "${NAV_CONFIG}"
  require_docker_image
  stop_live_navigation_before_mapping
  check_no_live_navigation
  cleanup_stale_mapping_runtime

  if [[ "${OWN_CONTAINER}" == "true" ]]; then
    if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
      die "Container already exists: ${CONTAINER_NAME}"
    fi
    start_mapping_container
  else
    docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || die "Mapping container is not running: ${CONTAINER_NAME}"
    log "Using existing mapping container: ${CONTAINER_NAME}"
  fi

  log "Waiting for /save_mapped_point_cloud service..."
  wait_for_service /save_mapped_point_cloud 90 || die "Timed out waiting for /save_mapped_point_cloud"
  wait_for_topic /lego_loam_map 90 || die "Timed out waiting for /lego_loam_map"
  start_map_result_rviz

  if [[ -n "${MAPPING_SECONDS_VALUE}" ]]; then
    log "Mapping for ${MAPPING_SECONDS_VALUE}s before save..."
    sleep "${MAPPING_SECONDS_VALUE}"
  else
    log "Mapping is running. Press Enter to save and update navigation config."
    read -r _
  fi

  local before_dir
  before_dir="$(newest_tmp_map_dir || true)"

  log "Calling /save_mapped_point_cloud..."
  docker_ros "timeout 120 ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty '{}'" >/tmp/go2_xt16_mapping_save_service.log
  cat /tmp/go2_xt16_mapping_save_service.log

  local saved_tmp_dir
  saved_tmp_dir="$(newest_tmp_map_dir)"
  [[ -n "${saved_tmp_dir}" ]] || die "No saved /tmp map directory was found"
  if [[ -n "${before_dir}" && "${saved_tmp_dir}" == "${before_dir}" ]]; then
    die "No new map directory was created. The map may not have enough keyframes yet."
  fi

  local saved_name local_ts map_name container_dir host_dir
  saved_name="$(basename "${saved_tmp_dir}")"
  local_ts="$(date +%Y%m%d_%H%M%S)"
  map_name="${MAP_PREFIX}_${local_ts}_map_${saved_name}"
  container_dir="${NAV_MAP_ROOT}/${map_name}"
  host_dir="${BAGS_DIR}/${map_name}"

  log "Copying ${saved_tmp_dir} -> ${container_dir}"
  docker exec "${CONTAINER_NAME}" bash -lc "set -euo pipefail; rm -rf '${container_dir}'; cp -a '${saved_tmp_dir}' '${container_dir}'"

  for required in map.pcd ground.pcd poses.pcd edges.pcd pcd; do
    [[ -e "${host_dir}/${required}" ]] || die "Saved map is missing ${required}: ${host_dir}"
  done

  if [[ "${UPDATE_NAV_CONFIG}" == "true" ]]; then
    log "Updating navigation config: ${NAV_CONFIG}"
    update_nav_config "${NAV_CONFIG}" "${container_dir}"
  else
    log "Skipping navigation config update because UPDATE_NAV_CONFIG=false"
  fi

  print_summary "${host_dir}"
  log "Navigation pose_graph_dir: ${container_dir}"

  if [[ "${STOP_AFTER_SAVE}" == "true" ]]; then
    log "Stopping mapping container: ${CONTAINER_NAME}"
    docker stop -t 5 "${CONTAINER_NAME}" >/dev/null || true
    docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true
  else
    log "Mapping container left running: ${CONTAINER_NAME}"
  fi
}

main "$@"
