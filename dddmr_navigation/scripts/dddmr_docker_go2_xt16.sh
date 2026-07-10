#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/dddmr_docker_go2_xt16.sh <command> [args...]

Commands:
  shell        Start an interactive official DDDMR x64 Docker shell.
  preflight    Read-only check of live Go2 XT16 /lidar_points contract.
  build-lego   Build lego_loam_bor and its dependencies inside Docker.
  mapping      Run no-motion Go2 XT16 LeGO-LOAM mapping inside Docker.
  mapping-bag  Run offline Go2 XT16 LeGO-LOAM mapping from a rosbag2 directory.
  build-navigation
               Build the Go2 XT16 DDDMR navigation test chain inside Docker.
  navigation-dry-run
               Run Go2 XT16 DDDMR navigation with /cmd_vel remapped to dry-run topics.
  navigation-live-source
               Run Go2 XT16 DDDMR navigation/RViz as a velocity source only.
               It publishes /dddmr_go2/dry_run_cmd_vel but never publishes /api/sport/request.
  build-image  Build the official dddmr:x64 Docker image from dddmr_docker.
  build-go2-image
               Build a thin Go2 image layer with CycloneDDS RMW.

Environment:
  DDDMR_IMAGE=dddmr_go2_xt16:x64
  DDDMR_BASE_IMAGE=dddmr:x64
  DDDMR_BAGS_DIR=/home/lin/dddmr_bags
  ROS_DOMAIN_ID=0
  GO2_DDS_IP=192.168.123.18
  GO2_NET_IFACE=<auto>
  DDDMR_BUILD_BASE=.docker_go2_xt16_build
  DDDMR_INSTALL_BASE=.docker_go2_xt16_install
  DDDMR_LOG_BASE=.docker_go2_xt16_log
  DDDMR_DOCKER_NAME=<optional docker container name>
  RVIZ=false
  PUBLISH_STATIC_TF=true
  RUN_SECONDS=<empty for no timeout>
  CONFIG_FILE=<container path for mapping-bag params file>
  DDDMR_MAPPING_DIR=<container output dir prefix for saved map, default under /root/dddmr_bags>
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
BASE_IMAGE="${DDDMR_BASE_IMAGE:-dddmr:x64}"
BAGS_DIR="${DDDMR_BAGS_DIR:-/home/lin/dddmr_bags}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-}"
BUILD_BASE_VALUE="${DDDMR_BUILD_BASE:-.docker_go2_xt16_build}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
LOG_BASE_VALUE="${DDDMR_LOG_BASE:-.docker_go2_xt16_log}"
DOCKER_NAME_VALUE="${DDDMR_DOCKER_NAME:-}"

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

command="$1"
shift

ensure_bags_dir() {
  mkdir -p "${BAGS_DIR}"
}

ensure_image() {
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "Docker image ${IMAGE} not found." >&2
    echo "Run: scripts/dddmr_docker_go2_xt16.sh build-go2-image" >&2
    exit 1
  fi
}

docker_base_args() {
  ensure_bags_dir
  local args=(
    --rm
    --privileged
    --network=host
    --env "DISPLAY=${DISPLAY:-}"
    --env "QT_X11_NO_MITSHM=1"
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
    --env "GO2_DDS_IP=${GO2_DDS_IP_VALUE}"
    --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    --env "DDDMR_BUILD_BASE=${BUILD_BASE_VALUE}"
    --env "DDDMR_INSTALL_BASE=${INSTALL_BASE_VALUE}"
    --env "DDDMR_LOG_BASE=${LOG_BASE_VALUE}"
    --volume "/tmp:/tmp"
    --volume "/dev:/dev"
    --volume "${WS_ROOT}:/root/dddmr_navigation"
    --volume "${BAGS_DIR}:/root/dddmr_bags"
    --volume "${BAGS_DIR}:${BAGS_DIR}"
  )
  if [[ -n "${GO2_NET_IFACE_VALUE}" ]]; then
    args+=(--env "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}")
  fi
  if [[ -n "${DOCKER_NAME_VALUE}" ]]; then
    args+=(--name "${DOCKER_NAME_VALUE}")
  fi
  printf '%s\0' "${args[@]}"
}

run_docker() {
  ensure_image
  local -a base_args
  mapfile -d '' -t base_args < <(docker_base_args)
  docker run "${base_args[@]}" "$@"
}

source_prefix='set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
set -u
cd /root/dddmr_navigation'

case "${command}" in
  shell)
    if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
      xhost +local:docker >/dev/null || true
    fi
    run_docker -it "${IMAGE}" bash
    ;;

  preflight)
    run_docker "${IMAGE}" bash -lc "${source_prefix}
python3 scripts/go2_xt16_lidar_preflight.py \"\$@\"" bash "$@"
    ;;

  build-lego)
    run_docker "${IMAGE}" bash -lc "${source_prefix}
colcon --log-base \"\${DDDMR_LOG_BASE}\" build --symlink-install --packages-up-to lego_loam_bor --build-base \"\${DDDMR_BUILD_BASE}\" --install-base \"\${DDDMR_INSTALL_BASE}\" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3"
    ;;

  build-navigation)
    run_docker "${IMAGE}" bash -lc "${source_prefix}
colcon --log-base \"\${DDDMR_LOG_BASE}\" build --symlink-install --packages-up-to lego_loam_bor dddmr_pg_map_server mcl_3dl global_planner p2p_move_base perception_3d dddmr_beginner_guide dddmr_rviz_default_plugins --build-base \"\${DDDMR_BUILD_BASE}\" --install-base \"\${DDDMR_INSTALL_BASE}\" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3"
    ;;

  mapping)
    rviz="${RVIZ:-false}"
    publish_static_tf="${PUBLISH_STATIC_TF:-true}"
    run_seconds="${RUN_SECONDS:-}"
    launch_cmd="set +u
source \"\${DDDMR_INSTALL_BASE}/setup.bash\"
set -u
ros2 launch lego_loam_bor lego_loam_go2_xt16_live.launch rviz:=${rviz} publish_static_tf:=${publish_static_tf} \"\$@\""
    if [[ -n "${run_seconds}" ]]; then
      run_docker "${IMAGE}" bash -lc "${source_prefix}
timeout -s TERM -k 5s ${run_seconds}s bash -lc '${launch_cmd}' bash \"\$@\"" bash "$@"
    else
      run_docker -it "${IMAGE}" bash -lc "${source_prefix}
${launch_cmd}" bash "$@"
    fi
    ;;

  mapping-bag)
    config_file="${CONFIG_FILE:-${1:-/root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_go2_xt16_20260704_081247.yaml}}"
    mapping_dir_prefix="${DDDMR_MAPPING_DIR:-/root/dddmr_bags/go2_xt16_dddmr_mapping_20260704_081247_map_}"
    run_docker "${IMAGE}" bash -lc "${source_prefix}
set +u
source \"\${DDDMR_INSTALL_BASE}/setup.bash\"
set -u
export DDDMR_MAPPING_DIR='${mapping_dir_prefix}'
echo \"GO2_XT16_BAG_MAPPING_CONFIG=${config_file}\"
echo \"GO2_XT16_BAG_MAPPING_OUTPUT_PREFIX=\${DDDMR_MAPPING_DIR}\"
ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z -0.24 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id base_footprint &
tf_base_footprint_pid=\$!
ros2 run tf2_ros static_transform_publisher --x -0.02557 --y 0.0 --z 0.04232 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id go2_imu &
tf_go2_imu_pid=\$!
ros2 run tf2_ros static_transform_publisher --x 0.1710 --y 0.0 --z 0.0908 --roll 0.0 --pitch 0.0 --yaw 1.57079632679 --frame-id go2_imu --child-frame-id hesai_lidar &
tf_hesai_pid=\$!
cleanup_mapping_bag() {
  kill \${tf_base_footprint_pid:-} \${tf_go2_imu_pid:-} \${tf_hesai_pid:-} 2>/dev/null || true
}
trap cleanup_mapping_bag EXIT
sleep 1.0
ros2 run lego_loam_bor lego_loam_bag --ros-args --params-file '${config_file}' &
mapping_pid=\$!
for i in \$(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -Fxq /bag_reader; then
    break
  fi
  sleep 0.5
done
ros2 topic pub --once /lego_loam_bag_pause std_msgs/msg/Bool '{data: false}' >/tmp/go2_xt16_bag_mapping_resume.log 2>&1 || {
  cat /tmp/go2_xt16_bag_mapping_resume.log >&2 || true
  kill \${mapping_pid} 2>/dev/null || true
  wait \${mapping_pid} 2>/dev/null || true
  exit 1
}
cat /tmp/go2_xt16_bag_mapping_resume.log || true
wait \${mapping_pid}"
    ;;

  navigation-dry-run)
    rviz="${RVIZ:-true}"
    publish_static_tf="${PUBLISH_STATIC_TF:-true}"
    run_seconds="${RUN_SECONDS:-}"
    if [[ "${rviz}" == "true" && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
      xhost +local:docker >/dev/null || true
    fi
    launch_cmd="set +u
source \"\${DDDMR_INSTALL_BASE}/setup.bash\"
set -u
ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch rviz:=${rviz} publish_static_tf:=${publish_static_tf} \"\$@\""
    if [[ -n "${run_seconds}" ]]; then
      run_docker "${IMAGE}" bash -lc "${source_prefix}
timeout -s TERM -k 5s ${run_seconds}s bash -lc '${launch_cmd}' bash \"\$@\"" bash "$@"
    else
      run_docker -it "${IMAGE}" bash -lc "${source_prefix}
${launch_cmd}" bash "$@"
    fi
    ;;

  navigation-live-source)
    rviz="${RVIZ:-true}"
    publish_static_tf="${PUBLISH_STATIC_TF:-true}"
    run_seconds="${RUN_SECONDS:-}"
    if [[ "${rviz}" == "true" && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
      xhost +local:docker >/dev/null || true
    fi
    launch_cmd="set +u
source \"\${DDDMR_INSTALL_BASE}/setup.bash\"
set -u
ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch rviz:=${rviz} publish_static_tf:=${publish_static_tf} start_sport_dry_run_adapter:=false start_go2_sport_adapter:=false \"\$@\""
    if [[ -n "${run_seconds}" ]]; then
      run_docker "${IMAGE}" bash -lc "${source_prefix}
timeout -s TERM -k 5s ${run_seconds}s bash -lc '${launch_cmd}' bash \"\$@\"" bash "$@"
    else
      run_docker "${IMAGE}" bash -lc "${source_prefix}
${launch_cmd}" bash "$@"
    fi
    ;;

  build-image)
    docker build --network host -t "${BASE_IMAGE}" -f "${WS_ROOT}/dddmr_docker/docker_file/Dockerfile_x64" "${WS_ROOT}/dddmr_docker/docker_file"
    ;;

  build-go2-image)
    if ! docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1; then
      echo "Base image ${BASE_IMAGE} not found. Building it first." >&2
      docker build --network host -t "${BASE_IMAGE}" -f "${WS_ROOT}/dddmr_docker/docker_file/Dockerfile_x64" "${WS_ROOT}/dddmr_docker/docker_file"
    fi
    docker build --network host -t "${IMAGE}" -f "${WS_ROOT}/dddmr_docker/docker_file/Dockerfile_go2_xt16" "${WS_ROOT}/dddmr_docker/docker_file"
    ;;

  -h|--help|help)
    usage
    ;;

  *)
    echo "Unknown command: ${command}" >&2
    usage >&2
    exit 2
    ;;
esac
