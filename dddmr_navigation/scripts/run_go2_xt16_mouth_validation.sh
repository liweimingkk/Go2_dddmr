#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
BAGS_DIR="${DDDMR_BAGS_DIR:-/home/lin/dddmr_bags}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-}"
BUILD_BASE_VALUE="${DDDMR_BUILD_BASE:-.docker_go2_xt16_build}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
LOG_BASE_VALUE="${DDDMR_LOG_BASE:-.docker_go2_xt16_log}"

RVIZ_VALUE="${RVIZ:-true}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
RUN_SECONDS_VALUE="${RUN_SECONDS:-}"
MOUTH_MAX_TIME_DIFF_VALUE="${MOUTH_MAX_TIME_DIFF:-4.0}"
MOUTH_TIME_OFFSET_SEC_VALUE="${MOUTH_TIME_OFFSET_SEC:-0.0}"

if [[ "${MOUTH_TOPIC:-/utlidar/cloud_base}" != "/utlidar/cloud_base" ]]; then
  echo "Refusing to run: this validation path is fixed to /utlidar/cloud_base." >&2
  exit 2
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "Docker image ${IMAGE} not found." >&2
  echo "Run: scripts/dddmr_docker_go2_xt16.sh build-go2-image" >&2
  exit 1
fi

mkdir -p "${BAGS_DIR}"

if [[ "${RVIZ_VALUE}" == "true" && -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null || true
fi

docker_args=(
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
  --env "RVIZ=${RVIZ_VALUE}"
  --env "PUBLISH_STATIC_TF=${PUBLISH_STATIC_TF_VALUE}"
  --env "RUN_SECONDS=${RUN_SECONDS_VALUE}"
  --env "MOUTH_MAX_TIME_DIFF=${MOUTH_MAX_TIME_DIFF_VALUE}"
  --env "MOUTH_TIME_OFFSET_SEC=${MOUTH_TIME_OFFSET_SEC_VALUE}"
  --volume "/tmp:/tmp"
  --volume "/dev:/dev"
  --volume "${WS_ROOT}:/root/dddmr_navigation"
  --volume "${BAGS_DIR}:/root/dddmr_bags"
  --volume "${BAGS_DIR}:${BAGS_DIR}"
)

if [[ -n "${GO2_NET_IFACE_VALUE}" ]]; then
  docker_args+=(--env "GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}")
fi

inner='set -eo pipefail
cd /root/dddmr_navigation
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source "${DDDMR_INSTALL_BASE}/setup.bash"
set -u

echo "MOUTH_VALIDATION_CONTRACT mouth_cloud_topic=/utlidar/cloud_base mouth_frame_override=<empty> mouth_filter_frame=base_link"
echo "MOUTH_VALIDATION_TIMING mouth_max_time_diff=${MOUTH_MAX_TIME_DIFF} mouth_time_offset_sec=${MOUTH_TIME_OFFSET_SEC}"

cmd=(
  ros2 launch lego_loam_bor lego_loam_go2_xt16_mouth.launch
  "rviz:=${RVIZ}"
  "rviz_config:=/root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/rviz/go2_xt16_mouth_validation.rviz"
  "publish_static_tf:=${PUBLISH_STATIC_TF}"
  "mouth_cloud_topic:=/utlidar/cloud_base"
  "mouth_filter_frame:=base_link"
  "mouth_max_time_diff:=${MOUTH_MAX_TIME_DIFF}"
  "mouth_time_offset_sec:=${MOUTH_TIME_OFFSET_SEC}"
)

if [[ -n "${RUN_SECONDS}" ]]; then
  timeout -s TERM -k 5s "${RUN_SECONDS}s" "${cmd[@]}"
else
  exec "${cmd[@]}"
fi'

exec docker run "${docker_args[@]}" "${IMAGE}" bash -lc "${inner}"
