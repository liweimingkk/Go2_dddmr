#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE="${DDDMR_IMAGE:-dddmr_go2_xt16:x64}"
BAGS_DIR="${DDDMR_BAGS_DIR:-${WS_ROOT}/../bags}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
GO2_DDS_IP_VALUE="${GO2_DDS_IP:-192.168.123.18}"
GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-}"
if [[ -z "${GO2_NET_IFACE_VALUE}" ]] && command -v ip >/dev/null 2>&1; then
  GO2_NET_IFACE_VALUE="$(ip route get "${GO2_DDS_IP_VALUE}" 2>/dev/null | awk '
    {for (i=1; i<=NF; ++i) if ($i == "dev" && i < NF) {print $(i+1); exit}}' || true
  )"
fi
BUILD_BASE_VALUE="${DDDMR_BUILD_BASE:-.docker_go2_xt16_build}"
INSTALL_BASE_VALUE="${DDDMR_INSTALL_BASE:-.docker_go2_xt16_install}"
LOG_BASE_VALUE="${DDDMR_LOG_BASE:-.docker_go2_xt16_log}"

RVIZ_VALUE="${RVIZ:-true}"
PUBLISH_STATIC_TF_VALUE="${PUBLISH_STATIC_TF:-true}"
RUN_SECONDS_VALUE="${RUN_SECONDS:-}"
MOUTH_MAX_TIME_DIFF_VALUE="${MOUTH_MAX_TIME_DIFF:-0.03}"
MOUTH_SYNC_MODE_VALUE="${MOUTH_SYNC_MODE:-receipt_time}"
MOUTH_TIME_OFFSET_SEC_VALUE="${MOUTH_TIME_OFFSET_SEC:-0.0}"
ODOM_TIME_OFFSET_SEC_VALUE="${ODOM_TIME_OFFSET_SEC:-}"

is_number() {
  [[ "$1" =~ ^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$ ]]
}

is_nonnegative_number() {
  is_number "$1" && awk -v value="$1" 'BEGIN {exit !(value >= 0)}'
}

is_positive_number() {
  is_number "$1" && awk -v value="$1" 'BEGIN {exit !(value > 0)}'
}

is_nonnegative_number "${MOUTH_MAX_TIME_DIFF_VALUE}" || {
  echo "MOUTH_MAX_TIME_DIFF must be finite and nonnegative." >&2
  exit 2
}
case "${MOUTH_SYNC_MODE_VALUE}" in
  receipt_time)
    is_positive_number "${MOUTH_MAX_TIME_DIFF_VALUE}" || {
      echo "MOUTH_MAX_TIME_DIFF must be positive in receipt_time mode." >&2
      exit 2
    }
    is_number "${MOUTH_TIME_OFFSET_SEC_VALUE}" || {
      echo "MOUTH_TIME_OFFSET_SEC must be numeric." >&2
      exit 2
    }
    awk -v value="${MOUTH_TIME_OFFSET_SEC_VALUE}" \
      'BEGIN {if (value < 0) value=-value; exit !(value <= 1e-9)}' || {
      echo "MOUTH_TIME_OFFSET_SEC must be 0 in receipt_time mode." >&2
      exit 2
    }
    MOUTH_TIME_OFFSET_SEC_VALUE="0.0"
    ;;
  header_offset)
    is_number "${MOUTH_TIME_OFFSET_SEC_VALUE}" || {
      echo "MOUTH_TIME_OFFSET_SEC must be numeric in header_offset mode." >&2
      exit 2
    }
    ;;
  *)
    echo "MOUTH_SYNC_MODE must be receipt_time or header_offset." >&2
    exit 2
    ;;
esac

if [[ "${MOUTH_TOPIC:-/utlidar/cloud_base}" != "/utlidar/cloud_base" ]]; then
  echo "Refusing to run: this validation path is fixed to /utlidar/cloud_base." >&2
  exit 2
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "Docker image ${IMAGE} not found." >&2
  echo "Run: scripts/dddmr_docker_go2_xt16.sh build-go2-image" >&2
  exit 1
fi

if [[ -z "${ODOM_TIME_OFFSET_SEC_VALUE}" ]]; then
  echo "No explicit ODOM_TIME_OFFSET_SEC; measuring two consistent live windows..."
  measurement_env=(
    env
    "DDDMR_IMAGE=${IMAGE}"
    "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
    "GO2_DDS_IP=${GO2_DDS_IP_VALUE}"
  )
  if [[ -n "${GO2_NET_IFACE_VALUE}" ]]; then
    measurement_env+=("GO2_NET_IFACE=${GO2_NET_IFACE_VALUE}")
  fi
  set +e
  measurement_report="$("${measurement_env[@]}" \
    "${SCRIPT_DIR}/run_go2_xt16_mouth_mapping_save_to_nav.sh" \
    --measure-odom-only 2>&1)"
  measurement_rc=$?
  set -e
  printf '%s\n' "${measurement_report}"
  if [[ "${measurement_rc}" -ne 0 ]]; then
    echo "Could not confirm a live odom/XT16 offset; validation aborted." >&2
    exit "${measurement_rc}"
  fi
  ODOM_TIME_OFFSET_SEC_VALUE="$(awk -F= \
    '$1 == "CONFIRMED_ODOM_TIME_OFFSET_SEC" {print $2}' \
    <<<"${measurement_report}" | tail -n 1)"
fi
is_number "${ODOM_TIME_OFFSET_SEC_VALUE}" || {
  echo "ODOM_TIME_OFFSET_SEC is missing or invalid." >&2
  exit 2
}

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
  --env "MOUTH_SYNC_MODE=${MOUTH_SYNC_MODE_VALUE}"
  --env "MOUTH_TIME_OFFSET_SEC=${MOUTH_TIME_OFFSET_SEC_VALUE}"
  --env "ODOM_TIME_OFFSET_SEC=${ODOM_TIME_OFFSET_SEC_VALUE}"
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
echo "MOUTH_VALIDATION_TIMING mouth_sync_mode=${MOUTH_SYNC_MODE} mouth_max_time_diff=${MOUTH_MAX_TIME_DIFF} mouth_time_offset_sec=${MOUTH_TIME_OFFSET_SEC}"
echo "ODOM_VALIDATION_TIMING odom_time_offset_sec=${ODOM_TIME_OFFSET_SEC}"

cmd=(
  ros2 launch lego_loam_bor lego_loam_go2_xt16_mouth.launch
  "rviz:=${RVIZ}"
  "rviz_config:=/root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/rviz/go2_xt16_mouth_validation.rviz"
  "publish_static_tf:=${PUBLISH_STATIC_TF}"
  "mouth_cloud_topic:=/utlidar/cloud_base"
  "mouth_filter_frame:=base_link"
  "mouth_sync_mode:=${MOUTH_SYNC_MODE}"
  "mouth_max_time_diff:=${MOUTH_MAX_TIME_DIFF}"
  "mouth_time_offset_sec:=${MOUTH_TIME_OFFSET_SEC}"
  "odom_time_offset_sec:=${ODOM_TIME_OFFSET_SEC}"
)

if [[ -n "${RUN_SECONDS}" ]]; then
  set +e
  timeout -s TERM -k 5s "${RUN_SECONDS}s" "${cmd[@]}"
  rc=$?
  set -e
  if [[ "${rc}" -eq 124 ]]; then
    echo "MOUTH_VALIDATION_COMPLETE bounded_run_seconds=${RUN_SECONDS}"
    exit 0
  fi
  exit "${rc}"
else
  exec "${cmd[@]}"
fi'

exec docker run "${docker_args[@]}" "${IMAGE}" bash -lc "${inner}"
