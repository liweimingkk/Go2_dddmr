#!/usr/bin/env bash
set -euo pipefail

BAG_ROOT="${BAG_ROOT:-${HOME}/bags}"
BAG_PREFIX="${BAG_PREFIX:-go2_xt16_dddmr_mapping}"
STATE_DIR="${STATE_DIR:-${HOME}/.go2_xt16_dddmr_mapping_recorder}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
CYCLONE_SETUP="${CYCLONE_SETUP:-${HOME}/cyclonedds_ws/install/setup.bash}"
GO2_GRAPH_SETUP="${GO2_GRAPH_SETUP:-/unitree/module/graph_pid_ws/install/setup.bash}"
GO2_WS_SETUP="${GO2_WS_SETUP:-${HOME}/go2_workspace/install/setup.bash}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CYCLONEDDS_URI="${CYCLONEDDS_URI:-${HOME}/cyclonedds_ws/cyclonedds.xml}"

# DDDMR/LeGO-LOAM XT16 offline mapping replay contract:
#   Required raw inputs:
#     /lidar_points: Hesai XT16 PointCloud2, frame usually hesai_lidar.
#     /utlidar/robot_odom: Go2 raw odom. Offline replay can standardize this
#       to base_link +X forward with go2_odom_standardizer.
#   Optional inputs are recorded only when their topic type is visible.
#     /dddmr_go2/robot_odom_standard: host-side standardized odom, if live
#       mapping/standardizer is running during capture.
#     /tf_static: useful when corrected static extrinsics are being published.
#     IMU/lowstate/pose topics are diagnostics, not required by LeGO mapping.
REQUIRED_TOPICS=(${REQUIRED_TOPICS:-/lidar_points /utlidar/robot_odom})
OPTIONAL_TOPICS=(${OPTIONAL_TOPICS:-/dddmr_go2/robot_odom_standard /tf_static /lidar_imu /utlidar/imu /utlidar/robot_pose /lowstate /odom_sync_diagnostics /mapping_diagnostics /key_poses /cloud_keypose_6d})
INCLUDE_DYNAMIC_TF="${INCLUDE_DYNAMIC_TF:-false}"
INCLUDE_MOUTH_CLOUD="${INCLUDE_MOUTH_CLOUD:-true}"

PID_FILE="${STATE_DIR}/record.pid"
BAG_FILE="${STATE_DIR}/record.bag"
LOG_FILE="${STATE_DIR}/record.log"
CMD_FILE="${STATE_DIR}/record.cmd"
TOPICS_FILE="${STATE_DIR}/record.topics"

mkdir -p "${STATE_DIR}" "${BAG_ROOT}"

ros_env_prefix() {
  local env_cmd
  env_cmd="source '${ROS_SETUP}'; "
  env_cmd+="if [[ -f '${CYCLONE_SETUP}' ]]; then source '${CYCLONE_SETUP}'; fi; "
  env_cmd+="if [[ -f '${GO2_GRAPH_SETUP}' ]]; then source '${GO2_GRAPH_SETUP}'; fi; "
  env_cmd+="if [[ -f '${GO2_WS_SETUP}' ]]; then source '${GO2_WS_SETUP}'; fi; "
  env_cmd+="export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'; "
  env_cmd+="export CYCLONEDDS_URI='${CYCLONEDDS_URI}'; "
  printf '%s' "${env_cmd}"
}

topic_info() {
  local topic="$1" env_cmd
  env_cmd="$(ros_env_prefix)"
  bash -lc "${env_cmd} timeout 3 ros2 topic info '${topic}'" 2>/dev/null || true
}

topic_exists() {
  local topic="$1" info
  info="$(topic_info "${topic}")"
  printf '%s\n' "${info}" | grep -q '^Type:'
}

topic_has_publisher() {
  local topic="$1" info
  info="$(topic_info "${topic}")"
  printf '%s\n' "${info}" | awk '
    /^Publisher count:/ {
      if ($3 + 0 > 0) found=1
    }
    END { exit found ? 0 : 1 }'
}

optional_topics_configured() {
  printf '%s\n' "${OPTIONAL_TOPICS[@]}"
  [[ "${INCLUDE_DYNAMIC_TF}" == "true" ]] && printf '%s\n' /tf
  [[ "${INCLUDE_MOUTH_CLOUD}" == "true" ]] && printf '%s\n' /utlidar/cloud_base
}

build_record_topics() {
  local topics=() topic
  for topic in "${REQUIRED_TOPICS[@]}"; do
    topics+=("${topic}")
  done

  while IFS= read -r topic; do
    [[ -z "${topic}" ]] && continue
    topic_exists "${topic}" || continue
    topics+=("${topic}")
  done < <(optional_topics_configured)

  printf '%s\n' "${topics[@]}" | awk '!seen[$0]++'
}

print_one_topic() {
  local topic="$1" label="$2" info
  info="$(topic_info "${topic}")"
  if [[ -z "${info}" ]]; then
    echo "  ${topic} (${label}): NOT FOUND"
    return 0
  fi
  printf '  %s (%s): ' "${topic}" "${label}"
  printf '%s\n' "${info}" \
    | awk '
        /^Type:/ {type=$0}
        /^Publisher count:/ {pub=$0}
        /^Subscription count:/ {subs=$0}
        END {
          if (type == "") type="Type: unknown";
          if (pub == "") pub="Publisher count: unknown";
          if (subs == "") subs="Subscription count: unknown";
          print type ", " pub ", " subs
        }'
}

print_topic_preflight() {
  local topic
  echo "Required raw mapping topics:"
  for topic in "${REQUIRED_TOPICS[@]}"; do
    print_one_topic "${topic}" "required"
  done
  echo "Optional topics configured:"
  while IFS= read -r topic; do
    [[ -z "${topic}" ]] && continue
    print_one_topic "${topic}" "optional"
  done < <(optional_topics_configured)
  echo "Topics that will be recorded now:"
  build_record_topics | sed 's/^/  /'
}

require_topic_publishers() {
  local missing=0 topic
  for topic in "${REQUIRED_TOPICS[@]}"; do
    if ! topic_has_publisher "${topic}"; then
      echo "ERROR: required topic ${topic} has no publisher." >&2
      missing=1
    fi
  done
  return "${missing}"
}

wait_for_subscription_log() {
  local deadline
  deadline=$((SECONDS + 8))
  while (( SECONDS < deadline )); do
    if grep -q "All requested topics are subscribed" "${LOG_FILE}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
}

is_running() {
  [[ -f "${PID_FILE}" ]] || return 1
  local pid
  pid="$(cat "${PID_FILE}")"
  [[ -n "${pid}" ]] || return 1
  kill -0 "${pid}" >/dev/null 2>&1
}

bag_path() {
  [[ -f "${BAG_FILE}" ]] && cat "${BAG_FILE}"
}

latest_bag() {
  find "${BAG_ROOT}" -maxdepth 1 -type d -name "${BAG_PREFIX}_*" -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr \
    | awk 'NR==1 {print $2}'
}

start_recording() {
  if is_running; then
    echo "Already recording."
    status
    return 0
  fi

  local now out env_cmd cmd pid topics
  now="$(date +%Y%m%d_%H%M%S)"
  out="${BAG_ROOT}/${BAG_PREFIX}_${now}"
  : >"${LOG_FILE}"

  print_topic_preflight
  require_topic_publishers

  mapfile -t topics < <(build_record_topics)
  printf '%s\n' "${topics[@]}" >"${TOPICS_FILE}"

  env_cmd="$(ros_env_prefix)"
  cmd="${env_cmd}exec ros2 bag record -o '${out}' ${topics[*]}"
  printf '%s\n' "${cmd}" >"${CMD_FILE}"
  printf '%s\n' "${out}" >"${BAG_FILE}"

  setsid bash -lc "${cmd}" >>"${LOG_FILE}" 2>&1 < /dev/null &
  pid="$!"
  printf '%s\n' "${pid}" >"${PID_FILE}"

  sleep 2
  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    echo "Failed to start recorder. Log:"
    sed -n '1,120p' "${LOG_FILE}" || true
    rm -f "${PID_FILE}"
    return 1
  fi

  echo "Recording started."
  echo "PID: ${pid}"
  echo "Bag: ${out}"
  echo "Topics:"
  sed 's/^/  /' "${TOPICS_FILE}"
  echo "Log: ${LOG_FILE}"
  echo "Record command:"
  sed -n '1p' "${CMD_FILE}"
  wait_for_subscription_log
  echo "Log tail:"
  tail -40 "${LOG_FILE}" 2>/dev/null || true
}

stop_recording() {
  if ! is_running; then
    echo "Recorder is not running."
    local latest
    latest="$(latest_bag || true)"
    [[ -n "${latest}" ]] && echo "Latest bag: ${latest}"
    return 0
  fi

  local pid
  pid="$(cat "${PID_FILE}")"
  echo "Stopping recorder PID ${pid} with SIGINT..."
  kill -INT "-${pid}" >/dev/null 2>&1 || kill -INT "${pid}" >/dev/null 2>&1 || true

  for _ in $(seq 1 60); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done

  if kill -0 "${pid}" >/dev/null 2>&1; then
    echo "Recorder did not exit after SIGINT; sending SIGTERM."
    kill -TERM "-${pid}" >/dev/null 2>&1 || kill -TERM "${pid}" >/dev/null 2>&1 || true
  fi

  rm -f "${PID_FILE}"
  echo "Recorder stopped."
  status
}

status() {
  if is_running; then
    local pid out
    pid="$(cat "${PID_FILE}")"
    out="$(bag_path || true)"
    echo "Status: recording"
    echo "PID: ${pid}"
    echo "Bag: ${out}"
    [[ -f "${TOPICS_FILE}" ]] && sed 's/^/  /' "${TOPICS_FILE}"
    [[ -d "${out}" ]] && du -sh "${out}" || true
    echo "Log tail:"
    tail -20 "${LOG_FILE}" 2>/dev/null || true
  else
    echo "Status: stopped"
    local latest
    latest="$(latest_bag || true)"
    if [[ -n "${latest}" ]]; then
      echo "Latest bag: ${latest}"
      du -sh "${latest}" || true
    fi
  fi
}

list_bags() {
  find "${BAG_ROOT}" -maxdepth 1 -type d -name "${BAG_PREFIX}_*" -printf '%TY-%Tm-%Td %TH:%TM %.0T@ %p\n' 2>/dev/null \
    | sort -k3,3nr \
    | sed -n '1,20p'
}

usage() {
  cat <<'USAGE'
Usage: go2_xt16_dddmr_mapping_record_remote.sh <start|stop|status|latest|list|preflight>

Defaults:
  BAG_ROOT=/home/unitree/bags
  BAG_PREFIX=go2_xt16_dddmr_mapping
  STATE_DIR=/home/unitree/.go2_xt16_dddmr_mapping_recorder
  REQUIRED_TOPICS="/lidar_points /utlidar/robot_odom"
  OPTIONAL_TOPICS="/dddmr_go2/robot_odom_standard /tf_static /lidar_imu /utlidar/imu /utlidar/robot_pose /lowstate /odom_sync_diagnostics /mapping_diagnostics /key_poses /cloud_keypose_6d"
  INCLUDE_DYNAMIC_TF=false
  INCLUDE_MOUTH_CLOUD=true

Notes:
  The default records the raw XT16/odom inputs plus /utlidar/cloud_base for
  offline mouth-ground fusion workflows.
  /tf is excluded by default to avoid recording live mapping output transforms.
  Set INCLUDE_DYNAMIC_TF=true only if you intentionally want dynamic /tf.
  Set INCLUDE_MOUTH_CLOUD=false only if you want an XT16-only bag.
USAGE
}

cmd="${1:-status}"
case "${cmd}" in
  start)
    start_recording
    ;;
  stop)
    stop_recording
    ;;
  status)
    status
    ;;
  latest)
    latest_bag
    ;;
  list)
    list_bags
    ;;
  preflight)
    print_topic_preflight
    require_topic_publishers
    ;;
  *)
    usage
    exit 2
    ;;
esac
