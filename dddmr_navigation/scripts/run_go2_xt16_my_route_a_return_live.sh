#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_go2_xt16_my_route_a_return_live.sh [--live|--prepare-only]

Interactive, onsite-supervised regression for:
  my_route_a start -> recorded-route endpoint -> P2P return to route start.

Modes:
  --live          Default. Run all no-motion gates, a short supervised Sport
                  probe, launch the 0.30 m/s / 0.40 rad/s live runtime, wait for
                  health, ask for final confirmation, and send the mission.
  --prepare-only  Build and run static, no-motion, and read-only sensor checks.
                  Never creates real Sport output or starts the live runtime.

Ctrl-C in --live mode asks the existing live supervisor to publish StopMove
and clean up the Docker navigation runtime and host Sport adapter.

Environment:
  GO2_NET_IFACE=<auto-detected robot interface>
  GO2_SETUP=<workspace>/.unitree_msg_ws/install/setup.bash
  DDDMR_BAGS_DIR=<repo-parent>/bags
  RVIZ=true
  GO2_MY_ROUTE_A_LIVE_TIMEOUT_SEC=360
EOF
}

mode="live"
if (( $# > 1 )); then
  usage >&2
  exit 2
fi
case "${1:---live}" in
  --live) mode="live" ;;
  --prepare-only) mode="prepare-only" ;;
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
BAGS_DIR="${DDDMR_BAGS_DIR:-${REPO_ROOT}/bags}"
MAP_NAME="go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36"
MAP_DIR="${BAGS_DIR}/${MAP_NAME}"
ROUTE_DIR="${BAGS_DIR}/routes"
ROUTE_ID="my_route_a"
NAV_CONFIG="${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml"
MAP_CONFIG="${WS_ROOT}/src/dddmr_beginner_guide/config/go2_xt16_my_route_a_return_test.yaml"
TEST_RUNNER="${SCRIPT_DIR}/run_go2_xt16_my_route_a_return_test.sh"
DOCKER_WRAPPER="${SCRIPT_DIR}/dddmr_docker_go2_xt16.sh"
SPORT_PROBE="${SCRIPT_DIR}/run_go2_sport_adapter_supervised_probe.sh"
MISSION_SUPERVISOR="${SCRIPT_DIR}/run_go2_xt16_outdoor_indoor_supervised.sh"
STOP_HELPER="${SCRIPT_DIR}/restart_go2_xt16_navigation_param_test.sh"
GO2_SETUP_VALUE="${GO2_SETUP:-${WS_ROOT}/.unitree_msg_ws/install/setup.bash}"
LIVE_TIMEOUT_SEC="${GO2_MY_ROUTE_A_LIVE_TIMEOUT_SEC:-360}"
RVIZ_VALUE="${RVIZ:-true}"

RETURN_X="0.514610589"
RETURN_Y="0.150227532"
RETURN_Z="-0.210476711"
RETURN_YAW="-1.741948169"
MAX_X="0.30"
MAX_Y="0.0"
MAX_YAW="0.40"
PROBE_MAX_YAW="0.35"
MOTION_DECISIONS="d_controlling,d_align_heading,d_align_goal_heading"
FIRST_CONFIRM="I_AM_SUPERVISING_MY_ROUTE_A_RETURN"
START_CONFIRM="START_MY_ROUTE_A_RETURN"

live_pid=""
live_log=""
probe_log=""
probe_summary=""
yaw_preview_log=""
yaw_preview_summary=""
nav_container=""
cleanup_started="false"

info() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

detect_go2_interface() {
  ip route get 192.168.123.18 2>/dev/null | awk '
    {
      for (i = 1; i <= NF; ++i) {
        if ($i == "dev" && (i + 1) <= NF) {
          print $(i + 1)
          exit
        }
      }
    }
  '
}

GO2_NET_IFACE_VALUE="${GO2_NET_IFACE:-$(detect_go2_interface)}"

prompt_exact() {
  local prompt="$1"
  local expected="$2"
  local answer
  printf '\n%s\n' "${prompt}"
  printf '输入 %s 后回车：' "${expected}"
  IFS= read -r answer
  [[ "${answer}" == "${expected}" ]] || die "现场确认不匹配，未启动运动"
}

wait_for_process_exit() {
  local pid="$1"
  local attempts="${2:-100}"
  local index
  for ((index = 0; index < attempts; ++index)); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

cleanup() {
  local status=$?
  if [[ "${cleanup_started}" == "true" ]]; then
    return
  fi
  cleanup_started="true"
  trap - EXIT INT TERM

  if [[ -n "${live_pid}" ]] && kill -0 "${live_pid}" >/dev/null 2>&1; then
    info "请求 live supervisor 停止并发送 StopMove"
    kill -TERM "${live_pid}" >/dev/null 2>&1 || true
    if ! wait_for_process_exit "${live_pid}" 150; then
      info "supervisor 未及时退出，调用仓库 stop-only 清理"
      "${STOP_HELPER}" --stop-only || true
    fi
    wait "${live_pid}" >/dev/null 2>&1 || true
  fi

  if [[ -n "${nav_container}" ]] && \
      docker ps --format '{{.Names}}' 2>/dev/null | grep -Fxq "${nav_container}"; then
    info "发现残留导航容器，调用 stop-only 清理"
    "${STOP_HELPER}" --stop-only || true
  fi

  [[ -z "${live_log}" ]] || info "live 日志：${live_log}"
  [[ -z "${probe_log}" ]] || info "Sport 探测日志：${probe_log}"
  [[ -z "${yaw_preview_log}" ]] || \
    info "${MAX_YAW} rad/s 隔离预览日志：${yaw_preview_log}"
  exit "${status}"
}

on_signal() {
  info "收到停止信号"
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

validate_local_inputs() {
  local path
  for path in \
    "${TEST_RUNNER}" \
    "${DOCKER_WRAPPER}" \
    "${SPORT_PROBE}" \
    "${MISSION_SUPERVISOR}" \
    "${STOP_HELPER}"; do
    [[ -x "${path}" ]] || die "缺少可执行脚本：${path}"
  done
  [[ -f "${GO2_SETUP_VALUE}" ]] || die "缺少 Go2 ROS 环境：${GO2_SETUP_VALUE}"
  [[ -f "${MAP_DIR}/poses.pcd" ]] || die "缺少 my_route_a 地图：${MAP_DIR}"
  [[ -f "${ROUTE_DIR}/${ROUTE_ID}.json" ]] || die "缺少路线：${ROUTE_ID}"
  [[ -f "${NAV_CONFIG}" ]] || die "缺少导航配置：${NAV_CONFIG}"
  [[ -f "${MAP_CONFIG}" ]] || die "缺少专用地图配置：${MAP_CONFIG}"
  [[ -n "${GO2_NET_IFACE_VALUE}" ]] || die "无法自动识别 Go2 网卡"
  ip link show "${GO2_NET_IFACE_VALUE}" >/dev/null 2>&1 || \
    die "Go2 网卡不存在：${GO2_NET_IFACE_VALUE}"
  [[ "${LIVE_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || \
    die "GO2_MY_ROUTE_A_LIVE_TIMEOUT_SEC 必须是正整数"
  case "${RVIZ_VALUE}" in
    true|false) ;;
    *) die "RVIZ 必须是 true 或 false" ;;
  esac
}

run_yaw_limit_preview() {
  local adapter_log preview_rc
  yaw_preview_log="$(mktemp /tmp/go2_my_route_a_yaw_preview.XXXXXX.log)"
  info "在隔离话题预览 ${MAX_YAW} rad/s，不连接真实 Sport 输出"
  set +e
  env \
    GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    GO2_SETUP="${GO2_SETUP_VALUE}" \
    GO2_SPORT_CMD_TOPIC=/dddmr_go2/my_route_a_yaw_preview_cmd_vel \
    GO2_DECISION_TOPIC=/dddmr_go2/my_route_a_yaw_preview_decision \
    GO2_SPORT_PROBE_X=0.0 \
    GO2_SPORT_PROBE_Y=0.0 \
    GO2_SPORT_PROBE_YAW="${MAX_YAW}" \
    GO2_SPORT_PROBE_DURATION=0.3 \
    GO2_SPORT_PROBE_DECISION=d_align_heading \
    GO2_SPORT_MAX_X=0.10 \
    GO2_SPORT_MAX_Y=0.0 \
    GO2_SPORT_MAX_YAW="${MAX_YAW}" \
    GO2_REQUIRE_MOTION_DECISION=true \
    GO2_MOTION_ALLOWED_DECISIONS="${MOTION_DECISIONS}" \
    GO2_ENABLE_YAW_ARC_SHIM=false \
    GO2_YAW_ARC_SHIM_MODE=off \
    GO2_SPORT_SKIP_LIVE_TOPIC_CHECK=true \
    "${SPORT_PROBE}" --dry-run | tee "${yaw_preview_log}"
  preview_rc="${PIPESTATUS[0]}"
  set -e
  (( preview_rc == 0 )) || die "${MAX_YAW} rad/s 隔离预览失败"

  yaw_preview_summary="$(awk -F= '$1 == "SUMMARY_LOG" {print $2}' \
    "${yaw_preview_log}" | tail -n 1)"
  [[ -n "${yaw_preview_summary}" && -f "${yaw_preview_summary}" ]] || \
    die "旋转上限预览未生成有效 SUMMARY_LOG"
  grep -Fxq 'MODE=dry-run' "${yaw_preview_summary}" || \
    die "旋转上限预览模式不正确"
  grep -Fxq "MAX_YAW=${MAX_YAW}" "${yaw_preview_summary}" || \
    die "旋转上限预览未应用 ${MAX_YAW} rad/s"
  grep -Fxq 'RESULT=GO2_SPORT_ADAPTER_dry_run_COMPLETE' \
    "${yaw_preview_summary}" || die "旋转上限预览未完成"

  adapter_log="$(awk -F= '$1 == "ADAPTER_LOG" {print $2}' \
    "${yaw_preview_summary}" | tail -n 1)"
  [[ -n "${adapter_log}" && -f "${adapter_log}" ]] || \
    die "旋转上限预览缺少 adapter 日志"
  grep -Fq 'SPORT OUTPUT DISABLED:' "${adapter_log}" || \
    die "旋转上限预览未禁用 Sport 输出"
  grep -Fq 'transformed_sport={"x":0.0,"y":0.0,"z":0.4}' "${adapter_log}" || \
    die "adapter 未接受 ${MAX_YAW} rad/s 旋转命令"
  info "${MAX_YAW} rad/s 隔离预览通过"
}

run_no_motion_gates() {
  info "Docker 构建导航工作区"
  GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    "${DOCKER_WRAPPER}" build-navigation

  info "验证 my_route_a、统一地图指纹和返回起点"
  "${TEST_RUNNER}" --check

  info "运行网络隔离的完整零运动 dry-run"
  "${TEST_RUNNER}" --dry-run

  run_yaw_limit_preview

  info "读取 3 帧真实 XT16 点云，不发布运动命令"
  GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    "${DOCKER_WRAPPER}" preflight --samples 3 --timeout 10
}

run_supervised_sport_probe() {
  local probe_rc
  probe_log="$(mktemp /tmp/go2_my_route_a_sport_probe.XXXXXX.log)"
  info "开始 0.05 m/s、0.6 s 的现场监督 Sport 探测"
  set +e
  env \
    GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    GO2_SETUP="${GO2_SETUP_VALUE}" \
    GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
    GO2_SPORT_PROBE_X=0.05 \
    GO2_SPORT_PROBE_Y=0.0 \
    GO2_SPORT_PROBE_YAW=0.0 \
    GO2_SPORT_PROBE_DURATION=0.6 \
    GO2_SPORT_PROBE_DECISION=d_controlling \
    GO2_SPORT_MAX_X=0.10 \
    GO2_SPORT_MAX_Y=0.0 \
    GO2_SPORT_MAX_YAW="${PROBE_MAX_YAW}" \
    GO2_REQUIRE_MOTION_DECISION=true \
    GO2_MOTION_ALLOWED_DECISIONS="${MOTION_DECISIONS}" \
    GO2_ENABLE_YAW_ARC_SHIM=false \
    GO2_YAW_ARC_SHIM_MODE=off \
    "${SPORT_PROBE}" --live | tee "${probe_log}"
  probe_rc="${PIPESTATUS[0]}"
  set -e
  (( probe_rc == 0 )) || die "Sport 探测失败，未启动任务"

  probe_summary="$(awk -F= '$1 == "SUMMARY_LOG" {print $2}' "${probe_log}" | tail -n 1)"
  [[ -n "${probe_summary}" && -f "${probe_summary}" ]] || \
    die "Sport 探测未生成有效 SUMMARY_LOG"
  info "Sport 探测凭据：${probe_summary}"
}

start_live_runtime() {
  local probe_summary="$1"
  local deadline
  live_log="$(mktemp /tmp/go2_my_route_a_live.XXXXXX.log)"
  info "启动 ${MAX_X} m/s、${MAX_YAW} rad/s、${LIVE_TIMEOUT_SEC} s 硬上限的 live 运行时"

  env \
    GO2_NET_IFACE="${GO2_NET_IFACE_VALUE}" \
    GO2_SETUP="${GO2_SETUP_VALUE}" \
    GO2_OUTDOOR_INDOOR_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_OUTDOOR_INDOOR_MISSION \
    GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
    GO2_SPORT_PROBE_SUMMARY="${probe_summary}" \
    MISSION_MAP_DIR="${MAP_DIR}" \
    MISSION_ROUTE_DIR="${ROUTE_DIR}" \
    MISSION_ROUTE_ID="${ROUTE_ID}" \
    NAV_CONFIG_FILE="${NAV_CONFIG}" \
    MISSION_MAP_CONFIG_FILE="${MAP_CONFIG}" \
    GO2_SPORT_MAX_X="${MAX_X}" \
    GO2_SPORT_MAX_Y="${MAX_Y}" \
    GO2_SPORT_MAX_YAW="${MAX_YAW}" \
    GO2_REQUIRE_MOTION_DECISION=true \
    GO2_MOTION_ALLOWED_DECISIONS="${MOTION_DECISIONS}" \
    GO2_ENABLE_YAW_ARC_SHIM=false \
    GO2_YAW_ARC_SHIM_MODE=off \
    RVIZ="${RVIZ_VALUE}" \
    PUBLISH_STATIC_TF=true \
    RUN_SECONDS="${LIVE_TIMEOUT_SEC}" \
    "${MISSION_SUPERVISOR}" --live > >(tee "${live_log}") 2>&1 &
  live_pid="$!"

  deadline=$((SECONDS + 120))
  while (( SECONDS < deadline )); do
    nav_container="$(awk -F= '$1 == "DOCKER_NAME" {print $2}' "${live_log}" | tail -n 1)"
    if grep -Fq 'RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING' "${live_log}" && \
        [[ -n "${nav_container}" ]]; then
      info "live 运行时已启动：${nav_container}"
      return 0
    fi
    if ! kill -0 "${live_pid}" >/dev/null 2>&1; then
      wait "${live_pid}" || true
      tail -n 100 "${live_log}" >&2 || true
      die "live supervisor 在就绪前退出"
    fi
    sleep 1
  done
  tail -n 100 "${live_log}" >&2 || true
  die "等待 live 运行时就绪超时"
}

check_live_readiness() {
  info "检查 TRACKING、HEALTHY、IDLE、零速度、障碍点云、里程计和起点位置"
  docker exec -i "${nav_container}" bash -lc '
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/.docker_go2_xt16_install/setup.bash
set -u
python3 -
' <<'PY'
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

START_X = 0.514610589
START_Y = 0.150227532
START_Z = -0.210476711


class Readiness(Node):
    def __init__(self):
        super().__init__("my_route_a_interactive_readiness")
        self.localization = ""
        self.health = ""
        self.phase = ""
        self.safe_cmd = None
        self.received = {}
        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/localization_status", self.on_localization, 10)
        self.create_subscription(String, "/localization_health", self.on_health, 10)
        self.create_subscription(
            String, "/outdoor_indoor_mission/status", self.on_mission, transient
        )
        self.create_subscription(Twist, "/dddmr_go2/safe_cmd_vel", self.on_cmd, 10)
        self.create_subscription(
            PointCloud2,
            "/perception_3d_local/lidar/current_observation",
            self.on_observation,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/dddmr_go2/robot_odom_standard", self.on_odom, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def mark(self, name):
        self.received[name] = time.monotonic()

    def on_localization(self, message):
        self.localization = message.data.strip().upper()
        self.mark("localization")

    def on_health(self, message):
        self.health = message.data.strip().upper()
        self.mark("health")

    def on_mission(self, message):
        try:
            self.phase = str(json.loads(message.data).get("phase", "")).upper()
        except (TypeError, ValueError):
            self.phase = "MALFORMED"
        self.mark("mission")

    def on_cmd(self, message):
        self.safe_cmd = message
        self.mark("cmd")

    def on_observation(self, _message):
        self.mark("observation")

    def on_odom(self, _message):
        self.mark("odom")

    def fresh(self, name, limit):
        stamp = self.received.get(name)
        return stamp is not None and 0.0 <= time.monotonic() - stamp <= limit


def command_is_zero(message):
    if message is None:
        return False
    values = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(math.isfinite(value) and abs(value) <= 1.0e-6 for value in values)


rclpy.init()
node = Readiness()
deadline = time.monotonic() + 60.0
last_tf_error = "no transform received"
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready = (
            node.localization == "TRACKING"
            and node.health == "HEALTHY"
            and node.phase == "IDLE"
            and command_is_zero(node.safe_cmd)
            and node.fresh("localization", 1.0)
            and node.fresh("health", 1.0)
            and node.fresh("cmd", 1.0)
            and node.fresh("observation", 1.0)
            and node.fresh("odom", 1.0)
        )
        if not ready:
            continue
        try:
            transform = node.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception as exc:  # tf2 exception types vary across ROS 2 patch releases.
            last_tf_error = str(exc)
            continue
        position = transform.transform.translation
        xy_error = math.hypot(position.x - START_X, position.y - START_Y)
        z_error = abs(position.z - START_Z)
        if xy_error > 0.60 or z_error > 0.35:
            last_tf_error = (
                f"route-start envelope failed: xy_error={xy_error:.3f}m "
                f"z_error={z_error:.3f}m"
            )
            continue
        print("LOCALIZATION=TRACKING")
        print("LOCALIZATION_HEALTH=HEALTHY")
        print("MISSION_PHASE=IDLE")
        print("SAFE_CMD_ZERO=PASS")
        print("LOCAL_OBSERVATION_FRESH=PASS")
        print("ODOMETRY_FRESH=PASS")
        print(
            "MAP_BASE_XYZ=%.3f %.3f %.3f" %
            (position.x, position.y, position.z)
        )
        print("ROUTE_START_XY_ERROR=%.3f" % xy_error)
        print("RESULT=MY_ROUTE_A_LIVE_READINESS_PASS")
        raise SystemExit(0)
    raise SystemExit(
        "live readiness failed: "
        f"localization={node.localization or '<none>'} "
        f"health={node.health or '<none>'} phase={node.phase or '<none>'} "
        f"safe_zero={command_is_zero(node.safe_cmd)} "
        f"fresh_observation={node.fresh('observation', 1.0)} "
        f"fresh_odom={node.fresh('odom', 1.0)} tf={last_tf_error}"
    )
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
}

send_mission() {
  local mission_id mission_rc
  mission_id="my-route-a-return-live-$(date +%Y%m%d-%H%M%S)"
  info "发送任务 ${mission_id}"
  set +e
  docker exec -it "${nav_container}" bash -lc "
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh
source /root/dddmr_navigation/.docker_go2_xt16_install/setup.bash
set -u
ros2 run dddmr_route_navigation outdoor_indoor_mission_client.py \\
  '${mission_id}' '${ROUTE_ID}' \\
  '${RETURN_X}' '${RETURN_Y}' '${RETURN_Z}' '${RETURN_YAW}'
"
  mission_rc=$?
  set -e
  (( mission_rc == 0 )) || die "任务未完成；正在执行 StopMove 清理"
  info "RESULT=MY_ROUTE_A_RETURN_LIVE_PASS"
}

validate_local_inputs

if [[ "${mode}" == "live" ]]; then
  [[ -t 0 && -t 1 ]] || die "--live 必须在交互式终端中运行"
fi

info "Go2 网卡：${GO2_NET_IFACE_VALUE}"
info "路线：${ROUTE_ID}，前向上限：${MAX_X} m/s，旋转上限：${MAX_YAW} rad/s，live 上限：${LIVE_TIMEOUT_SEC} s"
run_no_motion_gates

if [[ "${mode}" == "prepare-only" ]]; then
  info "RESULT=MY_ROUTE_A_RETURN_PREPARE_ONLY_PASS"
  exit 0
fi

cat <<EOF

即将执行真实运动：
  1. 以 0.05 m/s 做 0.6 s Sport 通道探测并停车；
  2. 以前向最高 ${MAX_X} m/s、旋转最高 ${MAX_YAW} rad/s 沿 my_route_a 到终点；
  3. 停稳后切换 P2P，返回记录起点；
  4. 任一健康门失败或 Ctrl-C 都会停止并清理。

请确认现场路线净空、机器狗已站稳、急停可用且有人全程监督。
EOF
prompt_exact "确认允许短探测和本次受监督 live 任务。" "${FIRST_CONFIRM}"

run_supervised_sport_probe
start_live_runtime "${probe_summary}"
check_live_readiness

prompt_exact \
  "所有 live 健康门已通过。再次观察现场，确认现在开始往返。" \
  "${START_CONFIRM}"
send_mission
