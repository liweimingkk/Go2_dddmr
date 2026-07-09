# Go2 XT16 DDDMR Live Navigation 问题分析与解决方案

## 0. 当前结论

本包的核心问题不是建图、定位或规划本身，而是**实时导航运行边界放错了**：

```text
DDDMR 导航/RViz 当前稳定运行在 Docker 内；
Go2 Sport API 适配器当前只能可靠运行在 Host 的 Unitree ROS 2 环境里；
现有 live navigation 脚本却试图在 Host 上启动完整 DDDMR 导航。
```

因此，不建议继续修 Host 全量 DDDMR 导航构建。最小安全方案是：

```text
Docker 内：只运行 DDDMR navigation + RViz，继续把 /cmd_vel remap 到 /dddmr_go2/dry_run_cmd_vel。
Host 上：只运行 Go2 Sport adapter，订阅 /dddmr_go2/dry_run_cmd_vel，并发布 /api/sport/request。
```

推荐运行结构：

```text
Docker: DDDMR navigation/RViz
  p2p_move_base /cmd_vel
    -> /dddmr_go2/dry_run_cmd_vel

Host: Go2 Sport adapter
  /dddmr_go2/dry_run_cmd_vel
    -> clamp / watchdog / StopMove
    -> /api/sport/request
```

状态判断：

```text
Sport live probe: PASS
Docker navigation build: PASS
Host full navigation build: FAIL / 不建议继续走
Docker navigation + Host Sport adapter 架构: 推荐
真实 RViz click goal live: 需要按本文修改后再做
```

---

## 1. 审核依据

本次包内关键信息：

```text
README_GPT55PRO.md
files/scripts/dddmr_docker_go2_xt16.sh
files/scripts/run_go2_xt16_navigation_supervised_live.sh
files/scripts/run_go2_sport_adapter_supervised_probe.sh
files/scripts/check_go2_xt16_sport_live_readiness.sh
files/src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
files/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
files/src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py
logs/live_probe_success/*
logs/host_build_attempts/*
logs/docker_build/build_navigation_current.log
```

已确认事实：

```text
1. Docker navigation build 成功：17 packages finished。
2. Host full navigation build 失败：先是 symlink-install 残留目录，后是缺 ackermann_msgs。
3. live Sport probe 第二次成功：echo 中有 Move api_id=1008 和 StopMove api_id=1003。
4. 当前 run_go2_xt16_navigation_supervised_live.sh 是 Host-only launcher，默认依赖 /home/lin/new2/dddmr_navigation/install/setup.bash。
5. 当前 Docker wrapper 已经能运行 navigation-dry-run，但没有专门的 mixed live source 模式。
```

---

## 2. 当前主要问题

### 问题 1：Host-only live launcher 与实际构建边界不一致

现有脚本：

```bash
./scripts/run_go2_xt16_navigation_supervised_live.sh --live
```

会 source：

```text
/home/lin/new2/dddmr_navigation/install/setup.bash
```

并试图在 Host 上启动：

```text
dddmr_beginner_guide go2_xt16_navigation.launch
```

但包内 Host 构建记录显示：

```text
cloud_msgs / dddmr_sys_core: symlink-install 残留目录导致失败
mpc_critics: 缺 ackermann_msgs
```

所以 Host-only 方式不是当前 checkout 的正确路线。

### 问题 2：Docker 里不应该发布真实 Go2 Sport API

`go2_xt16_navigation.launch` 里已经有：

```xml
start_go2_sport_adapter
```

理论上可以在 Docker 里启动 `go2_sport_cmd_vel_adapter.py`。但不建议这样做：

```text
1. Docker image 只额外安装了 ros-humble-rmw-cyclonedds-cpp，没有安装 Unitree ROS API message 环境。
2. 真实 /api/sport/request 已经在 Host 上 live probe 成功。
3. 安全边界要求真实 Sport API output 在 Host 上显式 supervised 启动。
```

因此 Docker 侧只能作为 velocity source，不能作为 `/api/sport/request` publisher。

### 问题 3：当前 Host live script 会拒绝 Docker 容器

`run_go2_xt16_navigation_supervised_live.sh` 里的冲突检查会在发现 Go2/DDDMR Docker container 时退出。这个检查适合 Host-only 模式，但不适合推荐的 mixed 模式：

```text
Docker navigation source + Host sport adapter
```

需要改成：

```text
脚本自己启动 Docker navigation source；
或者允许一个由脚本启动的 Docker navigation source container 存在；
同时仍然禁止重复 Sport adapter、重复 RViz、重复 p2p_move_base。
```

### 问题 4：planner 速度参数和真实 Sport adapter 限幅不一致

当前 navigation yaml 中：

```yaml
max_vel_x: 1.2
min_vel_x: 0.25
max_vel_theta: 0.6
min_vel_theta: 0.25
rotation_speed: 0.5
acc_lim_x: 3.0
acc_lim_theta: 3.0
```

但 live adapter 硬限制为：

```text
max_x=0.10
max_y=0.05
max_yaw=0.15
```

这会导致局部规划器以为机器人能按 0.25~1.2 m/s 运动，但实际被 adapter 压到 0.10 m/s。结果可能是：

```text
路径跟踪慢、局部 planner 误判、反复旋转、目标附近震荡、controller timeout。
```

第一次 live click goal 前必须增加低速导航 profile，使 DDDMR 输出速度和 Go2 实际执行能力一致。

### 问题 5：adapter 对连续 zero cmd_vel 会重复 StopMove

当前 `go2_sport_cmd_vel_adapter.py` 在每次收到 cmd_vel 时都会：

```python
self.stop_sent_or_logged = False
```

然后如果命令为 0，就调用 StopMove。若 p2p_move_base 在目标完成后持续发布 0 速度，则会以控制频率重复发送 StopMove。

这不是致命问题，但 live 阶段不优雅，也会污染 request log。应改为：

```text
非零 Move 才清除 stop_sent 标志；
连续零速度只发一次 StopMove；
重新收到非零 Move 后，下一次零速度再发 StopMove。
```

### 问题 6：adapter 退出时缺少显式 StopMove 兜底

当前 watchdog 可以在 cmd_vel stale 后发布 StopMove，但如果 adapter 被 Ctrl-C 或脚本 kill，退出路径没有保证再发布 StopMove。live 阶段应补一个 shutdown stop：

```text
收到 Ctrl-C / SIGTERM / cleanup 时，尽量发布 2~3 次 StopMove，再销毁节点。
```

---

## 3. 最小安全架构

### 3.1 不要改动的边界

保持：

```text
Docker navigation launch 内部：
  /cmd_vel -> /dddmr_go2/dry_run_cmd_vel

Host adapter：
  /dddmr_go2/dry_run_cmd_vel -> /api/sport/request
```

不要做：

```text
1. 不要让 Docker 内部发布 /api/sport/request。
2. 不要把 p2p_move_base 的 /cmd_vel 直接接 Go2。
3. 不要继续把 Host full navigation build 当成 live 必经路径。
4. 不要发布 /lowcmd。
```

### 3.2 推荐新增模式

在 Docker wrapper 中新增一个更明确的模式：

```bash
./scripts/dddmr_docker_go2_xt16.sh navigation-live-source
```

它和 `navigation-dry-run` 的区别是：

```text
仍然启动 DDDMR navigation/RViz；
仍然把 /cmd_vel remap 到 /dddmr_go2/dry_run_cmd_vel；
但不启动 dry-run logger；
也不启动真实 Go2 sport adapter；
只作为 Host adapter 的 velocity source。
```

等价 launch 参数：

```bash
ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch \
  rviz:=true \
  publish_static_tf:=true \
  start_sport_dry_run_adapter:=false \
  start_go2_sport_adapter:=false
```

---

## 4. Codex 修改任务

### 任务 A：给 Docker wrapper 增加 `navigation-live-source`

修改：

```text
scripts/dddmr_docker_go2_xt16.sh
```

#### 1. usage 增加

```text
navigation-live-source
  Run Go2 XT16 DDDMR navigation/RViz as a velocity source only.
  It publishes /dddmr_go2/dry_run_cmd_vel but never publishes /api/sport/request.
```

#### 2. 支持可选 container name

在变量区增加：

```bash
DOCKER_NAME_VALUE="${DDDMR_DOCKER_NAME:-}"
```

在 `docker_base_args()` 中加入：

```bash
if [[ -n "${DOCKER_NAME_VALUE}" ]]; then
  args+=(--name "${DOCKER_NAME_VALUE}")
fi
```

这样 live supervisor 可以精确 stop 自己启动的 container。

#### 3. 增加 case

```bash
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
ros2 launch dddmr_beginner_guide go2_xt16_navigation.launch \
  rviz:=${rviz} \
  publish_static_tf:=${publish_static_tf} \
  start_sport_dry_run_adapter:=false \
  start_go2_sport_adapter:=false \
  \"\$@\""
    if [[ -n "${run_seconds}" ]]; then
      run_docker "${IMAGE}" bash -lc "${source_prefix}
timeout -s TERM -k 5s ${run_seconds}s bash -lc '${launch_cmd}'" bash "$@"
    else
      run_docker "${IMAGE}" bash -lc "${source_prefix}
${launch_cmd}" bash "$@"
    fi
    ;;
```

注意这里不要用 `-it`，方便 supervisor 脚本后台启动和清理。

---

### 任务 B：重写 live navigation supervisor 为 mixed runtime

修改或替换：

```text
scripts/run_go2_xt16_navigation_supervised_live.sh
```

核心行为改为：

```text
1. live 模式仍然要求 GO2_NAV_LIVE_CONFIRM。
2. live 模式仍然要求 GO2_SPORT_PROBE_SUMMARY，并验证 summary 中 Move/StopMove。
3. 不再 source Host DDDMR navigation install。
4. Host 只 source Go2 ROS setup 和 scripts/setup_go2_dds_env.sh。
5. 脚本启动 Docker navigation-live-source。
6. 等待 /dddmr_go2/dry_run_cmd_vel publisher 出现。
7. 启动 Host go2_sport_cmd_vel_adapter.py。
8. 启动 /api/sport/request echo log。
9. Ctrl-C 时先 StopMove，再停 adapter，再停 Docker container。
```

#### 推荐运行结构

```text
run_go2_xt16_navigation_supervised_live.sh --live
  ├── Docker navigation-live-source
  │     └── publishes /dddmr_go2/dry_run_cmd_vel
  ├── Host go2_sport_cmd_vel_adapter.py
  │     └── publishes /api/sport/request
  └── Host request echo logger
        └── logs /api/sport/request request ids
```

#### Host source 环境

不要 source Host DDDMR install。只做：

```bash
source /home/lin/go2_workspace/unitree_ros2/setup.sh
source scripts/setup_go2_dds_env.sh
```

原因：Host DDDMR build 当前失败，但 Host 只需要：

```text
rclpy
geometry_msgs
unitree_api/msg/Request
```

这些来自 ROS + Unitree Go2 setup。

#### 等待 Docker velocity source

启动 Docker 后等待：

```bash
ros2 topic info /dddmr_go2/dry_run_cmd_vel
```

期望：

```text
Type: geometry_msgs/msg/Twist
Publisher count >= 1
```

如果 20s 内没有 publisher，立刻 cleanup。

#### Host adapter 启动命令

```bash
/usr/bin/python3 src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py \
  --ros-args \
  -p cmd_vel_topic:=/dddmr_go2/dry_run_cmd_vel \
  -p request_topic:=/api/sport/request \
  -p enable_sport_output:=true \
  -p allow_real_request_topic:=true \
  -p max_x:=0.10 \
  -p max_y:=0.05 \
  -p max_yaw:=0.15 \
  -p stale_timeout_sec:=0.30 \
  -p request_id_base:="${nav_request_id_base}" \
  -p log_period_sec:=0.10
```

#### 清理顺序

```text
1. 尽量发布 StopMove 2~3 次。
2. kill host adapter。
3. docker stop live navigation container。
4. kill request echo logger。
5. 打印 log 路径。
```

---

### 任务 C：修 `go2_sport_cmd_vel_adapter.py` 的 StopMove 行为

修改：

```text
src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py
```

#### 1. 连续 zero cmd_vel 只发一次 StopMove

当前逻辑会在每个 zero cmd 上重置 stop flag。建议改成：

```python
def cmd_vel_cb(self, msg: Twist) -> None:
    now = self.get_clock().now()
    self.last_cmd_time = now

    x, y, yaw = self.to_sport_command(msg)
    if x == 0.0 and y == 0.0 and yaw == 0.0:
        self.handle_stop("zero cmd_vel", now)
        return

    self.stop_sent_or_logged = False
    payload = {"x": x, "y": y, "z": yaw}
    ...
```

`handle_stop()` 内保留：

```python
if self.stop_sent_or_logged:
    return
```

或者等价判断，确保连续 zero 不刷屏。

#### 2. adapter shutdown 时发布 StopMove

在 `finally` 里，destroy 前增加：

```python
try:
    if node.publisher_enabled:
        now = node.get_clock().now()
        node.handle_stop("adapter shutdown", now)
        rclpy.spin_once(node, timeout_sec=0.05)
except Exception as exc:
    node.get_logger().error(f"failed to publish shutdown StopMove: {exc}")
```

更稳的是发布 2~3 次，间隔 50 ms。

#### 3. 避免二次 shutdown traceback

第一轮失败日志里出现过：

```text
ExternalShutdownException -> rclpy.shutdown already called
```

建议 finally 写成：

```python
finally:
    node.destroy_node()
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
```

---

### 任务 D：增加低速导航 profile

修改：

```text
src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
```

第一次真实 click goal 不要用 1.2 m/s planner 参数。建议先改成：

```yaml
local_planner:
  ros__parameters:
    xy_goal_tolerance: 0.4
    yaw_goal_tolerance: 0.8

trajectory_generators:
  ros__parameters:
    differential_drive_rotate_shortest_angle:
      rotation_speed: 0.15

    differential_drive_rotate_inplace:
      rotation_speed: 0.15

    differential_drive_simple:
      max_vel_x: 0.10
      min_vel_x: 0.03
      max_vel_theta: 0.15
      min_vel_theta: 0.04
      acc_lim_x: 0.20
      acc_lim_theta: 0.30
      sim_time: 0.6
      linear_x_sample: 4.0
      angular_z_sample: 6.0
```

同时把 launch 中 dry-run logger 默认限幅改成和 live 一致：

```xml
<arg name="sport_dry_run_max_x" default="0.10"/>
<arg name="sport_dry_run_max_y" default="0.05"/>
<arg name="sport_dry_run_max_yaw" default="0.15"/>
```

原因：dry-run 日志应该反映 live 真实限制，不能 dry-run 看起来能跑，live 被硬 clamp 后行为变形。

---

### 任务 E：Host 脚本统一 source DDS env

修改：

```text
scripts/check_go2_xt16_sport_live_readiness.sh
scripts/run_go2_sport_adapter_supervised_probe.sh
scripts/run_go2_xt16_navigation_supervised_live.sh
```

在 source Go2 setup 后增加：

```bash
# shellcheck disable=SC1090
source "${WS_ROOT}/scripts/setup_go2_dds_env.sh"
```

Docker wrapper 已经 source 了同一个文件。Host 也显式 source，可以减少 Docker/Host DDS discovery 不一致。

---

## 5. 不改代码的临时运行方式

在 Codex 修改前，可以按两终端方式做一次非常保守的验证，但不要点远目标。

### Terminal 1：Docker 导航作为 velocity source

```bash
cd /home/lin/new2/dddmr_navigation
RVIZ=true PUBLISH_STATIC_TF=true \
  ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run \
  start_sport_dry_run_adapter:=false \
  start_go2_sport_adapter:=false
```

这个命令仍然不会发布 `/api/sport/request`。

### Terminal 2：Host 运行 live adapter

先 source 环境：

```bash
cd /home/lin/new2/dddmr_navigation
source /home/lin/go2_workspace/unitree_ros2/setup.sh
source scripts/setup_go2_dds_env.sh
```

确认 Docker 发布的 cmd topic 可见：

```bash
ros2 topic info /dddmr_go2/dry_run_cmd_vel
```

启动 echo 记录：

```bash
ros2 topic echo /api/sport/request unitree_api/msg/Request \
  > /tmp/go2_rviz_live_request_echo_$(date +%Y%m%d_%H%M%S).log
```

另开一个 Host shell 启动 adapter：

```bash
cd /home/lin/new2/dddmr_navigation
source /home/lin/go2_workspace/unitree_ros2/setup.sh
source scripts/setup_go2_dds_env.sh

/usr/bin/python3 src/dddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py \
  --ros-args \
  -p cmd_vel_topic:=/dddmr_go2/dry_run_cmd_vel \
  -p request_topic:=/api/sport/request \
  -p enable_sport_output:=true \
  -p allow_real_request_topic:=true \
  -p max_x:=0.10 \
  -p max_y:=0.05 \
  -p max_yaw:=0.15 \
  -p stale_timeout_sec:=0.30 \
  -p request_id_base:=$(date +%s) \
  -p log_period_sec:=0.10
```

只点第一个极短目标：

```text
距离 0.3~0.5 m
空旷地面
人手准备物理急停
不要靠墙、桌腿、台阶
```

如果任何行为异常，Ctrl-C adapter，然后立刻发 StopMove：

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
  "{header: {identity: {id: 0, api_id: 1003}}, parameter: ''}"
```

---

## 6. 正式运行流程

完成 Codex 修改后，建议正式 live 流程如下。

### 6.1 Readiness

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/check_go2_xt16_sport_live_readiness.sh
```

期望：

```text
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
```

### 6.2 Sport live probe

```bash
GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
  ./scripts/run_go2_sport_adapter_supervised_probe.sh --live
```

保存输出：

```text
SUMMARY_LOG=/tmp/go2_sport_adapter_live_<timestamp>_summary.env
```

### 6.3 RViz live navigation

```bash
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_<timestamp>_summary.env \
  ./scripts/run_go2_xt16_navigation_supervised_live.sh --live
```

脚本内部应自动：

```text
启动 Docker navigation-live-source；
等待 /dddmr_go2/dry_run_cmd_vel；
启动 Host adapter；
记录 /api/sport/request；
Ctrl-C 时 StopMove + cleanup。
```

---

## 7. Live click goal 验收标准

第一次 click goal 只允许：

```text
目标距离：0.3~0.5 m
max_x: 0.10 m/s
max_y: 0.05 m/s
max_yaw: 0.15 rad/s
地面：空旷、无台阶、无桌腿
旁边必须有人能立即停机
```

通过标准：

```text
1. RViz 中 map -> base_link 不跳变。
2. /dddmr_go2/dry_run_cmd_vel 有合理小速度。
3. /api/sport/request 中出现 api_id=1008 Move。
4. 到达、取消目标、或 Ctrl-C 后出现 api_id=1003 StopMove。
5. Go2 没有突然转圈、后退、侧冲、原地抖动。
6. request id 大于本次 nav_request_id_base，便于和 probe 区分。
```

失败立即停止：

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
  "{header: {identity: {id: 0, api_id: 1003}}, parameter: ''}"
```

---

## 8. 后续增强，不阻塞第一版 mixed live

下面这些建议后续再做，不要阻塞最小可运行 mixed live：

```text
1. go2_nav_cmd_gate：在 Host adapter 前增加独立安全门控。
2. /utlidar/cloud_base 近场 bumper：x 0.10~0.70, y ±0.35, z -0.30~0.20，检测到障碍强制 StopMove。
3. TF freshness / perception freshness gate：TF 或 segmented_cloud_pure 过期就停。
4. acceleration clamp：避免速度阶跃。
5. RViz cancel goal -> explicit StopMove 验证脚本。
```

当前 adapter 只有 clamp + stale watchdog，不等于完整安全 gate。第一次 live 必须人工监督、低速、短目标。

---

## 9. 最终判断

这包已经证明：

```text
Go2 Sport adapter 能在 Host 上真实发布 Move/StopMove；
Docker 内 DDDMR navigation 能构建；
导航 dry-run 和 RViz 路线是当前正确基础。
```

但现在不能直接用原来的 Host-only live navigation launcher。正确修复方向是：

```text
Docker 负责 DDDMR navigation/RViz；
Host 负责 Go2 Sport adapter；
两者通过 /dddmr_go2/dry_run_cmd_vel 在 ROS 2 DDS 上连接。
```

优先修：

```text
1. Docker wrapper 增加 navigation-live-source。
2. live supervisor 改成 Docker navigation + Host adapter。
3. adapter 修连续 zero StopMove 和 shutdown StopMove。
4. 导航配置降到 0.10 m/s 级别，与 adapter 限幅一致。
```

完成后再进入第一个 0.3~0.5 m supervised RViz click goal。
