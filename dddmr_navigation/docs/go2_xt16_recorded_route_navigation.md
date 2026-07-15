# Go2 XT16 三维录制路线巡航（不使用 DDDMR 全局导航）

## 功能边界

这套功能把人工走过的路线作为三维参考线，直接交给 DDDMR Local Planner 生成
`vx / vy / yaw_rate`。它不启动以下节点：

- `global_planner`
- `P2PMoveBase`
- `clicked2goal`
- recovery behaviors

因此这里没有 Nav2，也没有 DDDMR 点到点全局重规划。实时障碍仍由现有
`perception_3d_local` 和 collision critics 检查；只要三维走廊内存在安全轨迹，
Local Planner 就可以短距离绕开障碍。走廊内没有安全轨迹时，控制器停车等待，
5 秒后仍未恢复就进入 `FAULT` 并停用，不会换一条未经人工走过的路线。

坡道和楼梯上的路线点包含 `x/y/z + quaternion`。其中 `z` 用于三维路线关联、
高度偏离检查和局部轨迹约束；输出仍是机器狗底盘平面速度，不发送垂直速度，
也不自动切换步态、身体高度或楼梯模式。坡面姿态稳定和落脚仍由 Go2 本体控制器
负责。

## 安全链

控制器是 fail-closed 的：

1. 启动后默认 `DISABLED`，没有自动启用参数。
2. 每次收到新路线都会立即停用，必须重新显式调用服务。
3. 只在 `/localization_status == TRACKING` 且状态、`map -> base_link` 位姿都新鲜时工作。
4. 默认只允许从路线起点附近开始，不允许从长路线中间随意吸附。
5. 路线进度只能向前，且只在当前进度附近搜索，避免在交叉路线处跳段。
6. 当前机器人必须处在水平 `0.60 m`、高度 `0.35 m` 的路线走廊内。
7. `RouteCorridorModel` 会拒绝任何将要离开同一三维走廊的前向局部轨迹。
8. 原有地图、激光雷达和机器人包围盒碰撞检查继续生效。
9. 控制器还会限幅到 `x <= 0.35 m/s`、`y = 0`、`|yaw| <= 0.50 rad/s`。
10. 专用 launch 只接 dry-run 速度链，并明确关闭 Unitree Sport 实际输出；监督式
    live 入口只能在宿主机通过额外三重门控接入 Sport，且再次限幅。

走廊宽度是“允许局部绕障的范围”，不是通行安全证明。室内参数只能用来验证
软件流程；户外坡道、楼梯必须重新测量定位误差、地图高度噪声和机器狗实际通过
能力后再收紧或调整。

## 构建

在仓库的 DDDMR 工作区中执行：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

`dddmr_beginner_guide` 已声明对 `dddmr_route_navigation` 的运行依赖，因此现有
`build-navigation` 会一起构建新包。

## 用现有室内位姿图生成测试路线

当前地图的 `poses.pcd` 是建图时的 `map -> base_link` 三维位姿序列，可以先用来
验证路线加载、进度和 dry-run 控制链：

```bash
cd /home/kkkkkkq/new2_success/new22/new2
mkdir -p bags/routes
python3 dddmr_navigation/src/dddmr_route_navigation/scripts/route_tool.py \
  convert-pose-graph \
  bags/go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36/poses.pcd \
  bags/routes/go2_xt16_indoor_reverse_test.json \
  --route-id go2_xt16_indoor_reverse_test \
  --map-dir bags/go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36
```

对当前文件的离线校验结果应为：去掉起步静止重复点后约 `62` 个点，三维长度约
`10.532 m`。可以再次独立校验：

```bash
python3 dddmr_navigation/src/dddmr_route_navigation/scripts/route_tool.py \
  inspect bags/routes/go2_xt16_indoor_reverse_test.json
```

JSON 会记录 `poses.pcd` 以及 `map/ground/edges` 的 SHA-256 指纹，便于确认路线和
地图是否来自同一批工件。当前实现把指纹作为审计信息；运行前仍需由操作员核对，
控制器不会自动寻找或替换地图。

## 启动默认无运动测试

推荐直接使用一键 dry-run 脚本：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/run_go2_xt16_recorded_route_dry_run.sh
```

脚本会自动生成或校验默认测试路线、核对路线与地图指纹、检查残留导航进程、执行
XT16 只读 preflight，然后在后台启动专用 dry-run launch。启动完成后控制器仍保持
`READY/DISABLED`，不会自动启用路线控制，也不会创建真实 Sport 请求发布者。

常用控制命令：

```bash
# 查看路线、定位和控制器状态
./scripts/run_go2_xt16_recorded_route_dry_run.sh --status

# 定位为 TRACKING 且机器人在路线起点附近后，显式启用 dry-run 计算
./scripts/run_go2_xt16_recorded_route_dry_run.sh --enable

# 停用控制并清零进度
./scripts/run_go2_xt16_recorded_route_dry_run.sh --disable

# 停止容器并保存 Docker 日志
./scripts/run_go2_xt16_recorded_route_dry_run.sh --stop
```

脚本没有 `--live` 模式。若只想离线生成和校验路线，可使用
`--prepare-only`；需要重新构建时使用 `BUILD=true`。以下保留手工启动方法，便于
排障和逐节点检查。

进入 Go2 Docker：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh shell
```

在容器中执行：

```bash
source /opt/ros/humble/setup.bash
source /root/dddmr_navigation/.docker_go2_xt16_install/setup.bash
ros2 launch dddmr_beginner_guide go2_xt16_recorded_route_navigation.launch \
  route_file:=/root/dddmr_bags/routes/go2_xt16_indoor_reverse_test.json \
  start_route_file_publisher:=true \
  rviz:=true
```

这个 launch 会启动地图、MCL、特征、局部感知、Local Planner、命令安全门和
dry-run adapter；不会启动全局规划/P2P，也不会启动真实 Sport adapter。即使路线
已经加载，状态仍应为 `READY`，输出只能是零：

```bash
ros2 topic echo /recorded_route_controller/status
ros2 topic echo /recorded_route_controller/route_ready
ros2 topic echo /recorded_route_controller/prepared_route
```

先确认 `/localization_status` 稳定为 `TRACKING`、机器人位于路线起点附近，再在
这个 dry-run launch 下显式启用：

```bash
ros2 service call /recorded_route_controller/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

观察但不执行真实运动：

```bash
ros2 topic echo /recorded_route_controller/status
ros2 topic echo /recorded_route_controller/progress
ros2 topic echo /dddmr_go2/dry_run_cmd_vel
ros2 topic echo /dddmr_go2/safe_cmd_vel
```

随时停用并清零进度：

```bash
ros2 service call /recorded_route_controller/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

## 人工遥控录制户外路线

先启动地图和 MCL，让 `/localization_status` 达到稳定 `TRACKING`。路线记录器本身
不发送任何运动命令；操作员使用原厂遥控器人工驾驶：

推荐使用录制脚本。它只会连接到已经运行的 recorded-route dry-run 容器，确认
`TRACKING` 后才开始订阅 `/mcl_pose`；不会启用控制器，也不会发布运动指令：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation

# 终端 1：保持控制器 READY/DISABLED，不要执行 --enable
./scripts/run_go2_xt16_recorded_route_dry_run.sh

# 终端 2：用原厂遥控器人工走路线，结束后按 Ctrl-C 保存
ROUTE_ID=park_route_a ./scripts/record_go2_xt16_recorded_route.sh
```

默认保存位置为 `bags/routes/<ROUTE_ID>.json`。脚本拒绝覆盖同名路线；确认需要
替换时才显式设置 `OVERWRITE_ROUTE=true`。录制完成后，使用脚本输出的
`ROUTE_FILE=... ROUTE_MAP_DIR=... run_go2_xt16_recorded_route_dry_run.sh` 命令，
重新启动并加载新路线。

手工命令等价于：

```bash
ros2 run dddmr_route_navigation route_tool.py record \
  /root/dddmr_bags/routes/park_route_a.json \
  --route-id park_route_a \
  --map-dir /root/dddmr_bags/PARK_MAP_DIRECTORY \
  --pose-topic /mcl_pose \
  --status-topic /localization_status
```

记录器只在定位状态为 `TRACKING` 时采样，默认每移动 `0.10 m` 或转向
`0.15 rad` 记录一点。人工路线结束后按 `Ctrl-C`，文件会先写临时文件再原子替换。
若相邻定位点间隔超过 `2.0 m`、四元数无效或点数少于 3，保存会失败。

户外路线建议同时记录以下人工审查信息（当前 JSON 尚不自动控制这些项目）：

- 单向/双向通行约束；
- 每段坡度、楼梯方向、台阶高度和落脚宽度；
- 必须人工接管的位置和通信盲区；
- 行人密集区、临水区、车行道等禁止局部绕行边界；
- 所需 Go2 步态、身体高度和最大速度。

## 从 dry-run 到现场验证

现场放开运动前至少应依次完成：

1. 离线单元测试和路线 JSON 校验；
2. RViz 中核对 `prepared_route` 与同一地图完全重合；
3. 静止状态验证 MCL `TRACKING`、TF、XT16 更新率和障碍输入；
4. dry-run 检查路线起点、终点、进度单调性、走廊故障和堵塞超时；
5. 抬脚架空或轮廓隔离测试实际适配器的轴向和 StopMove；
6. 平地短距离、低风险、有人急停的监督测试；
7. 坡道单段测试；
8. 楼梯单段测试；
9. 最后才是完整户外长路线。

不要把 dry-run launch 中的 `start_go2_sport_adapter=false` 改成 true。专用入口是：

```text
scripts/run_go2_xt16_recorded_route_supervised_live.sh
```

它不复用普通点到点的 `run_go2_xt16_navigation_supervised_live.sh --live`，而是启动
录制路线专用 Docker 栈，保持控制器 `DISABLED`，再由宿主机连接最终 Sport
适配器。默认不带参数时只进行离线检查，绝不启动 Docker 或发布 ROS 消息。

### `my_route_a` 离线检查

当前路线应为 `26` 个输入点、三维长度约 `3.132 m`，并绑定地图指纹
`4c341cfe0dc1fb4c5eab2b60b32d2bdbbbb33a1e6733287bc1ac15e82b00ad85`：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation

ROUTE_FILE=/home/kkkkkkq/new2_success/new22/new2/bags/routes/my_route_a.json \
ROUTE_MAP_DIR=/home/kkkkkkq/new2_success/new22/new2/bags/go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36 \
./scripts/run_go2_xt16_recorded_route_supervised_live.sh --check
```

预期结束标志为：

```text
RESULT=GO2_XT16_RECORDED_ROUTE_OFFLINE_CHECK_PASS
```

### 现场 Sport 短探针

执行探针前必须停掉所有 dry-run/navigation 容器。下面的 `--live` 探针会让实体
Go2 以 `0.05 m/s` 前进约 `0.6 s`，只能在平整、开阔区域进行；第二名操作员必须
持有原厂遥控器或物理急停。它同时验证 `Move`、`StopMove` 和路线使用的决策门：

```bash
./scripts/run_go2_xt16_recorded_route_dry_run.sh --stop

GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
GO2_SPORT_PROBE_X=0.05 \
GO2_SPORT_PROBE_Y=0.0 \
GO2_SPORT_PROBE_YAW=0.0 \
GO2_SPORT_MAX_X=0.10 \
GO2_SPORT_MAX_Y=0.0 \
GO2_SPORT_MAX_YAW=0.20 \
GO2_SPORT_PROBE_DECISION=d_controlling \
GO2_REQUIRE_MOTION_DECISION=true \
GO2_MOTION_ALLOWED_DECISIONS=d_controlling,d_align_heading,d_align_goal_heading \
./scripts/run_go2_sport_adapter_supervised_probe.sh --live
```

保留输出的 `SUMMARY_LOG=/tmp/go2_sport_adapter_live_..._summary.env`。监督式路线入口
会重新读取原始 request echo，要求确实出现本次探针 request ID 之后的
`api_id=1008` 和 `api_id=1003`，并要求探针限幅、超时、决策门参数与实跑完全一致。
探针还会记录 Sport adapter 的 SHA-256；代码有变化就必须重新探针。报告默认只能
在生成后 `1 h` 内使用。

### `my_route_a` 监督式低速实跑

先用原厂遥控器把 Go2 放到路线起点附近，在 RViz 中确认朝向与路线第一段一致。
`my_route_a` 的起点约为 `map: (-0.288, 0.344, -0.141)`；控制器虽然允许
`0.60 m / 0.35 m` 的起点包络，第一次测试应尽量贴近录制起点。确保整条短路线
清空、终点有人看守、终端保持前台后执行。监督入口会在连接真实 adapter 之前
额外执行更严格的起点检查：水平误差不超过 `0.25 m`、高度误差不超过 `0.20 m`、
航向误差不超过 `0.35 rad`；不满足时应修复定位或重新摆放，不能靠放宽阈值掩盖：

```bash
GO2_RECORDED_ROUTE_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_RECORDED_ROUTE \
GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_YYYYMMDD_HHMMSS_summary.env \
GO2_RECORDED_ROUTE_EXPECTED_ID=my_route_a \
ROUTE_FILE=/home/kkkkkkq/new2_success/new22/new2/bags/routes/my_route_a.json \
ROUTE_MAP_DIR=/home/kkkkkkq/new2_success/new22/new2/bags/go2_xt16_mouth_mapping_20260714_153136_map_2026_07_14_07_31_36 \
RVIZ=true \
./scripts/run_go2_xt16_recorded_route_supervised_live.sh --live
```

程序会依次执行 XT16 只读 preflight、路线/地图指纹核对、MCL `TRACKING`、节点
拓扑、速度/决策 publisher 唯一性检查。真实适配器接通后，路线控制器仍为
`DISABLED`，此时只会发送 `StopMove`。操作员再次核对现场后必须在同一终端输入：

```text
ENABLE my_route_a
```

必须在 `60 s` 内输入，且完全匹配后才会调用控制器 enable 服务。默认最终硬限幅为
`x <= 0.10 m/s`、`y = 0`、`|yaw| <= 0.20 rad/s`，并强制要求新鲜的
`d_controlling / d_align_heading / d_align_goal_heading` 决策。yaw-arc shim 在这条
首轮实跑入口中固定关闭。

以下任一条件都会停用控制器并发送额外 `StopMove`：

- 按 `Ctrl-C`；
- 定位或控制器进入 `FAULT`；
- 跟踪进度连续 `15 s` 没有增长；
- 起点或终点朝向对齐超过 `20 s`；
- 总运动时间超过 `120 s`；
- 真实 Sport、safe velocity 或路线决策 publisher 不再保持唯一；
- Docker 速度源或宿主机 Sport adapter 异常退出。

结束后脚本会删除本次容器，并在 `/tmp` 保存 Docker、adapter、request echo 和
summary 日志。第一次真实验证仅限平地短距离；该入口不会自动切换 Go2 步态、身体
高度或楼梯模式，不能因为平地通过就直接用于坡道或楼梯。

## 关键参数

参数文件：
`dddmr_route_navigation/config/go2_xt16_recorded_route.yaml`

- `route.corridor_max_xy_error`：机器人当前水平偏离上限；
- `route.corridor_max_z_error`：机器人当前高度偏离上限；
- `route.progress_search_forward/backward`：允许关联的局部进度窗口；
- `route.local_plan_forward/backward`：每周期送给 Local Planner 的路线片段；
- `route.start_max_*`：显式启用时的起点包络；
- `route.goal_max_*`：进入终点朝向对齐的包络；
- `route_corridor.max_xy_distance/max_z_distance`：预测轨迹硬走廊；
- `blocked_timeout_sec`：走廊内无安全轨迹时的停车等待上限。

`route.corridor_*` 与 `route_corridor.*` 应保持一致。不要只放大其中一个，也不要用
放宽高度走廊来掩盖楼梯地图、TF 或 MCL 的高度错误。
