# Go2 XT16 室内外连续任务

本功能使用一张连续的 DDDMR 位姿图完成“户外录制路线 → 门口停车 → 室内点目标导航”。运行过程中不切换地图，也不重启定位；切换的是控制模式。

## 运行结构

任务 Action 为 `/outdoor_indoor_mission`，Goal 包含：

- `mission_id`：本次任务标识；
- `route_id`：`${route_directory}/<route_id>.json` 中的户外路线；
- `indoor_goal`：`map` 坐标系中的室内 `PoseStamped`。

状态顺序为：

```text
VALIDATING
  -> OUTDOOR_TRACKING
  -> TRANSITION_STOP
  -> INDOOR_NAVIGATION
  -> COMPLETED
```

任何路线故障、定位非 `TRACKING`、定位健康非 `HEALTHY`、局部点云或里程计过期、停车确认超时、室内 Action 失败及任务取消，都会把命令仲裁切回 `none` 并停止任务。第一版不自动重试或沿路线倒退。

`outdoor_indoor_move_base_node` 只创建一套局部感知、轨迹生成器、critic 和局部规划器，户外控制器与室内 P2P 控制器共享这套资源。两者分别输出：

```text
/dddmr_go2/outdoor_cmd_vel + /dddmr_go2/outdoor_decision
/dddmr_go2/indoor_cmd_vel  + /dddmr_go2/indoor_decision
```

P2P 触发的恢复行为也被重映射到室内命令源；关闭 P2P 门时会同步取消未完成的恢复 Action。

`go2_mission_cmd_mux` 只把任务阶段选中的新鲜命令和对应 decision 转发到 `/dddmr_go2/dry_run_cmd_vel`。模式心跳、命令或 decision 超时均输出零速度。下游现有定位安全门和 Sport 适配器边界保持不变。

## 统一地图准备

1. 使用一次连续建图覆盖户外路线、门口过渡区和室内区域。门保持在实际任务状态，并在门口静止扫描及双向经过。
2. 检查 `map.pcd`、`ground.pcd`、`poses.pcd` 和 `edges.pcd`，确认门槛附近地面可连接且没有明显位姿漂移。
3. 把 `go2_xt16_navigation.yaml` 中 `map1.pose_graph_dir` 指向该地图在容器内的 `/root/dddmr_bags/...` 路径。
4. 针对同一地图重新录制或生成户外路线。路线 JSON 必须携带该地图的 SHA-256 指纹，路线终点就是入口交接锚点。
5. 不提交地图、ROS bag 或生成构建目录；它们保存在仓库外的 `bags/` 运行资产目录。

旧地图生成的路线不会被复用。任务服务会在接受运动前重新计算当前地图指纹，不匹配时返回 `REJECTED`。

## 检查与 dry-run

先执行纯静态检查：

```bash
cd /home/lin/new2/dddmr_navigation
MISSION_MAP_DIR=/home/lin/new2/bags/<unified-map> \
MISSION_ROUTE_ID=<outdoor-route-id> \
./scripts/run_go2_xt16_outdoor_indoor_supervised.sh --check
```

通过后启动 dry-run：

```bash
MISSION_MAP_DIR=/home/lin/new2/bags/<unified-map> \
MISSION_ROUTE_ID=<outdoor-route-id> \
./scripts/run_go2_xt16_outdoor_indoor_supervised.sh --dry-run
```

另一个已 source 同一 ROS 2 环境的终端发送任务：

```bash
ros2 run dddmr_route_navigation outdoor_indoor_mission_client.py \
  mission-001 <outdoor-route-id> <indoor-x> <indoor-y> <indoor-z> <indoor-yaw-rad>
```

查看状态：

```bash
ros2 topic echo /outdoor_indoor_mission/status
ros2 topic echo /go2_mission_cmd_mux/status
```

dry-run 只记录本来会形成的 Sport 请求，不创建 `/api/sport/request` 发布者。

## my_route_a 起点回程测试

专用配置 `go2_xt16_my_route_a_return_test.yaml` 绑定 `my_route_a` 对应的
2026-07-14 地图，并把 P2P 回程目标固定为路线起点：

```text
x=0.514610589  y=0.150227532  z=-0.210476711  yaw=-1.741948169
```

先运行静态检查，再运行完全隔离的零运动状态机测试：

```bash
./scripts/run_go2_xt16_my_route_a_return_test.sh --check
./scripts/run_go2_xt16_my_route_a_return_test.sh --dry-run
```

这里的 `--dry-run` 使用模拟路线控制器、持续零速里程计和模拟 P2P 成功，
验证巡航完成、交接停车、P2P 回起点及最终停车的完整状态顺序。Docker
使用 `--network=none`，不启动规划器、速度发布器、Sport adapter 或真实机器狗接口。
它验证任务编排，不替代后续带真实传感器但断开运动输出的感知/规划验收。

## 监督真机入口

真机运行复用现有 Sport adapter 探针、人工确认、速度限制和退出时 StopMove 逻辑。只有统一地图、路线指纹和 dry-run 全链路验收后，才可执行：

```bash
GO2_OUTDOOR_INDOOR_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_OUTDOOR_INDOOR_MISSION \
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
GO2_SPORT_PROBE_SUMMARY=/tmp/<successful-probe-summary>.env \
MISSION_MAP_DIR=/home/lin/new2/bags/<unified-map> \
MISSION_ROUTE_ID=<outdoor-route-id> \
./scripts/run_go2_xt16_outdoor_indoor_supervised.sh --live
```

启动本身不会发送任务。现场监督人员确认机器人位置、门状态、路线净空和室内目标后，再通过 Action 客户端发送任务。Ctrl-C、Action 取消或任一健康门失败都会关闭任务命令源并触发现有 StopMove 清理。
