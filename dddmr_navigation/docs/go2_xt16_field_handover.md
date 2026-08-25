# Go2 XT16 现场运行交接

本文是现场操作速查页。命令均从机器人上的工作区执行：

```bash
cd /home/unitree/go2_dddmr_full/dddmr_navigation
```

如果部署目录不同，只替换上面的 `cd` 路径；后续命令仍从
`dddmr_navigation/` 目录运行。Orin 使用 `eth0` 连接 Go2/XT16，`wlan0`
供操作笔记本查看 RViz，默认 `ROS_DOMAIN_ID=0`。

## 1. 最常用的三个入口

| 用途 | 脚本入口 | 是否会让机器人运动 |
|---|---|---|
| 建图、保存、切换导航地图 | `scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh` | 否，但人推/遥控机器人采图时必须低速并有人看护 |
| 导航 dry-run | `scripts/run_go2_xt16_navigation_supervised_live.sh --dry-run` | 否，只记录本来要发送的速度 |
| 真实导航 | `scripts/run_go2_xt16_navigation_supervised_live.sh --live` | **会**，必须先通过 readiness 和短距离 Sport probe |

建图和导航不要同时运行。建图脚本默认会先停止已有导航；真实导航终端按
`Ctrl-C` 会退出并发送 `StopMove`。

## 2. 每次开机先做只读预检

```bash
DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
GO2_NET_IFACE=eth0 \
GO2_DDS_EXTRA_IFACES=wlan0 \
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

必须看到 3 个真实 `/lidar_points` 样本。当前正确合同是：

```text
frame=hesai_lidar
width=32000
ring=0..15
约 10 Hz
字段包含 x/y/z/intensity/ring/timestamp
```

只有 publisher、没有实际样本，不能开始建图或导航。

源码或配置更新后，在 Orin 上重新构建对应入口：

```bash
# 建图链路
DDDMR_PLATFORM=orin-jp5 DDDMR_DOCKER_USE_SUDO=1 \
  ./scripts/dddmr_docker_go2_xt16.sh build-lego

# 导航链路；换图后也建议重新执行一次
DDDMR_PLATFORM=orin-jp5 DDDMR_DOCKER_USE_SUDO=1 \
  ./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

## 3. 建图

### 3.1 单终端：启动、采图、保存一次完成

```bash
sudo env \
  DDDMR_PLATFORM=orin-jp5 \
  GO2_NET_IFACE=eth0 \
  GO2_DDS_EXTRA_IFACES=wlan0 \
  RVIZ=false \
  MAP_RVIZ=false \
  ./scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh
```

脚本完成传感器、时间同步、TF、地图话题和口部地面点检查后开始建图。先让机器人
静止，确认首帧地面不空心，再低速采图。采集完成时，在这个终端中输入大写：

```text
SAVE
```

脚本随后会：

1. 调用 `/save_mapped_point_cloud`；
2. 将地图复制到仓库上一级的 `bags/<新地图目录>/`；
3. 检查 `map.pcd`、`ground.pcd`、`poses.pcd`、`edges.pcd` 和 `pcd/`；
4. 自动修改 `src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml` 中的
   `map1.pose_graph_dir`；
5. 默认停止建图容器。

也可定时自动保存，例如采集 180 秒：

```bash
sudo env \
  DDDMR_PLATFORM=orin-jp5 \
  GO2_NET_IFACE=eth0 \
  GO2_DDS_EXTRA_IFACES=wlan0 \
  RVIZ=false MAP_RVIZ=false MAPPING_SECONDS=180 \
  ./scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh
```

### 3.2 两阶段：Orin 建图，笔记本看 RViz，之后再保存

先在 Orin 启动一个固定名称的后台建图容器：

```bash
sudo env \
  DDDMR_PLATFORM=orin-jp5 \
  DDDMR_DOCKER_NAME=go2_xt16_mouth_mapping_field \
  GO2_NET_IFACE=eth0 \
  GO2_DDS_EXTRA_IFACES=wlan0 \
  RVIZ=false MAP_RVIZ=false \
  ./scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh --start-only
```

在操作笔记本的本仓库中打开只读建图画面：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/run_go2_xt16_laptop_mapping_rviz.sh
```

采集完成后回到 Orin 保存，容器名必须与启动时一致：

```bash
sudo env \
  DDDMR_PLATFORM=orin-jp5 \
  GO2_NET_IFACE=eth0 \
  GO2_DDS_EXTRA_IFACES=wlan0 \
  RVIZ=false MAP_RVIZ=false \
  ./scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh \
  --save-existing go2_xt16_mouth_mapping_field
```

`--start-only` 结束时也会在终端打印可直接复制的重连/保存命令。

### 3.3 建图注意事项

- 这个一键脚本是“顶部 XT16 + 口部雷达地面补充”建图流程，要求
  `/lidar_points`、`/utlidar/cloud_base` 和 `/utlidar/robot_odom` 正常。
- 当前已验收的 P2P 稳定基线地图是单 XT16 地图：
  `/root/dddmr_bags/go2_xt16_mapping_20260804_141647_xt16_only`。新脚本生成的
  双雷达地图不能未经 dry-run 和现场验收就替代该稳定基线。
- 想同时保留可离线重放的原始 bag 时，改用
  `scripts/run_go2_xt16_mouth_mapping_record_bag_save_to_nav.sh`；其交互仍是在
  结束时输入 `SAVE`。
- 中途不想保存时按 `Ctrl-C`。普通单终端模式会停止它控制的建图容器；
  `--start-only` 已经脱离终端，需使用脚本打印的 `--save-existing` 命令保存。

查看导航当前选中的地图：

```bash
rg -n 'pose_graph_dir' \
  src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
```

## 4. 导航 dry-run（先做，机器人不运动）

在 Orin 启动导航和安全日志链路：

```bash
DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
GO2_NET_IFACE=eth0 \
GO2_DDS_EXTRA_IFACES=wlan0 \
RVIZ=false \
./scripts/run_go2_xt16_navigation_supervised_live.sh --dry-run
```

在操作笔记本打开导航 RViz：

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/run_go2_xt16_laptop_navigation_rviz.sh
```

先设置初始位姿，再发一个近距离目标。至少确认：

- `/map1/mapcloud`、`/map1/mapground` 和机器人 TF 对齐；
- MCL 定位稳定；
- 目标产生 `/global_path`；
- `/dddmr_go2/dry_run_cmd_vel` 和 `/dddmr_go2/safe_cmd_vel` 有合理输出；
- 没有真实 `/api/sport/request` 运动发布器。

按 `Ctrl-C` 停止 dry-run 后，再考虑真实导航。

## 5. 真实导航（必须现场监督）

> 以下步骤会包含一次 `0.05 m/s`、默认 `0.6 s` 的短前进 probe，并最终允许
> RViz 目标驱动 Go2。清空场地，准备遥控器/App/物理急停，并由一人始终看护。

三个脚本共用同一个宿主机 Unitree ROS 环境。部署仓库已有消息工作区时先设置：

```bash
export GO2_SETUP="$PWD/.unitree_msg_ws/install/setup.bash"
sudo -v
```

`sudo -v` 只预先刷新 Orin 的 Docker 权限，不会启动 ROS 或发送运动命令；避免
后续 Docker velocity source 在后台等待 sudo 密码。

第一步，只读 readiness：

```bash
GO2_SETUP="$GO2_SETUP" \
  ./scripts/check_go2_xt16_sport_live_readiness.sh
```

末行必须是：

```text
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
```

第二步，在人和急停都就位后做短距离真实 Sport probe：

```bash
GO2_SETUP="$GO2_SETUP" \
GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
  ./scripts/run_go2_sport_adapter_supervised_probe.sh --live
```

记录输出中的 `SUMMARY_LOG=/tmp/go2_sport_adapter_live_..._summary.env`。

第三步，把刚生成的 summary 传给真实导航入口：

```bash
GO2_SETUP="$GO2_SETUP" \
DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
GO2_NET_IFACE=eth0 \
GO2_DDS_EXTRA_IFACES=wlan0 \
RVIZ=false \
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_YYYYMMDD_HHMMSS_summary.env \
  ./scripts/run_go2_xt16_navigation_supervised_live.sh --live
```

将示例 summary 文件名替换为第二步的真实输出。看到
`RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING` 后，才在笔记本 RViz 中发送一个
近距离、空旷、可随时中止的目标。停止时在 Orin 的 live 导航终端按 `Ctrl-C`，
并确认脚本已经发送 `StopMove`、容器已退出。

禁止绕过 readiness、probe 或确认短语，也不要同时运行键盘遥控、另一套导航、
`ros2 topic pub` 速度命令或第二个 Sport adapter。

## 6. 常见问题

- **Docker 权限失败**：wrapper 命令使用 `DDDMR_DOCKER_USE_SUDO=1`；建图一键脚本
  按本文使用 `sudo env ...`。
- **笔记本看不到地图/RViz 话题**：确认 Orin 命令包含
  `GO2_DDS_EXTRA_IFACES=wlan0`，两端 `ROS_DOMAIN_ID` 相同，笔记本已连接 Orin
  操作热点。
- **地面空心**：机器人保持静止，先检查 `patch_first_ring_to_baselink` 和地面
  FOV，不要边运动边调参。
- **没有全局路径**：先检查定位、起点/终点是否贴近有效地面，以及
  `/map1/mapground` 是否连续；不要直接进入真实运动排障。
- **新地图导航异常**：把 `pose_graph_dir` 切回已验收的单 XT16 基线，重新执行
  `build-navigation` 并只跑 dry-run。

更详细的背景见：

- [当前稳定地图与 P2P 基线](go2_xt16_p2p_release_20260804.md)；
- [Orin 构建、DDS 和笔记本 RViz](go2_xt16_orin_jp5.md)；
- [真实 Sport 导航门禁](go2_xt16_sport_live_runbook.md)；
- [x64/Humble 开发机流程](ubuntu22_humble_go2_xt16_run_guide.md)。
