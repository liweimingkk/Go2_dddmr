# Go2 XT16 技术文档与推进路线

本文档面向当前工作区：

```text
/home/lin/new2/dddmr_navigation
```

目标是把这个库的技术结构、Go2 + Hesai XT16 本地适配、已验证状态和后续路线合并成一份可继续维护的工程文档。

## 1. 项目定位

本仓库是基于 DDDMR Navigation 的 ROS 2 Humble 三维导航工作区。整体能力覆盖：

- 三维激光建图：基于 `dddmr_lego_loam/lego_loam_bor` 生成 pose graph、`map.pcd`、`ground.pcd` 等地图产物。
- 三维定位：`dddmr_mcl_3dl` 使用 LeGO-LOAM 生成的 pose graph/submap 进行粒子滤波定位。
- 三维感知层：`dddmr_perception_3d` 将静态地图、动态障碍、禁行区、限速区等转成可规划图层。
- 三维全局规划：`dddmr_global_planner` 在地面点云/图结构上做 A* 路径搜索，并发布 `weighted_ground` 等调试层。
- 三维局部规划：`dddmr_local_planner` 使用轨迹生成器、critics 和三维碰撞体进行局部避障与速度选择。
- 点到点导航状态机：`dddmr_p2p_move_base` 接收 3D 目标，查询全局路径，调用局部规划器，并输出速度。
- RViz 工具链：`dddmr_rviz_tools` 提供 3D 初始位姿、3D 目标、地图编辑、pose graph 编辑等插件。

当前工作区在上游框架之上增加了 Go2 EDU + Hesai XT16 的本地适配，重点是：

- 在 Docker 内使用稳定的 PCL/GTSAM/ROS 依赖组合。
- 支持 Go2 live `/lidar_points` XT16 点云。
- 支持 `/utlidar/robot_odom` 到标准 ROS body frame 的 odom 转换。
- 支持口部雷达/雷达点云 `/utlidar/cloud_base` 作为近场地面补充。
- 提供不发布真实运动命令的导航 dry-run 路线。

## 2. 核心包与职责

| 模块 | 路径 | 作用 |
|---|---|---|
| 建图 | `src/dddmr_lego_loam/lego_loam_bor` | XT16 点云投影、地面提取、特征关联、里程计/建图、pose graph 保存 |
| 定位 | `src/dddmr_mcl_3dl` | 读取 pose graph/submap，用粒子滤波输出 `map -> odom/base_link` 定位 |
| 地图服务 | `src/dddmr_pg_map_server` | 从 pose graph 目录发布 `mapcloud/mapground` |
| 感知 | `src/dddmr_perception_3d` | 静态层、动态障碍层、禁行层、限速层、点云编辑工具 |
| 全局规划 | `src/dddmr_global_planner` | 在地面图上生成全局路径，支持 static/dynamic graph |
| 局部规划 | `src/dddmr_local_planner` | 三维轨迹采样、碰撞检测、path tracking、恢复行为 |
| 状态机 | `src/dddmr_p2p_move_base` | `PToPMoveBase` action server，协调全局/局部规划并输出速度 |
| Go2 示例 | `src/dddmr_beginner_guide` | `go2_xt16_navigation.launch`、Go2 导航配置和 RViz |
| Docker/脚本 | `scripts/`, `dddmr_docker/` | Go2 Docker wrapper、预检、构建、建图、导航 dry-run |

## 3. 关键数据流

### 3.1 建图链路

```text
/lidar_points
  -> lego_loam_bor imageProjection
  -> /ground_cloud + /segmented_cloud_pure
  -> featureAssociation
  -> mapOptimization
  -> saved pose graph / map.pcd / ground.pcd
```

Go2 XT16 建图入口：

```text
src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_live.launch
src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_live.yaml
```

主要 XT16 参数：

```yaml
num_vertical_scans: 16
num_horizontal_scans: 4000
vertical_angle_bottom: -15.0
vertical_angle_top: 15.0
scan_period: 0.1
patch_first_ring_to_baselink: true
enable_loop_closure: false
```

当前 XT16 live 合同：

```text
topic: /lidar_points
frame: hesai_lidar
type: sensor_msgs/msg/PointCloud2
width: 64000
fields: x, y, z, intensity, ring, timestamp
ring: 0..15
scan period: about 0.1s
```

### 3.2 口部地面补充链路

```text
/utlidar/cloud_base
  -> time matching
  -> TF to base_link
  -> ROI/z-window ground filter
  -> /mouth_ground_cloud
  -> merge into /ground_cloud
```

入口文件：

```text
src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_mouth.launch
src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_mouth_config.yaml
scripts/run_go2_xt16_mouth_validation.sh
docs/go2_xt16_mouth_ground_fusion_notes.md
run_logs/go2_xt16_mouth_validation_20260702.md
```

当前固定输入：

```text
mouth topic: /utlidar/cloud_base
mouth frame override: empty
mouth filter frame: base_link
stationary z window: -0.42..-0.18
x window: 0.05..3.0
abs(y): <= 1.2
```

注意：`/utlidar/cloud_base` 已验证；原始 `/utlidar/cloud` 当前不作为默认路线，因为其 `frame_id=utlidar_lidar` 在当前 TF 树中未完成验证。

### 3.3 导航链路

```text
pose graph dir
  -> dddmr_pg_map_server
  -> map1/mapcloud + map1/mapground
  -> mcl_3dl localization
  -> perception_3d static/dynamic graph
  -> global_planner get_dwa_plan
  -> p2p_move_base
  -> local_planner
  -> /cmd_vel or dry-run remap
```

当前 Go2 dry-run 导航入口：

```text
src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
src/dddmr_beginner_guide/rviz/go2_xt16_navigation.rviz
```

`go2_xt16_navigation.launch` 默认做了安全隔离：

```text
/cmd_vel         -> /dddmr_go2/dry_run_cmd_vel
/cmd_vel_stamped -> /dddmr_go2/dry_run_cmd_vel_stamped
```

因此该路线适合先验证地图、定位、全局路径、局部规划和速度输出形态，不会直接驱动 Go2。

## 4. Go2 本地适配点

### 4.1 Docker 路线

本工作区保留上游 Docker，并增加薄 Go2 XT16 layer：

```text
base image: dddmr:x64
Go2 image: dddmr_go2_xt16:x64
wrapper: scripts/dddmr_docker_go2_xt16.sh
quickstart: docs/go2_xt16_docker_quickstart.md
```

常用命令：

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-go2-image
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
./scripts/dddmr_docker_go2_xt16.sh build-lego
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh mapping
```

### 4.2 坐标与 TF

当前路线把 Go2/XT16 统一到 ROS 常见 body 约定：

```text
base_link +X = physical forward
base_link +Y = left
base_link +Z = up
```

关键 launch 参数：

```text
base_link -> go2_imu:
  x=-0.02557, y=0.0, z=0.04232

go2_imu -> hesai_lidar:
  x=0.1710, y=0.0, z=0.0908, yaw=+1.5707963
```

`go2_odom_standardizer` 现在只保留稳定 topic 名称，默认不再旋转
`/utlidar/robot_odom`。2026-07-03 的轴向 probe 显示 raw odom 的位移方向和
yaw 基本自洽，旧的 `raw_to_standard_yaw=±90deg` 会破坏位移方向与 yaw 的一致性。

```text
input:  /utlidar/robot_odom
output: /dddmr_go2/robot_odom_standard
raw_to_standard_yaw: 0.0
rotate_parent_frame: false
rotate_twist: false
```

在 `lego_loam_go2_xt16_mouth.launch` 中，标准化 odom 默认关闭；如果打开，也应先保持
pass-through 并把 `odom_topic` 指到 `/dddmr_go2/robot_odom_standard`。只有独立轴向
probe 证明 raw odom 非标准时，才考虑非零 `raw_to_standard_yaw`。

## 5. 当前已验证状态

### 5.1 建图产物

顶层已有离线/现场建图产物：

```text
offline_maps/go2_xt16_dddmr_mapping_20260701_153225_20260701_153900/
offline_maps/go2_xt16_mouth_mapping_20260615_181903_20260701_152234/
live_rviz_test_20260701_165048/
```

其中包含：

```text
saved_maps/*/map.pcd
saved_maps/*/ground.pcd
saved_maps/*/edges.pcd
saved_maps/*/poses.pcd
rviz_map_only_screenshot.png
logs/
DONE
```

### 5.2 口部地面补充自动验证

`run_logs/go2_xt16_mouth_validation_20260702.md` 记录了 2026-07-02 的验证：

- 固定输入为 `/utlidar/cloud_base`。
- `/mouth_ground_cloud`、`/ground_cloud`、`/segmented_cloud_pure` 都有输出。
- 验证 bag 时长约 `19.8757s`。
- topic 计数：

```text
/utlidar/cloud_base       305
/mouth_ground_cloud       170
/ground_cloud             170
/segmented_cloud_pure     159
/lidar_points             170
/tf                       340
/tf_static                4
```

离线几何检查结论：

```text
/mouth_ground_cloud z_base min=-0.420 p05=-0.414 med=-0.367 p95=-0.261 max=-0.181
mouth_roi_ratio=1.0000
mouth_voxels_present_in_ground_cloud med=1.000
mouth_voxel_overlap_with_segmented_pure med=0.000 max=0.000
```

这说明当前代码已经把口部近场地面补充进 `/ground_cloud`，且没有把口部点注入 `segmented_cloud_pure`。

剩余人工验收：

- RViz 中 `/mouth_ground_cloud` 只能包含地面。
- 不能包含前腿、脚尖、墙根、箱体、低矮障碍。
- `/ground_cloud` 应比单 XT16 视角有更完整的前方近场地面。
- `/segmented_cloud_pure` 不应出现口部传感器带来的非地面污染。

## 6. 推进路线

### 阶段 0：安全边界

默认边界：

- 不发布真实 `/cmd_vel`。
- 不发布 `/api/sport/request`、`/lowcmd` 或其他 Go2 运动控制。
- 不修改 Go2 网络、系统服务、启动项、固件、标定或 `/home/unitree/xt16_ws`。
- 所有 live 任务先做只读预检。

进入真实闭环运动前，需要用户明确批准“监督运动测试”。

### 阶段 1：环境与 live 预检

目标：确认 Docker、DDS、XT16 和 odom/TF 当前仍可用。

命令：

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

验收：

- `/lidar_points` 有真实样本，不只看 publisher。
- 点云字段包含 `ring`、`timestamp`。
- frame 为 `hesai_lidar`。
- 速率接近 10 Hz。
- 无重复/陈旧容器干扰。

### 阶段 2：单 XT16 建图稳定性

目标：先跑稳定的单 XT16 建图，不引入口部补充和导航闭环。

命令：

```bash
./scripts/dddmr_docker_go2_xt16.sh build-lego
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh mapping
```

验收：

- `lego_loam` 不崩溃。
- `/ground_cloud`、`/segmented_cloud_pure` 有输出。
- RViz 地面不是空心。
- 保存后产出 `map.pcd`、`ground.pcd`、`edges.pcd`、`poses.pcd`。

如果地面空心，先在静止状态下调：

```text
patch_first_ring_to_baselink
ground_fov_bottom
ground_fov_top
ground_positive/negative start/stop
```

不要在运动中先调参数。

### 阶段 3：口部地面补充验收

目标：证明口部近场地面补充可靠，再把它作为正式建图输入。

命令：

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true MOUTH_MAX_TIME_DIFF=4.0 \
  scripts/run_go2_xt16_mouth_validation.sh
```

正式部署前必须重新测时间偏移：

```bash
source /opt/ros/humble/setup.bash
source scripts/setup_go2_dds_env.sh
python3 scripts/measure_go2_mouth_xt16_time_offset.py \
  --mouth-topic /utlidar/cloud_base \
  --xt16-topic /lidar_points \
  --duration 8
```

路线要求：

- 几何烟测可临时用 `mouth_max_time_diff:=3.0` 或 `4.0`。
- 运动或正式建图不应使用过大的 `mouth_max_time_diff`。
- 时间修好后建议回到 `0.08..0.15`。
- `mouth_time_offset_sec` 只能使用当前 live 图重新测得且稳定的值。

### 阶段 4：地图保存与导航配置切换

目标：把验收通过的 map/ground 切到导航配置。

需要确认地图目录包含：

```text
map.pcd
ground.pcd
edges.pcd
poses.pcd
pcd/
```

导航配置入口：

```text
src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
```

关键字段：

```yaml
map1:
  ros__parameters:
    pose_graph_dir: "/root/dddmr_bags/go2_xt16_dddmr_20260701_074740"
```

后续要把 `pose_graph_dir` 指向最终验收的地图目录，并保证 Docker 内路径和宿主机路径都能解析。

### 阶段 5：导航 dry-run

目标：验证定位、全局路径、局部规划和速度输出形态，不驱动 Go2。

构建：

```bash
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

启动：

```bash
RVIZ=true PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run
```

观察：

```text
/map1/mapcloud
/map1/mapground
/sub_mapcloud
/sub_mapground
/weighted_ground
/global_path
/dddmr_go2/dry_run_cmd_vel
/dddmr_go2/dry_run_cmd_vel_stamped
```

验收：

- RViz 中地图和机器人 TF 对齐。
- 初始位姿可被 MCL 接受。
- 3D goal 后有全局路径。
- 局部规划不把地面误判为障碍。
- dry-run 速度输出方向和大小符合预期。
- 没有真实 `/cmd_vel` 输出。

### 阶段 6：监督运动测试

只有阶段 5 通过后，且用户明确批准，才进入监督运动。

进入前检查：

- Go2 处于可物理接管状态。
- 有急停/遥控接管方案。
- 限速、限角速度、局部规划 cuboid、障碍层都已检查。
- dry-run 速度方向已验证。
- 没有重复 TF、重复 MCL、重复 planner 节点。

建议第一轮只做短距离、低速、单目标测试。

## 7. 常见故障定位顺序

### XT16 无数据或字段不对

先查：

```bash
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

不要只看 topic 是否存在。必须解码点云字段、ring 范围、timestamp 和 frame。

### `lego_loam` 崩溃

优先按上游 issue 58 思路排查：

- Docker/PCL/GTSAM ABI 是否混用。
- scan 参数是否和 XT16 实际数据匹配。
- `num_horizontal_scans`、`num_vertical_scans`、`scan_period` 是否正确。
- 是否存在 OMP voxel filter 或 loop closure 相关不稳定点。

本地 Go2 XT16 配置已把 `enable_loop_closure` 设为 `false`，并在 live 路线中关闭了 OMP ground voxel filter。

### 地面空心

优先按上游 issue 61 思路：

- 静止启动。
- 检查 `patch_first_ring_to_baselink: true`。
- 调整 ground FOV。
- 确认第一帧地面不是空心后再移动。

### 没有全局路径

依次查：

- `pose_graph_dir` 是否正确。
- `map1/mapground` 是否发布。
- `/weighted_ground` 是否连通。
- 起点/终点是否落在有效地面附近。
- 障碍层或 cuboid 是否把路径堵死。

### 有路径但速度为零

依次查：

- 目标是否已发送到 `PToPMoveBase`。
- `global_path` 是否存在。
- local planner 的 `odom_topic` 是否是当前有效 odom。
- TF 是否新鲜。
- `segmented_cloud_pure` 是否把可通行地面或机器人自身当障碍。
- dry-run 速度是否被 remap 到 `/dddmr_go2/dry_run_cmd_vel`。

## 8. 当前最短可执行路线

如果今天继续推进，建议按这个顺序做：

1. 跑 live 只读预检。
2. 用当前地图目录启动 navigation dry-run。
3. RViz 里确认 `map1/mapground`、`weighted_ground`、TF 和机器人位置。
4. 发送一个近距离 3D goal。
5. 只看 `/global_path` 和 `/dddmr_go2/dry_run_cmd_vel`。
6. 如果路径/速度合理，再记录 dry-run 证据。
7. 不进入真实运动，除非单独批准。

推荐入口命令：

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
./scripts/dddmr_docker_go2_xt16.sh build-navigation
RVIZ=true PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run
```
