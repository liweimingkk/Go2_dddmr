# DDDMR LeGO-LOAM 离线建图适配说明

> 适用对象：`dfl-rlab/dddmr_navigation/src/dddmr_lego_loam`，尤其是 `lego_loam_bor` 的离线 bag 建图流程。  
> 目标：明确离线建图需要录制哪些 topic、TF 需要哪些、以及建图适配时最容易踩坑的参数。

---

## 1. README 核心理解

这个包不是原版 LeGO-LOAM 的简单 ROS2 移植，而是面向地面移动机器人的 **LeGO-LOAM-BOR** 改版。它支持：

- ROS2 Humble；
- 在线建图和离线 bag 建图；
- LiDAR-only / 可选 wheel odometry / 可选 IMU；
- RViz 交互式暂停、继续和参数调整；
- pose graph 可视化；
- 条件闭环；
- pose graph 保存；
- pose graph editor 手动闭环；
- 两张 pose graph 合并；
- 后续给 DDDMR MCL 定位使用。

工程适配时要重点理解：**这个包的结果不仅是最终拼接的 `map.pcd`，更重要的是完整 pose graph 输出目录。**

---

## 2. 离线建图需要录制哪些 topic

### 2.1 最小可用录制

最小只需要录制 LiDAR 原始点云：

```bash
ros2 bag record -o map_bag /your_lidar_points
```

其中 `/your_lidar_points` 替换成真实点云 topic，例如：

```text
/lslidar_point_cloud
/velodyne_points
/livox/lidar
/hesai/pandar
/rslidar_points
```

对应配置中需要修改：

```yaml
bag_reader:
  ros__parameters:
    use_sim_time: true
    bag_file_dir: "/path/to/map_bag"
    point_cloud_topic: "/your_lidar_points"
    odometry_topic: "/odom"
    skip_frame: 1
```

即使第一轮不用轮速里程计，也建议 `odometry_topic` 保持一个合法 topic 名，比如 `/odom`，不要留空。

---

### 2.2 推荐录制

实车离线建图建议录制：

```bash
ros2 bag record -o mapping_001 \
  /your_lidar_points \
  /odom \
  /tf \
  /tf_static
```

推荐原因：

- `/your_lidar_points`：必须，算法主要输入。
- `/odom`：强烈建议录制。第一轮可以不用，但后续如果切换到 `wheel_odometry` 会用到。
- `/tf`：建议录制，方便排查当时机器人 TF 状态。
- `/tf_static`：建议录制，方便留档和排查静态外参。

但是要注意：**录了 `/tf` 和 `/tf_static` 不代表离线建图时它们会自动生效。** 这个包的离线流程不是简单 `ros2 bag play`，而是 `lego_loam_bag` 自己读取 bag。正常流程下仍然应该在 launch 中显式发布 LiDAR 静态外参。

---

## 3. 离线建图第一轮建议配置

第一轮建议先使用激光里程计，不要一上来依赖轮速：

```yaml
laser:
  odom_type: "laser_odometry"
```

当点云投影、地面提取、TF、轨迹都稳定后，再评估是否切换到：

```yaml
laser:
  odom_type: "wheel_odometry"
```

如果使用 `wheel_odometry`，则 `/odom` 必须录制，并且时间戳、frame、方向、尺度都要正确。

---

## 4. TF 需要哪些

### 4.1 最核心的 TF

必须保证下面这条 TF 链可查到：

```text
base_ground_frame -> lidar_frame
```

其中：

- `base_ground_frame` 来自 YAML 中的：

```yaml
laser:
  base_ground_frame: "base_link"       # 或 "base_footprint"
```

- `lidar_frame` 来自点云消息的 `header.frame_id`，不是 topic 名。

检查点云 frame：

```bash
ros2 topic echo /your_lidar_points/header --once
```

如果输出是：

```yaml
frame_id: laser_link
```

那离线建图时必须能查到：

```text
base_link -> laser_link
```

或者：

```text
base_footprint -> laser_link
```

具体取决于 YAML 里的 `laser.base_ground_frame`。

---

### 4.2 推荐 TF 树结构

#### 情况 A：`base_link` 就在地面中心

可以使用：

```text
base_link
└── laser_link
```

YAML：

```yaml
laser:
  base_ground_frame: "base_link"
```

launch 中发布静态 TF，例如：

```xml
<node pkg="tf2_ros"
      exec="static_transform_publisher"
      name="sensor2baselink"
      args="x y z yaw pitch roll base_link laser_link" />
```

这里的 `x y z yaw pitch roll` 必须替换成你们实测的 LiDAR 外参，不能照抄 demo 数值。

---

#### 情况 B：`base_link` 不在地面，车体中心离地

地面车更推荐使用 `base_footprint`：

```text
base_footprint
└── base_link
    └── laser_link
```

或者简化为：

```text
base_footprint
└── laser_link
```

YAML：

```yaml
laser:
  base_ground_frame: "base_footprint"
```

验证：

```bash
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

只要这个命令能持续输出正确的平移和旋转，核心 TF 就满足要求。

---

### 4.3 `/tf_static` 录了也要在离线 launch 里发布

离线 bag 中建议录制 `/tf_static`，但建图时仍建议在 launch 中显式写：

```xml
<node pkg="tf2_ros"
      exec="static_transform_publisher"
      name="sensor2baselink"
      args="你的外参 base_link laser_link" />
```

原因是 `lego_loam_bag` 主要读取配置中的点云 topic 和 odom topic，不会像 `ros2 bag play` 那样自动把 bag 里的 `/tf_static` 重新发布成当前 ROS graph 的 TF。

---

## 5. `/odom` 应该怎么录和怎么设

如果第一轮使用：

```yaml
laser:
  odom_type: "laser_odometry"
```

那么 `/odom` 不是强依赖，但建议录制，方便后续对比和调参。

如果使用：

```yaml
laser:
  odom_type: "wheel_odometry"
```

则 `/odom` 必须是：

```text
nav_msgs/msg/Odometry
```

推荐 frame：

```text
odom.header.frame_id = "odom"
odom.child_frame_id  = "base_link" 或 "base_footprint"
```

更稳妥的 TF 结构：

```text
odom
└── base_footprint
    └── base_link
        └── laser_link
```

或：

```text
odom
└── base_link
    └── laser_link
```

注意：**不要让多个节点同时发布同一条 `odom -> base_link` 或 `odom -> base_footprint` TF。** TF 冲突会导致建图异常跳变。

---

## 6. LiDAR 参数适配重点

建图适配时，点云投影参数必须和真实 LiDAR 对上。重点参数包括：

```yaml
laser:
  num_vertical_scans: 16
  num_horizontal_scans: 1000
  vertical_angle_bottom: -15.0
  vertical_angle_top: 15.0
  scan_period: 0.1
```

以及：

```yaml
imageProjection:
  minimum_detection_range: 0.5
  maximum_detection_range: 80.0
  stitcher_num: 1
```

需要确认：

- `num_vertical_scans` 是否等于真实线数或等效线数；
- `num_horizontal_scans` 是否让 range image 不过稀、不过密；
- `vertical_angle_bottom/top` 是否覆盖真实垂直 FOV；
- `scan_period` 是否匹配真实雷达频率；
- `minimum_detection_range` 是否排除了车体、雷达盲区和近距离噪点；
- `maximum_detection_range` 是否避免远距离噪声过多进入优化；
- 非重复扫描雷达，如 Livox / Mid360，是否需要调整 `stitcher_num`。

对于 Livox / Mid360 这类非重复扫描雷达，适当增大 `stitcher_num` 可以增加点云密度，但车速快、转弯快时会引入运动畸变。因此建议第一轮从 `1` 开始。

---

## 7. Ground FOV 是适配重点

这个版本对 Ground FOV 非常敏感，因为它需要依赖地面约束稳定 roll、pitch 和 z。

典型参数：

```yaml
imageProjection:
  ground_fov_bottom: ...
  ground_fov_top: ...
  ground_positive_start: 0.0
  ground_positive_stop: 3.1415926
  ground_negative_start: 0.0
  ground_negative_stop: -3.1415926
```

注意：配置中的 ground FOV 通常是 **弧度**，例如：

```text
-0.2617994 rad ≈ -15°
-0.0872660 rad ≈ -5°
```

调参判断：

- Ground FOV 太窄：地面点少，地图容易飘，z/roll/pitch 约束弱。
- Ground FOV 太宽：墙、货架、车体、坡道边缘容易被误认为地面。
- 雷达倒装、斜装、90° 侧装时，Ground FOV 的符号和上下界经常需要重新计算。
- 车体、保险杠、轮子进入雷达视野时，可以配合 ignore FOV 或加大 `minimum_detection_range` 排除。

建议在 RViz 中重点观察：

```text
ground_cloud
patched_ground
patched_ground_edge
```

不要只看最终 `map.pcd`。

---

## 8. 地面 patch 和坡度过滤

常见参数：

```yaml
imageProjection:
  distance_for_patch_between_rings: 1.0
  ground_slope_tolerance: 0.34
  ground_dz_tolerance: 0.1
  patch_first_ring_to_baselink: true
```

调参建议：

- 平整室内、仓库：阈值可以偏紧，减少墙脚、货架脚误入地面。
- 室外坡道、减速带、地面起伏：阈值要适当放松，否则地面会被切碎。
- 雷达安装较高、有近距离盲区：`patch_first_ring_to_baselink: true` 通常有帮助。
- 车体前方有复杂结构：patch 可能把非地面补进去，需要结合 RViz 判断。

---

## 9. 回环和 keyframe 参数

常见 mapping 参数：

```yaml
mapping:
  distance_between_key_frame: 1.0
  angle_between_key_frame: 1.0
  enable_loop_closure: true
  surrounding_keyframe_search_num: 20
  history_keyframe_search_radius: 12.0
  history_keyframe_search_num: 5
  history_keyframe_fitness_score: 0.5
  ground_voxel_size: 0.3
```

建议流程：

1. 第一版先关闭或保守使用回环，确认不开回环的轨迹是稳定的。
2. 再打开回环。
3. `history_keyframe_fitness_score` 不要一开始设得太松，错误闭环比没有闭环更糟。
4. `distance_between_key_frame` 越小，地图细节越多，但 CPU、内存和 pose graph 压力越大。
5. 走廊、长直路、重复货架区域容易误闭环，需要用 pose graph editor 检查。

---

## 10. 推荐录包前检查命令

### 10.1 查找点云 topic

```bash
ros2 topic list | grep -E "points|lidar|cloud|velodyne|livox|rslidar|lslidar"
```

### 10.2 确认点云类型

```bash
ros2 topic info /your_lidar_points
```

期望类型：

```text
sensor_msgs/msg/PointCloud2
```

### 10.3 确认点云 frame

```bash
ros2 topic echo /your_lidar_points/header --once
```

重点看：

```yaml
frame_id: xxx
```

这个 `xxx` 必须和 TF 中 LiDAR frame 一致。

### 10.4 确认 odom

```bash
ros2 topic info /odom
ros2 topic echo /odom --once
```

期望类型：

```text
nav_msgs/msg/Odometry
```

### 10.5 确认核心 TF

如果使用 `base_link`：

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

如果使用 `base_footprint`：

```bash
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

这里的 `laser_link` 要替换成点云 `header.frame_id`。

---

## 11. 推荐离线建图录制命令模板

假设点云 topic 是 `/velodyne_points`，底盘里程计是 `/odom`：

```bash
ros2 bag record -o mapping_001 \
  /velodyne_points \
  /odom \
  /tf \
  /tf_static
```

如果是 Livox / Mid360：

```bash
ros2 bag record -o mapping_001 \
  /livox/lidar \
  /odom \
  /tf \
  /tf_static
```

如果暂时没有 odom：

```bash
ros2 bag record -o mapping_001 \
  /your_lidar_points \
  /tf \
  /tf_static
```

---

## 12. 推荐离线配置模板

```yaml
bag_reader:
  ros__parameters:
    use_sim_time: true
    bag_file_dir: "/data/bags/mapping_001"
    point_cloud_topic: "/velodyne_points"
    odometry_topic: "/odom"
    skip_frame: 1

lego_loam_ip:
  ros__parameters:
    use_sim_time: true
    laser:
      odom_type: "laser_odometry"
      base_ground_frame: "base_link"       # 或 base_footprint
      num_vertical_scans: 16
      num_horizontal_scans: 1000
      vertical_angle_bottom: -15.0
      vertical_angle_top: 15.0
      scan_period: 0.1

    imageProjection:
      minimum_detection_range: 0.5
      maximum_detection_range: 80.0
      stitcher_num: 1
      ground_fov_bottom: -0.2617994
      ground_fov_top: -0.0872660
      ground_positive_start: 0.0
      ground_positive_stop: 3.1415926
      ground_negative_start: 0.0
      ground_negative_stop: -3.1415926
```

launch 中静态 TF 示例：

```xml
<node pkg="tf2_ros"
      exec="static_transform_publisher"
      name="sensor2baselink"
      args="x y z yaw pitch roll base_link laser_link" />
```

`x y z yaw pitch roll` 必须使用实测外参；`laser_link` 必须和点云 `header.frame_id` 一致。

---

## 13. 离线建图验收清单

1. `ros2 bag info mapping_001` 能看到点云 topic。
2. 点云 topic 类型是 `sensor_msgs/msg/PointCloud2`。
3. 点云 `header.frame_id` 和静态 TF 的 child frame 一致。
4. `base_ground_frame -> lidar_frame` 能通过 `tf2_echo` 查到。
5. RViz 中点云方向正确：前后、左右、上下不能反。
6. 地面点提取连续，墙体和车体不要大量进入 ground cloud。
7. 小范围绕圈回到起点，轨迹不应明显跳飞。
8. 保存结果时不要只拿 `map.pcd`，要保存完整 pose graph 输出目录。
9. 输出目录应包含类似：

```text
map.pcd
ground.pcd
poses.pcd
edges.pcd
pcd/
```

10. 必要时用 pose graph editor 做手动闭环或合并地图。

---

## 14. 最容易踩坑的问题

### 14.1 点云 frame 和 TF child frame 不一致

例如：

```text
点云 header.frame_id = velodyne
静态 TF child frame = laser_link
```

这种情况下算法会查 `base_link -> velodyne`，而不是 `base_link -> laser_link`，导致 TF 查询失败。

---

### 14.2 以为录了 `/tf_static` 就自动生效

离线建图时仍应在 launch 中显式发布 LiDAR 静态外参，或者启动对应的 `robot_state_publisher` / static TF launch。

---

### 14.3 没录 `/odom`，后续又切到 `wheel_odometry`

如果后续要使用轮速里程计辅助建图，必须在录包时包含 `/odom`。

---

### 14.4 base frame 选错

如果 `base_link` 不在地面中心，建议使用 `base_footprint` 作为 `laser.base_ground_frame`，并保证：

```text
base_footprint -> laser_link
```

可查。

---

### 14.5 Ground FOV 没按安装姿态重算

水平安装、俯仰安装、倒装、侧装的 Ground FOV 完全不同。不要直接套 demo 参数。

---

## 15. 一句话总结

离线建图录包建议录：

```text
/点云 + /odom + /tf + /tf_static
```

建图运行时必须保证：

```text
laser.base_ground_frame -> 点云 header.frame_id
```

这条 TF 可查。第一轮建议使用 `laser_odometry` 跑通点云投影、地面提取和轨迹稳定性，再考虑接入 `wheel_odometry` 和回环优化。

---

## 16. 参考链接

- README：<https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam>
- 离线 bag launch：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_bag.launch>
- 在线 launch：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam.launch>
- C16 离线配置：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_c16_config.yaml>
- Mid360 离线配置：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_mid360_config.yaml>
- BagReader 实现：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/src/lego_loam_bag_node.cpp>
- ImageProjection 实现：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp>
- FeatureAssociation 实现：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/src/featureAssociation.cpp>
- MapOptimization 实现：<https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_lego_loam/lego_loam_bor/src/mapOptimization.cpp>
