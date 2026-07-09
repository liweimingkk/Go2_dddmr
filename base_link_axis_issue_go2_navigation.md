# Go2 / DDDMR 建图与导航中的 `base_link` 坐标轴问题处理说明

## 1. 问题背景

当前现象：

```text
在 base_link 坐标系中，机器狗头部前方对应的是 -Y 轴方向。
```

这意味着当前 `base_link` 不符合 ROS 地面机器人常用坐标系约定。标准约定通常是：

```text
base_link:
  +X = 机器人前方
  +Y = 机器人左侧
  +Z = 机器人上方
```

对于后续 DDDMR / LeGO-LOAM 建图、MCL 定位、Nav2 路径规划、局部控制器、`cmd_vel` 下发、footprint 配置、代价地图避障等模块，`base_link` 方向错误会成为系统性隐患。

结论：

```text
不建议继续带着“狗头前方 = base_link -Y”的坐标系做导航适配。
应该尽快将公开给 ROS / Nav2 / DDDMR 的 base_link 或 base_footprint 统一为标准方向。
```

---

## 2. 为什么这会有问题

### 2.1 对建图的影响

如果 LiDAR 静态外参、Z 轴方向、地面方向在当前错误坐标系下是自洽的，LeGO-LOAM 可能仍然可以建出地图。

但是这只是“能跑”，不代表系统坐标语义正确。后续会在以下环节暴露问题：

```text
1. 地图坐标朝向和机器人真实朝向不一致
2. LiDAR ground_fov 调参容易绕错方向
3. pose graph 中的机器人朝向和实际狗头方向差 90°
4. 后续 MCL 初始位姿方向不直观
5. RViz 中导航目标箭头方向和机器狗真实前方不一致
6. Nav2 控制器输出的 cmd_vel 与底层机器狗运动方向不一致
```

DDDMR 的 LeGO-LOAM 改版本身强调倾斜 LiDAR 需要正确 static TF，并且如果 `base_link` 不在地面，建议增加 `base_footprint`。因此建图阶段就应该把机器人基准 frame 调正确。

### 2.2 对定位的影响

定位系统通常认为：

```text
map -> odom -> base_link
```

其中 `base_link` 是机器人机体坐标系。如果 `base_link +X` 不是狗头前方，则会出现：

```text
1. RViz 中机器人箭头方向和真实狗头方向不一致
2. 给定初始位姿时方向容易错 90°
3. MCL 看似收敛，但机器人朝向语义错误
4. 后续导航路径方向和实际运动方向对不上
```

### 2.3 对导航控制的影响

Nav2 / ROS 控制链路一般默认：

```text
/cmd_vel.linear.x > 0  表示机器人沿 base_link +X 前进
/cmd_vel.linear.y > 0  表示机器人沿 base_link +Y 左移
/cmd_vel.angular.z > 0 表示机器人逆时针左转
```

如果当前系统中：

```text
狗头前进方向 = base_link -Y
```

那么 Nav2 发出的“前进”命令可能被底层解释为横移，或者底层驱动必须再做一次坐标转换。对于 Unitree Go2 这类四足机器人，`linear.y` 也可能被用于横移，因此坐标系错误会严重影响运动控制。

---

## 3. 首先判断：到底是 `base_link` 错，还是 LiDAR 外参错

你看到“狗头前方在 `base_link` 里是 -Y”，可能有两种情况。

### 情况 A：`base_link` 本身定义错了

在 RViz 中显示 `base_link` 坐标轴，如果看到：

```text
红色 X 轴没有指向狗头
绿色 Y 轴没有指向狗左侧
蓝色 Z 轴没有向上
```

说明 `base_link` frame 本身不符合 ROS 约定。

这种情况应该修 `base_link` 定义。

### 情况 B：`base_link` 是对的，但 LiDAR 外参错了

如果在 RViz 中：

```text
base_link 红色 X 轴确实指向狗头
base_link 绿色 Y 轴确实指向狗左侧
base_link 蓝色 Z 轴确实向上
```

但是点云中“前方障碍物”落在 `base_link -Y`，则说明问题不是 `base_link`，而是：

```text
base_link -> lidar_frame
```

这条 LiDAR 静态外参的 yaw / pitch / roll 写错了。

建议检查命令：

```bash
ros2 topic echo /your_lidar_points/header --once
ros2 run tf2_ros tf2_echo base_link <点云header里的frame_id>
```

同时在 RViz 中添加：

```text
TF
Axes: base_link
Axes: lidar_frame
PointCloud2: /your_lidar_points
```

---

## 4. 推荐的标准 TF 结构

对于机器狗 / 地面机器人，推荐使用：

```text
map
└── odom
    └── base_footprint    # 地面投影 frame，主要用于建图、定位、导航
        └── base_link     # 机身 frame，可包含高度、roll、pitch
            └── lidar_link
```

如果当前项目暂时不区分 `base_footprint` 和 `base_link`，最低限度也要保证：

```text
base_link:
  +X = 狗头方向
  +Y = 狗左侧
  +Z = 上方
```

DDDMR / LeGO-LOAM 配置中建议：

```yaml
laser:
  base_ground_frame: "base_footprint"
```

如果暂时没有 `base_footprint`，则使用：

```yaml
laser:
  base_ground_frame: "base_link"
```

前提是该 frame 必须符合标准方向。

---

## 5. 如果当前旧坐标系是“狗头 = -Y”，应该如何旋转

假设当前错误 frame 叫：

```text
base_link_raw
```

并且它满足：

```text
狗头方向 = -Y_raw
Z_raw = 向上
```

现在要创建一个标准 `base_link`，使其满足：

```text
+X_base_link = 狗头方向 = -Y_raw
+Y_base_link = 狗左侧 = +X_raw
+Z_base_link = +Z_raw
```

从旧 frame `base_link_raw` 到新 frame `base_link`，可以发布一个绕 Z 轴 **-90°** 的静态旋转：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw -1.57079632679 \
  --frame-id base_link_raw \
  --child-frame-id base_link
```

对应 TF：

```text
base_link_raw
└── base_link
```

注意：这更适合作为临时补救。更推荐从源头把旧的错误 frame 改名为：

```text
base_link_raw
base_sdk
body_raw
```

然后把标准化后的 frame 命名为真正的：

```text
base_link
```

不要让系统里同时存在两个语义不同但都叫 `base_link` 的 frame。

---

## 6. 工程处理方案

### 6.1 方案一：从源头修正，推荐

将机器狗 SDK、URDF、odom、LiDAR 外参统一改成标准 ROS 坐标系：

```text
base_link:
  +X forward
  +Y left
  +Z up
```

最终应保证：

```text
/odom:
  header.frame_id = odom
  child_frame_id  = base_link 或 base_footprint

/tf:
  odom -> base_link 或 odom -> base_footprint

/tf_static 或 robot_state_publisher:
  base_link -> lidar_link
  base_link -> imu_link
  base_link -> camera_link
```

检查命令：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

验收标准：

```text
1. 狗向前走时，odom 中主要体现为 x 方向前进
2. RViz 中 base_link 红色 X 轴指向狗头
3. RViz 中 base_link 绿色 Y 轴指向狗左侧
4. RViz 中 base_link 蓝色 Z 轴向上
5. Nav2 或手动发布 cmd_vel.linear.x > 0 时，机器狗头朝前走
```

### 6.2 方案二：增加 ROS 适配层，适合不能改 SDK 的情况

如果 Unitree SDK 或底层驱动固定使用旧坐标系，无法从源头修改，则不要让旧坐标系继续叫 `base_link`。

建议：

```text
旧坐标系命名为：base_link_raw 或 base_sdk
标准 ROS 坐标系命名为：base_link
```

一种临时 TF 结构：

```text
odom
└── base_link_raw
    └── base_link
        └── lidar_link
```

更推荐的结构是：

```text
odom
└── base_link
    ├── base_link_raw
    └── lidar_link
```

第二种更干净，但前提是你能让 odom 直接发布到标准 `base_link`。

---

## 7. `/cmd_vel` 也必须转换

如果 Nav2 发出的是标准 ROS 坐标：

```text
vx_ros = msg.linear.x   # 前进
vy_ros = msg.linear.y   # 左移
wz_ros = msg.angular.z  # 左转
```

而底层旧坐标满足：

```text
狗头前进 = -Y_raw
Z_raw = 向上
```

那么从标准 ROS 命令转换到底层 raw 命令，大致为：

```python
vx_raw =  vy_ros
vy_raw = -vx_ros
wz_raw =  wz_ros
```

也就是说：

```text
ROS 想前进：vx_ros > 0
底层应收到：vy_raw < 0
```

如果底层 yaw 正方向也与 ROS 相反，则 `wz_raw` 也需要取负。这个必须通过实车小速度 dry-run / no-motion 安全模式验证。

建议测试命令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

期望：

```text
机器狗头朝前走
```

测试左移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

期望：

```text
机器狗向左横移
```

测试左转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

期望：

```text
机器狗原地逆时针左转
```

---

## 8. `/odom` 不能只靠静态 TF 糊过去

这是容易被忽略的问题。

如果当前 `/odom` 是旧坐标输出，例如：

```text
/odom.header.frame_id = odom
/odom.child_frame_id  = base_link
```

但这个 `base_link` 实际上是错误方向，那么不能只加一条静态 TF 就认为导航系统正确了。

`nav_msgs/Odometry` 中的：

```text
pose  表示 child_frame 在 odom 下的位姿
twist 表示 child_frame 下的速度
```

因此必须让 `/odom.child_frame_id` 对应标准 `base_link`，或者发布一个转换后的 `/odom_standard`。

如果旧 raw 坐标中：

```text
狗头前进 = -Y_raw
Z_raw = 向上
```

则 raw 速度转标准速度大致为：

```python
vx_ros = -vy_raw
vy_ros =  vx_raw
wz_ros =  wz_raw
```

如果 yaw 正方向相反，`wz_ros` 也要取负。

---

## 9. 对当前离线建图的建议

如果你还没有采集最终地图，建议：

```text
先修 base_link / base_footprint / lidar TF
再用同一份 bag 重新离线建图
```

如果 bag 中录的是原始点云，只要点云 `header.frame_id` 和离线 launch 中的静态 TF 能修正，通常不需要重新录 bag。

如果 bag 中的 `/odom` 已经使用错误 `base_link` 写死，并且你想用 `wheel_odometry`，建议二选一：

```text
方案 A：第一轮先用 laser_odometry，不使用旧 /odom
方案 B：写一个离线转换节点，将 /odom 转成标准坐标后再建图
```

DDDMR / LeGO-LOAM 的离线建图中，IMU / Odometry 是可选输入。因此 `/odom` 坐标不可信时，第一轮可以先使用纯激光里程计建图。

---

## 10. 推荐修改顺序

### 第一步：确认 base_link 轴是否真的错

```bash
ros2 run tf2_ros view_frames
```

RViz 中添加：

```text
TF
Axes: base_link
Axes: lidar_link
RobotModel，如果有 URDF
PointCloud2: /your_lidar_points
```

确认：

```text
base_link 红色 X 是否指向狗头
base_link 绿色 Y 是否指向狗左侧
base_link 蓝色 Z 是否向上
```

### 第二步：把错误 frame 改名

不要继续让错误方向的 frame 叫 `base_link`。

建议命名：

```text
base_link_raw
base_sdk
body_raw
```

然后创建标准：

```text
base_link
```

如有必要，再创建：

```text
base_footprint
```

### 第三步：修改 DDDMR 建图配置

如果使用 `base_footprint`：

```yaml
laser:
  base_ground_frame: "base_footprint"
```

如果暂时只使用 `base_link`：

```yaml
laser:
  base_ground_frame: "base_link"
```

然后验证：

```bash
ros2 topic echo /your_lidar_points/header --once
ros2 run tf2_ros tf2_echo base_link <点云header里的frame_id>
```

或：

```bash
ros2 run tf2_ros tf2_echo base_footprint <点云header里的frame_id>
```

### 第四步：重算 LiDAR 外参

修正 base frame 后，原来的 LiDAR static TF 不能盲目照搬。

最终必须保证：

```text
base_ground_frame -> lidar_frame
```

可以查到，且方向正确。

### 第五步：修 cmd_vel 驱动接口

标准化后应满足：

```text
cmd_vel.linear.x > 0  狗头向前
cmd_vel.linear.y > 0  狗向左横移
cmd_vel.angular.z > 0 狗逆时针左转
```

### 第六步：修 odom 输出

最终建议：

```text
odom -> base_footprint -> base_link
```

或最低限度：

```text
odom -> base_link
```

其中 `base_link` 必须是标准方向。

---

## 11. 最终验收清单

### TF 验收

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros view_frames
```

RViz 中检查：

```text
base_link +X 指向狗头
base_link +Y 指向狗左侧
base_link +Z 向上
lidar_link 与真实雷达安装方向一致
点云前方障碍物位于 base_link +X 方向
```

### cmd_vel 验收

```text
linear.x > 0  实际向前
linear.y > 0  实际向左
angular.z > 0 实际逆时针左转
```

### odom 验收

```text
狗头向前走时，odom 轨迹应主要沿前进方向变化
odom.child_frame_id 应该是标准 base_link 或 base_footprint
odom twist 方向应与标准 ROS 坐标一致
```

### DDDMR 建图验收

```text
laser.base_ground_frame 设置为标准 base_link 或 base_footprint
base_ground_frame -> lidar_frame 可查到
原始点云方向正确
ground cloud 提取方向正确
离线建图轨迹方向与机器狗运动方向一致
```

### Nav2 验收

```text
robot_base_frame 设置为标准 base_link 或 base_footprint
footprint 方向和机器人实际外形一致
局部代价地图中障碍物位于正确方位
给定导航目标后，机器人不会横着走或倒着走
```

---

## 12. 简短结论

当前：

```text
狗头前方 = base_link -Y
```

这是不推荐的。它可能让建图阶段勉强能跑，但会给后续定位、导航、控制、避障和地图复用带来固定 90° 坐标误差。

建议最终统一为：

```text
base_link / base_footprint:
  +X = 狗头方向
  +Y = 狗左侧
  +Z = 上方
```

如果底层 SDK 不能改，则将旧 frame 改名为：

```text
base_link_raw 或 base_sdk
```

再发布标准 `base_link`。在“狗头 = -Y_raw，Z 向上”的情况下，可临时使用：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw -1.57079632679 \
  --frame-id base_link_raw \
  --child-frame-id base_link
```

随后必须同步修正：

```text
1. /odom
2. /cmd_vel
3. base_link -> lidar_link
4. DDDMR 的 laser.base_ground_frame
5. Nav2 的 robot_base_frame
6. footprint / costmap 参数
```

最稳路线：

```text
先统一 base_link / base_footprint 坐标系
再用同一份 bag 重新离线建图
最后接 DDDMR MCL / Nav2 导航
```

---

## 13. 参考资料

- REP-103: Standard Units of Measure and Coordinate Conventions  
  https://reps.openrobotics.org/rep-0103/

- REP-105: Coordinate Frames for Mobile Platforms  
  https://reps.openrobotics.org/rep-0105/

- Nav2 Transform Setup Guide  
  https://docs.nav2.org/setup_guides/transformation/setup_transforms.html

- DDDMR Navigation / dddmr_lego_loam README  
  https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam
