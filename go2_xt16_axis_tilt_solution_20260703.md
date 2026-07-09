# Go2 + XT16 DDDMR 轴向 / 地图倾斜问题分析与解决方案

## 0. 结论

当前问题不要继续用“把 odom 再旋转 ±90°”来修。上传包里的证据更支持下面这个判断：

```text
/utlidar/robot_odom 原始 odom 在前进方向上已经基本自洽；
当前 /dddmr_go2/robot_odom_standard 的 ±90° 标准化反而会破坏“位移方向”和“yaw 朝向”的一致性；
RViz 中“物理向前看起来沿 map -Y 走”本身不能证明轴错，因为 map 坐标轴是地图坐标，不等于机器人初始朝向；
真正需要修的是：不要用 ±90° odom standardization；先让 odom 变成 pass-through，再处理 MCL 产生的 map->odom roll/pitch 倾斜。
```

建议状态：

```text
live navigation: supervised Gate4 PASS at 0.30m/s; future motion still requires onsite approval
observation-only RViz: OK
短距离 Sport probe: 0.10m/s adapter handshake only, still requires supervision
继续点 RViz live goal: 只按已审计 supervised wrapper 执行，不做无人值守闭环
```

当前权威状态（2026-07-04）：

```text
report shape: PASS
runtime clean: PASS
acceptance: PASS
completion: PASS
```

以 current evidence、verifier 输出和最新 Gate4 0.30 记录为准：

```text
/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
```

当前 4 个 gate：

```text
Gate 1 RViz base_link 前方已知物体检查: PASS
Gate 2 initial-pose TF tilt 复查: PASS
Gate 3 raw + standard odom axis probe: PASS
Gate 4 supervised live short goal at 0.30m/s: PASS
```

说明：

```text
前文的历史 probe / dry-run PASS 只证明对应阶段的局部结论；
是否能把本报告标记为完成，只看 current evidence + verifier：
GO2_XT16_ACCEPTANCE_STATUS=PASS。
当前 verifier 输出是 PASS。
```

---

## 1. 当前证据复盘

### 1.1 运行环境是安全观察模式

当前观察 run 是：

```text
scripts/dddmr_docker_go2_xt16.sh navigation-live-source
rviz:=true
publish_static_tf:=true
start_sport_dry_run_adapter:=false
start_go2_sport_adapter:=false
```

也就是说，这个 run 本身不应该由本仓库向 `/api/sport/request` 发真实运动命令。

---

### 1.2 当前 active TF

当前静态 TF：

```text
base_link -> hesai_lidar:
  xyz = 0.145, 0.000, 0.133
  rpy = 0, 0, +90 deg
```

当前 odom TF：

```text
odom -> base_link:
  xyz = -0.257, -0.723, 0.301
  rpy = 1.54 deg, 2.52 deg, 5.00 deg
```

当前 map TF：

```text
map -> base_link:
  xyz = 0.579, -0.073, -0.273
  rpy = 13.35 deg, 2.88 deg, -70.26 deg
```

由这两个 TF 可以反推出：

```text
map -> odom 约等于：
  roll  ≈ 11.7 deg
  pitch ≈ 1.4 deg
  yaw   ≈ -75.7 deg
```

所以“地图看起来倾斜”的直接原因不是 `odom -> base_link` 的 1–3° 机身姿态，而是 localization / `map -> odom` 又额外引入了大约 12° 的 roll。

---

### 1.3 轴向 probe 证明 raw odom 比 standard odom 更可信

之前的轴向 probe 使用：

```text
Sport Move(x=0.30, y=0, z=0)，持续 0.6s
```

测得 raw `/utlidar/robot_odom` 位移：

```text
RAW_move_dx = +0.0864
RAW_move_dy = +0.1154
raw yaw     ≈ 45.7 deg
位移方向     ≈ 53.2 deg
位移方向与 yaw 差值 ≈ 7.5 deg
```

这说明 raw odom 的“运动方向”和 raw yaw 基本一致。换句话说，raw `/utlidar/robot_odom` 在 `base_link +X = 物理前方` 这件事上并没有明显错误。

同一次记录里的 standardized odom 位移是：

```text
STANDARD_move_dx = -0.1136
STANDARD_move_dy = +0.0839
standard yaw     ≈ 45.7 deg
位移方向          ≈ 143.6 deg
位移方向与 yaw 差值 ≈ 97.9 deg
```

这说明旧的 standardization 已经把“位移方向”和“yaw 朝向”转到近乎垂直。

更关键的是，当前代码把 `raw_to_standard_yaw` 改成 `+90deg` 后，按照现有 `go2_odom_standardizer_node.cpp` 的数学逻辑，位置会被旋到另一个方向，预期也会产生大约 80° 以上的 heading mismatch。也就是说：

```text
raw_to_standard_yaw = -90 deg: 错
raw_to_standard_yaw = +90 deg: 仍然错
raw_to_standard_yaw = 0 deg / pass-through: 当前证据下最合理
```

---

## 2. 对几个核心问题的回答

### 2.1 正确坐标约定应该是什么？

本项目导航侧应该统一成：

```text
base_link +X = 机器狗头部物理前方
base_link +Y = 机器狗左侧
base_link +Z = 向上
cmd_vel.linear.x > 0 = 让 Go2 向头部前方走
cmd_vel.angular.z > 0 = ROS 约定的逆时针 yaw
```

Go2 Sport API adapter 中当前 `axis_mode=standard`：

```text
Twist.linear.x  -> Sport Move x
Twist.linear.y  -> Sport Move y，当前建议保持 max_y=0
Twist.angular.z -> Sport Move z
```

这个方向不应该因为 RViz 里看起来沿 `map -Y` 移动就改掉。

---

### 2.2 `raw_to_standard_yaw` 应该是 +90、-90，还是移除？

当前证据下应该移除，或者设为 0：

```yaml
raw_to_standard_yaw: 0.0
rotate_parent_frame: false
rotate_twist: false
```

为了不大改 topic graph，可以保留 `/dddmr_go2/robot_odom_standard` 这个 topic 名字，但让 `go2_odom_standardizer` 只做 pass-through：

```text
/utlidar/robot_odom
  -> go2_odom_standardizer pass-through
  -> /dddmr_go2/robot_odom_standard
```

如果更想减少中间节点，也可以直接让所有消费者用 raw odom：

```text
odom_topic := /utlidar/robot_odom
standardize_odom := false
```

但这需要同步改：

```text
mcl_feature remap /odom
mcl_3dl remap /odom
local_planner.odom_topic
任何脚本里的 ODOM_TOPIC
```

所以最小风险方案是：**保留 topic，标准化节点改成默认 pass-through。**

---

### 2.3 `go2_imu -> hesai_lidar` 的 +90° yaw 要不要保留？

这个不能用 odom probe 直接决定。它和 odom standardization 是两件事：

```text
odom standardization: 处理 /utlidar/robot_odom 的坐标约定
LiDAR static TF:      处理 /lidar_points 点云 frame 到 base_link 的几何关系
```

所以不要因为 odom 有问题就直接删 LiDAR 的 +90°。

`go2_imu -> hesai_lidar yaw=+90°` 应该这样验证：

```text
RViz Fixed Frame = base_link
显示 /lidar_points, /utlidar/cloud_base, TF, RobotModel
在机器狗正前方放箱子或墙面
看这个物体是否落在 base_link +X 方向
```

如果 XT16 点云中“物理前方”经过 TF 后确实是 `base_link +X`，就保留 +90°。这时它只是 Hesai driver 点云坐标轴修正，不是机械外参。你之前图片里 XT16 相对 Go2 IMU 的机械外参 yaw=0，并不一定等于 driver 输出点云 frame 的 yaw=0。

如果前方物体落在 `base_link +Y`、`-Y` 或 `-X`，再改 `lidar_yaw`。

---

### 2.4 为什么 physical forward 在 RViz 里像沿 `map -Y`？

这不一定是错。

当前 `map -> base_link` 的 yaw 约为：

```text
-70 deg
```

机器人在 `map` 中朝向 -70° 时，向自身前方走，在 map 坐标里的方向向量大约是：

```text
[cos(-70°), sin(-70°)] ≈ [0.34, -0.94]
```

视觉上就会主要沿 `map -Y` 移动。

所以：

```text
“向前看起来沿 map -Y”本身不是错误；
错误要看 base_link +X 是否是物理前方，以及 odom 位移是否与 base yaw 一致。
```

当前 raw odom probe 已经说明 raw odom 在这一点上基本一致。

---

### 2.5 为什么 `map -> base_link` 有 13° roll？

因为 localization / MCL 发布的 `map -> odom` 吸收了大约 11–12° 的 roll。

当前：

```text
odom -> base_link roll ≈ 1.5 deg
map  -> base_link roll ≈ 13.3 deg
```

两者差值主要来自：

```text
map -> odom roll ≈ 11.7 deg
```

可能原因：

```text
1. MCL 使用 6DoF 粒子，roll/pitch/z 搜索空间过大；
2. odom 输入被 ±90° 标准化后与 yaw/位移不一致，MCL 通过 map->odom 倾斜来吸收误差；
3. 初始 3D pose 的 z/roll/pitch 和建图时坐标不一致；
4. base_link 不是地面 frame，但当前导航直接以 base_link 做 robot_frame；
5. 当前 base_link->base_footprint 是 static child，不是真正去 roll/pitch 的 footprint frame。
```

---

## 3. 必改项

### 3.1 不要再用 ±90° odom standardization

修改：

```text
src/dddmr_lego_loam/lego_loam_bor/src/go2_odom_standardizer_node.cpp
src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_live.launch
src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_mouth.launch
```

建议默认：

```cpp
raw_to_standard_yaw_ = declare_parameter<double>("raw_to_standard_yaw", 0.0);
rotate_parent_frame_ = declare_parameter<bool>("rotate_parent_frame", false);
rotate_twist_ = declare_parameter<bool>("rotate_twist", false);
```

launch 默认：

```xml
<arg name="raw_to_standard_yaw" default="0.0"/>
```

并且显式传：

```xml
<param name="raw_to_standard_yaw" value="$(var raw_to_standard_yaw)"/>
<param name="rotate_parent_frame" value="false"/>
<param name="rotate_twist" value="false"/>
```

如果 Codex 希望保留实验功能，可以加一个强 warning：

```cpp
if (std::abs(raw_to_standard_yaw_) > 1e-6) {
  RCLCPP_WARN(
    get_logger(),
    "raw_to_standard_yaw is nonzero. Do not use this for /utlidar/robot_odom unless an axis probe proves the raw odom frame is not standard."
  );
}
```

---

### 3.2 增加 odom 自洽检查工具

新增脚本：

```text
scripts/check_go2_odom_axis_consistency.py
```

输入：

```text
--odom-topic /utlidar/robot_odom
--duration 3
```

它计算一段运动窗口内：

```python
move_dir = atan2(dy, dx)
yaw_mid = average_yaw
heading_error = shortest_angle(move_dir - yaw_mid)
forward = dx*cos(yaw_mid) + dy*sin(yaw_mid)
lateral = -dx*sin(yaw_mid) + dy*cos(yaw_mid)
```

通过条件建议：

```text
forward > 0.05 m
abs(heading_error) < 20 deg
abs(lateral) / max(abs(forward), 1e-6) < 0.35
```

根据已有 probe：

```text
raw /utlidar/robot_odom: PASS
old /dddmr_go2/robot_odom_standard with ±90: FAIL
```

---

### 3.3 增加 TF tilt 健康检查

新增脚本：

```text
scripts/check_go2_xt16_tf_health.py
```

检查：

```text
base_link -> hesai_lidar
odom -> base_link
map -> odom
map -> base_link
```

建议阈值：

```text
odom->base_link roll/pitch: 站立静止时 < 5 deg
map->odom roll/pitch:       < 3 deg
map->base_link roll/pitch:  站立静止时 < 6 deg
```

如果出现：

```text
map->odom roll/pitch > 5 deg
```

直接判定：

```text
禁止 live navigation
需要重置 initial pose 或收紧 MCL roll/pitch/z
```

---

### 3.4 MCL 暂时收紧 roll/pitch/z 搜索

在 `go2_xt16_navigation.yaml` 中，先保守收紧：

```yaml
mcl_3dl:
  ros__parameters:
    init_var_z: 0.2
    init_var_roll: 0.02
    init_var_pitch: 0.02
    init_var_yaw: 0.5

    resample_var_z: 0.05
    resample_var_roll: 0.02
    resample_var_pitch: 0.02
    resample_var_yaw: 0.1

    expansion_var_z: 0.05
    expansion_var_roll: 0.02
    expansion_var_pitch: 0.02
    expansion_var_yaw: 0.2
```

不要先把 yaw 收得太小，因为 map 初始 yaw 仍需要用户 2D pose estimate / 3D pose estimate 来对齐。

如果收紧后 `map->odom roll/pitch` 仍然飘到 10° 以上，说明需要更明确地做 2.5D localization：

```text
定位估计 x/y/yaw；
roll/pitch 主要来自 odom/IMU，不允许 MCL 用 map->odom 去额外倾斜世界。
```

---

### 3.5 当前 `base_footprint` 不是有效 footprint

现在 launch 中：

```xml
base_link -> base_footprint:
  xyz = 0,0,-0.24
  rpy = 0,0,0
```

这是 `base_link` 的 static child。它只是把原点往下平移，不会去掉 `base_link` 的 roll/pitch。

真正的 footprint 应该是：

```text
odom/map -> base_footprint: x/y/yaw only, roll=pitch=0
base_footprint -> base_link: body height + roll/pitch information
```

这不是这轮最小修复的必需项，但不要误以为当前 static `base_footprint` 能解决地图倾斜。

---

## 4. 次要但必须修的导航参数

### 4.1 `oscillation_distance: 5.0` 对当前短距离测试不合理

当前配置：

```yaml
oscillation_distance: 5.0
oscillation_patience: 15.0
```

这会导致机器人即使走了 0.3–0.5m，也可能被判定为“15s 内没有足够进展”。日志里已经出现：

```text
Oscillation time out is detected: 18.40 secs for 0.38 m.
```

短距离导航测试建议改成：

```yaml
oscillation_distance: 0.05
oscillation_angle: 0.3
oscillation_patience: 15.0
```

或者第一轮稳定性测试先禁用 aggressive recovery，但不要在真实机器人上长时间忽略 recovery。

---

### 4.2 不要保留 `x=0.50` 作为默认 live 测试速度

当前 config 中：

```yaml
min_vel_x: 0.50
max_vel_x: 0.50
```

这对调轴向/定位太激进。已有证据显示：

```text
x=0.05: 能产生短距离位移
x=0.10: 接近颤动 / 阈值边缘
x=0.30: 首次明显 closed-loop walking
x=0.50: 不适合作为定位/轴向修复阶段默认值
```

建议在 axis/tilt 修复验证阶段：

```yaml
differential_drive_simple:
  min_vel_x: 0.30
  max_vel_x: 0.30
  acc_lim_x: 3.00
```

同时 gate / adapter：

```text
max_x = 0.30
max_y = 0.0
max_yaw = 0.25
```

等定位稳定后，再逐步调速度。

---

## 5. 推荐修改方案给 Codex

直接交给 Codex：

```text
Problem: Go2 XT16 live navigation has a frame/axis/tilt issue. Do not continue live navigation until odom axis and map tilt are validated.

Core findings:
- Raw /utlidar/robot_odom is already heading-consistent during Move(x=0.30).
- The ±90deg go2_odom_standardizer output breaks displacement-vs-yaw consistency.
- map -Y forward motion in RViz is not proof of an axis bug because map yaw can be arbitrary.
- map->base_link roll ~13deg is caused mostly by map->odom roll ~12deg from localization and must be guarded.

Required code/config changes:

1. Make go2_odom_standardizer default to pass-through.
   In go2_odom_standardizer_node.cpp:
     raw_to_standard_yaw default = 0.0
     rotate_parent_frame default = false
     rotate_twist default = false
   Keep /dddmr_go2/robot_odom_standard as a stable topic name, but do not rotate raw odom by default.
   Add a warning if raw_to_standard_yaw is nonzero.

2. Update all Go2 XT16 launch defaults:
     raw_to_standard_yaw default = 0.0
   Files:
     src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
     src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_live.launch
     src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_mouth.launch

3. Do not change go2_imu -> hesai_lidar yaw based on odom results.
   Keep lidar_yaw=+90 only if RViz Fixed Frame=base_link shows objects physically in front of the dog at base_link +X.
   If that check fails, fix lidar_yaw separately.

4. Add scripts/check_go2_odom_axis_consistency.py.
   It should verify that odom displacement direction matches odom yaw during a short forward probe.
   Raw /utlidar/robot_odom should pass; /dddmr_go2/robot_odom_standard should also pass after pass-through.

5. Add scripts/check_go2_xt16_tf_health.py.
   It should fail if map->odom roll/pitch exceeds about 3deg or map->base_link roll/pitch exceeds about 6deg while standing still.

6. Tighten mcl_3dl roll/pitch/z variance in go2_xt16_navigation.yaml:
     init_var_roll/pitch: 0.02
     resample_var_roll/pitch: 0.02
     expansion_var_roll/pitch: 0.02
     resample_var_z: 0.05
     expansion_var_z: 0.05
   Keep yaw variance usable for initial pose alignment.

7. Fix short-distance recovery tuning:
     oscillation_distance: 0.05
   not 5.0.

8. Revert live speed default from x=0.50 to the supervised walking threshold x=0.30:
     min_vel_x: 0.30
     max_vel_x: 0.30
     gate max_x: 0.30
     adapter max_x: 0.30
   Keep max_y=0.0.

9. Do not run live RViz goals until the following checks pass:
   - odom axis consistency PASS
   - RViz base_link point-cloud direction check PASS
   - map->odom roll/pitch health PASS
   - dry-run goal generates reasonable /dddmr_go2/dry_run_cmd_vel
```

---

## 6. 下一步验证顺序

### Step 1：无运动，确认没有 live adapter

```bash
ros2 topic info /api/sport/request
ps -eo pid,ppid,pgid,cmd | rg -i 'go2_sport_cmd_vel_adapter|ros2 topic pub /api/sport/request' || true
```

确认当前 observation run 没有本仓库的 live adapter。

---

### Step 2：pass-through odom 启动 observation-only

启动：

```bash
scripts/dddmr_docker_go2_xt16.sh navigation-live-source \
  rviz:=true \
  publish_static_tf:=true \
  start_sport_dry_run_adapter:=false \
  start_go2_sport_adapter:=false \
  raw_to_standard_yaw:=0.0
```

然后检查：

```bash
ros2 topic echo /dddmr_go2/robot_odom_standard --once
ros2 topic echo /utlidar/robot_odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

期望：

```text
/dddmr_go2/robot_odom_standard 和 /utlidar/robot_odom 的 pose/twist 基本一致。
odom->base_link roll/pitch 在站立静止时只有几度以内。
```

---

### Step 3：RViz 固定 `base_link` 看双雷达方向

RViz：

```text
Fixed Frame = base_link
显示 /lidar_points
显示 /utlidar/cloud_base
显示 TF
显示 RobotModel
```

在机器狗正前方放箱子或墙面。通过条件：

```text
XT16 和嘴部雷达看到的前方物体都应该落在 base_link +X 方向；
不能出现 90° 旋转、左右镜像、前后颠倒。
```

如果 XT16 不对，只改 `lidar_yaw`，不要改 odom。

---

### Step 4：设置初始 pose 后看 map tilt

在 RViz 中给初始位姿后，立刻查：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

通过条件：

```text
map->odom roll/pitch < 3 deg
map->base_link roll/pitch 站立时 < 6 deg
```

如果 `map->odom roll` 仍然是 10° 以上，禁止 live navigation。先调 MCL roll/pitch/z，或重置初始 pose。

---

### Step 5：短轴向 probe

只有在人工监督和急停可用时做：

```bash
GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE \
GO2_YAW_PROBE_X=0.30 \
GO2_YAW_PROBE_Y=0.0 \
GO2_YAW_PROBE_YAW=0.0 \
GO2_YAW_PROBE_DURATION=0.6 \
GO2_YAW_PROBE_PRE_BALANCE_STAND=true \
GO2_YAW_PROBE_PRE_BALANCE_DELAY=0.8 \
GO2_YAW_PROBE_ANALYZE=true \
RVIZ=false \
PUBLISH_STATIC_TF=true \
scripts/run_go2_yaw_feedback_probe.sh --live
```

然后用 axis consistency 脚本分析 raw 和 standard odom。

通过条件：

```text
raw odom PASS
standard odom PASS
standard odom 不再相对 raw 旋转 90°
```

---

### Step 6：dry-run goal

仍然不要 live adapter：

```text
start_go2_sport_adapter:=false
start_sport_dry_run_adapter:=true 或只看 /dddmr_go2/dry_run_cmd_vel
```

点 0.3–0.5m 目标，看：

```text
/global_path
/awared_global_path
/prune_plan
/dddmr_go2/dry_run_cmd_vel
/dddmr_go2/safe_cmd_vel
/mcl_pose
map->base_link TF
```

通过条件：

```text
path 不应突然跳到奇怪方向；
cmd_vel.forward 为正时，RViz 中机器人朝自身 +X 方向前进；
map->odom 不应产生大 roll/pitch；
```

---

### Step 7：再恢复 supervised live

只有前面都通过，再开启：

```text
max_x = 0.30
max_y = 0.0
max_yaw = 0.25
目标距离 0.3–0.5m
人工急停
```

不要在 axis/tilt 还没稳定时继续用 `x=0.50`。

---

## 7. 本轮不要做的事

```text
1. 不要为了让 RViz 中 forward 看起来沿 map +X 而旋转 odom。
2. 不要同时改 raw_to_standard_yaw 和 lidar_yaw。
3. 不要把 go2_imu->hesai_lidar 的 +90 当成 odom 的 +90。
4. 不要用 static base_link->base_footprint 解决 roll/pitch；它不能去倾斜。
5. 不要在 map->odom roll/pitch 仍然 10° 以上时 live navigation。
6. 不要用 x=0.50 继续调定位和坐标轴。
```

---

## 8. 最终判断

当前最可能的主因是：

```text
/utlidar/robot_odom 原始数据已经基本符合 base_link +X 前进；
go2_odom_standardizer 的 ±90° 旋转破坏了 odom 位移和 yaw 的一致性；
MCL/localization 又用 map->odom 的 roll/pitch/yaw 去吸收这些不一致，造成地图在 RViz 中倾斜和路径跟踪异常。
```

最小修复路线：

```text
odom standardizer pass-through
  -> RViz base_link 双雷达方向验证
  -> map->odom roll/pitch 健康检查
  -> MCL roll/pitch/z 收紧
  -> dry-run goal
  -> supervised 0.30m/s live short goal
```

通过这些之后，再回到 path tracking、collision critic、recovery behavior 的问题。现在不要继续在坐标系未稳定的情况下调 local planner。

---

## 9. 2026-07-03 本轮实施状态

本轮已经把上面的最小修复路线落到 `/home/lin/new2/dddmr_navigation`，并且没有执行真实运动命令。

### 9.1 已修改项

1. `go2_odom_standardizer` 默认改成 pass-through：

```text
src/dddmr_lego_loam/lego_loam_bor/src/go2_odom_standardizer_node.cpp

raw_to_standard_yaw default = 0.0
rotate_parent_frame default = false
rotate_twist default = false
```

同时保留非零 `raw_to_standard_yaw` 的 warning，防止后续误把 ±90° 当成默认修复。

2. 三个 Go2 XT16 launch 默认值已同步：

```text
src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_live.launch
src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_go2_xt16_mouth.launch
```

当前默认：

```text
raw_to_standard_yaw:=0.0
rotate_odom_parent_frame:=false
rotate_odom_twist:=false
```

3. `go2_imu -> hesai_lidar` 的 `lidar_yaw=+90°` 没有改。本轮只修 odom standardization，不把 LiDAR TF 和 odom 轴向问题混在一起。

4. 新增只读 odom 轴向检查脚本：

```text
scripts/check_go2_odom_axis_consistency.py
```

用途：

```bash
python3 scripts/check_go2_odom_axis_consistency.py \
  --odom-topic /utlidar/robot_odom \
  --duration 3
```

它输出：

```text
move_dir
average yaw
heading_error
forward
lateral
lateral_ratio
PASS / FAIL
```

5. 新增只读 TF 倾斜健康检查脚本：

```text
scripts/check_go2_xt16_tf_health.py
```

默认检查：

```text
base_link -> hesai_lidar
odom -> base_link
map -> odom
map -> base_link
```

默认阈值：

```text
odom->base_link roll/pitch < 5 deg
map->odom roll/pitch       < 3 deg
map->base_link roll/pitch  < 6 deg
```

6. `go2_xt16_navigation.yaml` 已收紧 MCL z/roll/pitch 搜索：

```text
init_var_z: 0.2
init_var_roll: 0.02
init_var_pitch: 0.02
resample_var_z: 0.05
resample_var_roll: 0.02
resample_var_pitch: 0.02
expansion_var_z: 0.05
expansion_var_roll: 0.02
expansion_var_pitch: 0.02
```

yaw 仍保留可用搜索空间：

```text
init_var_yaw: 0.5
resample_var_yaw: 0.1
expansion_var_yaw: 0.2
```

7. 短距离 recovery 参数已改：

```text
oscillation_distance: 0.05
oscillation_angle: 0.3
oscillation_patience: 15.0
```

8. axis/tilt 修复阶段默认速度已从 `0.50` 降到 `0.30`：

```text
differential_drive_simple.max_vel_x: 0.30
differential_drive_simple.min_vel_x: 0.30
differential_drive_simple.acc_lim_x: 3.00

go2_nav_cmd_gate_max_x: 0.30
sport_dry_run_max_x: 0.30
go2_sport_max_x: 0.30
```

9. 更新了旧技术文档中的 odom 说明：

```text
docs/go2_xt16_technical_document_and_route.md
```

旧文档里 `raw_to_standard_yaw=-90°` 的说法已改为 pass-through，并注明 2026-07-03 轴向 probe 结论覆盖旧判断。

10. 新增 workspace hygiene：

```text
report_packages/COLCON_IGNORE
```

原因是 `report_packages/.../code/src` 里有打包副本，colcon 会把它误当成第二份 ROS package，导致 `dddmr_beginner_guide` 和 `lego_loam_bor` 包名重复。

### 9.2 本轮验证结果

已完成的验证：

```text
python3 -m py_compile scripts/check_go2_odom_axis_consistency.py scripts/check_go2_xt16_tf_health.py
结果：PASS

./scripts/dddmr_docker_go2_xt16.sh build-navigation
结果：PASS，17 packages finished

RVIZ=false ./scripts/dddmr_docker_go2_xt16.sh navigation-live-source --show-args
结果：PASS，launch 参数解析正常，默认值显示 raw_to_standard_yaw=0.0、rotate_odom_parent_frame=false、rotate_odom_twist=false、max_x=0.30

RUN_SECONDS=1 RVIZ=false ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run --show-args
结果：wrapper 进入限时启动路径并以 timeout exit 124 退出；启动日志确认参数生效
```

`navigation-dry-run` 1 秒日志中关键证据：

```text
go2_odom_standardizer:
  raw_to_standard_yaw=0.000000 rad
  first odom standardized raw p/yaw == output p/yaw

mcl_3dl:
  init_var_z=0.20
  init_var_roll=0.02
  init_var_pitch=0.02
  resample_var_z=0.05
  resample_var_roll=0.02
  resample_var_pitch=0.02
  expansion_var_z=0.05
  expansion_var_roll=0.02
  expansion_var_pitch=0.02

p2p_move_base / trajectory_generators:
  differential_drive_simple min_vel_x=0.30
  differential_drive_simple max_vel_x=0.30
  acc_lim_x=3.00

go2_nav_cmd_gate:
  max_x=0.300
  max_y=0.000
  max_yaw=0.250

go2_sport_cmd_vel_dry_run:
  DRY RUN ONLY
  no /api/sport/request publisher is created
  max_x=0.300
```

构建前遇到的问题和处理：

```text
问题：report_packages/.../code/src 中的打包副本导致 colcon duplicate package names
处理：在 report_packages/ 下添加 COLCON_IGNORE
结果：重新 build-navigation 通过
```

结束前清理状态：

```text
已停止补丁前遗留的 go2_xt16_localization_rviz_20260703_1624 observation 容器；
docker ps 无同类残留；
ps 未发现 go2_xt16_navigation.launch / go2_odom_standardizer / go2_sport_cmd_vel / rviz2 等残留进程。
```

### 9.3 仍未完成的 live / 人工验收

下面这些仍然没有做，不能写成已通过：

```text
1. 新版 pass-through standard odom 的现场轴向 probe。
2. RViz Fixed Frame=base_link 的 XT16 前方物体方向检查。
3. 设置 initial pose 后的 live map->odom / map->base_link roll/pitch 健康检查。
4. dry-run goal 的路径方向和 /dddmr_go2/dry_run_cmd_vel 合理性检查。
5. supervised live 0.30m/s 短目标。
```

下一次继续时，顺序仍然是：

```text
只读 / no-motion preflight
  -> check_go2_xt16_tf_health.py
  -> RViz base_link 点云方向检查
  -> supervised 轴向 probe 后跑 check_go2_odom_axis_consistency.py
  -> navigation-dry-run goal
  -> 用户明确批准后才做 supervised live short goal
```

---

## 10. 2026-07-03 继续验证记录

本轮继续推进上面的 no-motion 验收项。没有执行 Go2 运动命令，没有发布
`/api/sport/request`，没有修改 Go2 网络、系统服务、启动项或 XT16 driver 工作区。

### 10.1 live XT16 只读 preflight

命令：

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

结果：

```text
RESULT: PASS
```

关键证据：

```text
sample count: 3
frame: hesai_lidar
size: 64000x1
point_step: 26
fields: x FLOAT32, y FLOAT32, z FLOAT32, intensity FLOAT32, ring UINT16, timestamp FLOAT64
ring: min=0 max=15 unique=16
timestamp span: 0.099582s - 0.099815s
header_delta: 0.000000s
header rate avg: 10.03Hz
receive rate avg: 10.05Hz
```

这证明当前 live `/lidar_points` 合同仍然满足 XT16 输入要求。

### 10.2 修复 Docker timeout 路径的 launch 参数转发

验证过程中发现：

```text
RUN_SECONDS=20 ... navigation-live-source start_move_base:=false start_go2_nav_cmd_gate:=false
```

原本应该只启动 localization/perception，但 timeout 路径没有把追加 launch 参数传给内部
`ros2 launch`，导致 `p2p_move_base` 和 `go2_nav_cmd_gate` 仍然启动。

已修改：

```text
scripts/dddmr_docker_go2_xt16.sh
```

修复点：

```text
mapping
navigation-dry-run
navigation-live-source
```

三个 `RUN_SECONDS` timeout 分支现在都会把 `"$@"` 继续传给内部 launch。

验证：

```bash
RUN_SECONDS=5 RVIZ=false \
DDDMR_DOCKER_NAME=go2_xt16_tf_args_probe_... \
./scripts/dddmr_docker_go2_xt16.sh navigation-live-source \
  start_move_base:=false \
  start_go2_nav_cmd_gate:=false
```

结果符合预期，只启动：

```text
go2_odom_standardizer
base_link_to_base_footprint
base_link_to_go2_imu
go2_imu_to_hesai_lidar
mcl_feature
dddmr_pg_map_server_node
mcl_3dl
```

未再启动：

```text
p2p_move_base_node
go2_nav_cmd_gate
go2_sport_cmd_vel_dry_run
go2_sport_cmd_vel_adapter
rviz2
```

### 10.3 干净 observation-only TF health

启动命令：

```bash
RUN_SECONDS=15 RVIZ=false \
DDDMR_DOCKER_NAME=go2_xt16_tf_health_clean_... \
./scripts/dddmr_docker_go2_xt16.sh navigation-live-source \
  start_move_base:=false \
  start_go2_nav_cmd_gate:=false
```

检查命令：

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    python3 scripts/check_go2_xt16_tf_health.py --timeout-sec 8'
```

结果：

```text
TF_HEALTH_STATUS=PASS
```

关键 TF：

```text
base_link -> hesai_lidar:
  xyz = 0.145430, 0.000000, 0.133120
  rpy = 0.000, 0.000, 90.000 deg
  status = PASS

odom -> base_link:
  xyz = 0.726686, -0.255009, 0.300823
  rpy = -2.930, 1.559, -20.665 deg
  max roll/pitch = 2.930 deg, threshold = 5.000 deg
  status = PASS

map -> odom:
  xyz = -0.581053, -0.360710, -0.352267
  rpy = 2.113, -2.242, 20.803 deg
  max roll/pitch = 2.242 deg, threshold = 3.000 deg
  status = PASS

map -> base_link:
  xyz = 0.181360, -0.355543, -0.032854
  rpy = -0.141, 0.169, 0.158 deg
  max roll/pitch = 0.169 deg, threshold = 6.000 deg
  status = PASS
```

本次 no-motion observation 下，之前最担心的 `map->odom` 十几度 roll/pitch 没有复现。
这说明 pass-through odom + 收紧 MCL roll/pitch/z 后，当前静止定位姿态满足 tilt health gate。

注意：这仍不等于完成“RViz 人工设置 initial pose 后”的全部验收。它只证明当前默认
observation/localization 状态下 TF tilt gate 通过。

### 10.4 结束状态

结束后复查：

```text
docker ps: 无 go2_xt16 / dddmr_navigation 残留容器
ps: 未发现 go2_xt16_navigation.launch、go2_odom_standardizer、mcl_3dl、mcl_feature、rviz2、go2_sport_cmd_vel 等残留进程
```

额外静态检查：

```text
bash -n scripts/dddmr_docker_go2_xt16.sh: PASS
python3 -m py_compile scripts/check_go2_odom_axis_consistency.py scripts/check_go2_xt16_tf_health.py: PASS
```

### 10.5 当前剩余项

已经完成：

```text
live /lidar_points preflight: PASS
clean observation-only TF tilt health: PASS
timeout wrapper 参数转发: fixed and verified
```

仍未完成，不能写成已通过：

```text
1. 新版 pass-through standard odom 的现场轴向 probe。
2. RViz Fixed Frame=base_link 的 XT16 前方物体方向人工检查。
3. RViz 人工设置 initial pose 后再次复查 map->odom / map->base_link roll/pitch。
4. dry-run goal 的路径方向和 /dddmr_go2/dry_run_cmd_vel 合理性检查。
5. supervised live 0.30m/s 短目标。
```

下一步建议仍然保持：

```text
先做 RViz base_link 点云方向人工检查；
再做 supervised 轴向 probe 并用 check_go2_odom_axis_consistency.py 分析；
最后才做 dry-run goal 和用户明确批准后的 supervised live short goal。
```

## 11. 2026-07-03 dry-run goal no-motion probe 追加结果

新增脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/probe_go2_xt16_dry_run_goal.py
```

用途：

```text
只发送 PToPMoveBase action goal；
订阅 /dddmr_go2/dry_run_cmd_vel、/dddmr_go2/safe_cmd_vel；
订阅 /global_path、/awared_global_path、/prune_plan；
统计路径方向、非零 dry-run cmd、速度上限和 gate 输出；
支持相对目标和绝对 map-frame 目标。
```

静态验证：

```text
python3 -m py_compile scripts/probe_go2_xt16_dry_run_goal.py: PASS
Docker overlay --help import/run: PASS
```

### 11.1 第一次相对 0.5m dry-run goal

启动方式：

```bash
RUN_SECONDS=35 RVIZ=false \
DDDMR_DOCKER_NAME=go2_xt16_dry_goal_probe_20260703_... \
./scripts/dddmr_docker_go2_xt16.sh navigation-live-source
```

安全边界：

```text
start_sport_dry_run_adapter:=false
start_go2_sport_adapter:=false
rviz:=false
/cmd_vel remap 到 /dddmr_go2/dry_run_cmd_vel
go2_nav_cmd_gate 输出 /dddmr_go2/safe_cmd_vel
未发布 /api/sport/request
```

probe 输出摘要：

```text
DRY_RUN_GOAL_START_X=-0.046134
DRY_RUN_GOAL_START_Y=0.064513
DRY_RUN_GOAL_START_YAW_DEG=-15.696
DRY_RUN_GOAL_TARGET_X=0.435222
DRY_RUN_GOAL_TARGET_Y=-0.070750
DRY_RUN_GOAL_ACCEPTED=true
DRY_RUN_GOAL_CANCEL_ACCEPTED=true
DRY_RUN_DECISIONS=d_initial,d_planning,d_planning_waitdone
DRY_RUN_CMD_COUNT=0
SAFE_CMD_COUNT=583
SAFE_CMD_NONZERO_COUNT=0
DRY_RUN_GOT_PATH=false
DRY_RUN_GOAL_STATUS=FAIL
```

planner 日志解释：

```text
Selected goal: 0.44, -0.07, 0.01, Nearest-> id: 976, x: 0.49, y: -0.09, z: -0.17
Selected start: -0.05, 0.06, 0.01, Nearest-> id: 984, x: -0.08, y: 0.11, z: -0.11
No path found from: 984 to 976
```

结论：这次不是 motion/gate 泄漏问题，而是当前自动定位起点到该目标的图连通性不成立。
gate 只有 zero/no_input/timeout 输出。

### 11.2 第二次绝对 map 目标 dry-run goal

目标点选自前一轮成功日志中邻近 `id=1008` 的区域：

```text
target map=(0.08, 0.49)
```

probe 输出摘要：

```text
DRY_RUN_GOAL_START_X=0.930093
DRY_RUN_GOAL_START_Y=0.679817
DRY_RUN_GOAL_START_YAW_DEG=-6.481
DRY_RUN_GOAL_TARGET_X=0.080000
DRY_RUN_GOAL_TARGET_Y=0.490000
DRY_RUN_GOAL_MODE=absolute
DRY_RUN_GOAL_DISTANCE_M=0.871
DRY_RUN_GOAL_ACCEPTED=true
DRY_RUN_GOAL_CANCEL_ACCEPTED=true
DRY_RUN_DECISIONS=d_initial,d_planning,d_planning_waitdone
DRY_RUN_CMD_COUNT=0
SAFE_CMD_COUNT=585
SAFE_CMD_NONZERO_COUNT=0
DRY_RUN_GOT_PATH=false
DRY_RUN_GOAL_STATUS=FAIL
```

planner 日志解释：

```text
Selected goal: 0.08, 0.49, -0.06, Nearest-> id: 1008, x: 0.08, y: 0.49, z: -0.14
Selected start: 0.93, 0.68, -0.06, Nearest-> id: 840, x: 0.98, y: 0.41, z: -0.32
No path found from: 840 to 1008
```

结论：绝对目标本身能落到预期图点 `id=1008`，但本次自动 localization 起点漂到 `id=840`
附近，仍无法连通到该目标。此项不能写成已通过。

### 11.3 本轮更新后的剩余判断

新增可保留证据：

```text
dry-run action server 可接受并可取消 goal。
失败目标不会产生 /dddmr_go2/dry_run_cmd_vel 非零命令。
go2_nav_cmd_gate 只输出 0 或 timeout 后 0。
planner abort 的直接原因是目标 admissibility / graph connectivity，而不是 /cmd_vel remap 或 sport adapter 泄漏。
```

仍未完成：

```text
dry-run goal 的 path direction + nonzero /dddmr_go2/dry_run_cmd_vel 合理性验收。
```

下一步顺序应调整为：

```text
1. 先在 RViz 中人工设置 initial pose，让 map->base_link 稳定落在同一连通区域。
2. 复查 map->odom / map->base_link roll/pitch tilt。
3. 再运行 probe_go2_xt16_dry_run_goal.py。
4. 只有在 DRY_RUN_GOAL_STATUS=PASS 后，才进入用户明确批准的 supervised live short goal。
```

## 12. 2026-07-03 dry-run goal 通过记录

本节覆盖 11.3 中“dry-run goal 未完成”的状态。新的做法是先用 planner-only
candidate probe 找可连通目标，再用 P2P dry-run probe 验证 path direction 和 dry-run cmd。

新增脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/probe_go2_xt16_plan_candidates.py
```

用途：

```text
只调用 dddmr_sys_core/action/GetPlan；
不发布 /cmd_vel；
不发布 /api/sport/request；
按当前 map->base_link 位姿生成候选目标；
输出可连通 target_x / target_y 供 dry-run P2P probe 使用。
```

静态验证：

```text
python3 -m py_compile scripts/probe_go2_xt16_plan_candidates.py scripts/probe_go2_xt16_dry_run_goal.py: PASS
Docker overlay --help import/run: PASS
```

### 12.1 planner-only candidate probe

启动 no-motion navigation source：

```bash
RUN_SECONDS=70 RVIZ=false \
DDDMR_DOCKER_NAME=go2_xt16_plan_candidate_probe_20260703_... \
./scripts/dddmr_docker_go2_xt16.sh navigation-live-source
```

安全边界保持：

```text
start_sport_dry_run_adapter:=false
start_go2_sport_adapter:=false
rviz:=false
/cmd_vel remap 到 /dddmr_go2/dry_run_cmd_vel
无 /api/sport/request 发布器
```

候选目标 probe 命令：

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    set +u && source .docker_go2_xt16_install/setup.bash && set -u && \
    python3 scripts/probe_go2_xt16_plan_candidates.py \
      --action-name /get_plan \
      --distances 0.5,0.7,0.9,1.1 \
      --angles-deg=-150,-120,-90,-60,-30,0,30,60,90,120,150,180 \
      --tf-timeout-sec 8 \
      --action-timeout-sec 8 \
      --result-timeout-sec 5 \
      --min-path-forward 0.20 \
      --min-path-size 3'
```

关键结果：

```text
PLAN_PROBE_START_X=0.062718
PLAN_PROBE_START_Y=0.000942
PLAN_PROBE_START_Z=-0.041481
PLAN_PROBE_START_YAW_DEG=-1.287
PLAN_BEST_STATUS=PASS
PLAN_BEST_INDEX=25
PLAN_BEST_TARGET_X=-0.404676
PLAN_BEST_TARGET_Y=-0.768176
PLAN_BEST_TARGET_Z=-0.041481
PLAN_BEST_DISTANCE_M=0.900
PLAN_BEST_ANGLE_DEG=-120.0
PLAN_BEST_PATH_SIZE=14
PLAN_BEST_PATH_FORWARD_M=0.893863
PLAN_BEST_PATH_LATERAL_RATIO=0.018941
```

planner-only 结论：当前自动 localization 下，`map=(-0.404676, -0.768176)` 是可连通目标，
距离 0.9m，大于 P2P `xy_goal_tolerance=0.40`，适合做 dry-run cmd 验证。

### 12.2 P2P dry-run goal probe

P2P dry-run probe 命令：

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    set +u && source .docker_go2_xt16_install/setup.bash && set -u && \
    python3 scripts/probe_go2_xt16_dry_run_goal.py \
      --target-x -0.404676 \
      --target-y -0.768176 \
      --duration 8 \
      --tf-timeout-sec 8 \
      --action-timeout-sec 8 \
      --min-path-forward 0.20 \
      --max-path-lateral-ratio 2.0'
```

probe 结果：

```text
DRY_RUN_GOAL_START_X=0.062788
DRY_RUN_GOAL_START_Y=0.000923
DRY_RUN_GOAL_START_Z=-0.041479
DRY_RUN_GOAL_START_YAW_DEG=-1.441
DRY_RUN_GOAL_TARGET_X=-0.404676
DRY_RUN_GOAL_TARGET_Y=-0.768176
DRY_RUN_GOAL_MODE=absolute
DRY_RUN_GOAL_DISTANCE_M=0.900
DRY_RUN_PATH_REFERENCE_HEADING_DEG=-121.292
DRY_RUN_GOAL_ACCEPTED=true
DRY_RUN_GOAL_CANCEL_ACCEPTED=true
DRY_RUN_DECISIONS=d_initial,d_planning,d_planning_waitdone,d_align_heading
DRY_RUN_AWARED_PATH_MAX_SIZE=16
DRY_RUN_PRUNE_PLAN_MAX_SIZE=15
DRY_RUN_BEST_PATH_SOURCE=awared_global_path
DRY_RUN_BEST_PATH_SIZE=16
DRY_RUN_PATH_FORWARD_M=0.893864
DRY_RUN_PATH_LATERAL_M=-0.016861
DRY_RUN_PATH_LATERAL_RATIO=0.018863
DRY_RUN_PATH_DIRECTION_OK=true
DRY_RUN_CMD_COUNT=158
DRY_RUN_CMD_NONZERO_COUNT=158
DRY_RUN_CMD_MAX_ABS_X=0.030000
DRY_RUN_CMD_MAX_ABS_Y=0.000000
DRY_RUN_CMD_MAX_ABS_YAW=0.200000
DRY_RUN_CMD_LIMIT_OK=true
SAFE_CMD_COUNT=589
SAFE_CMD_NONZERO_COUNT=392
SAFE_CMD_MAX_ABS_X=0.030000
SAFE_CMD_MAX_ABS_Y=0.000000
SAFE_CMD_MAX_ABS_YAW=0.200000
SAFE_CMD_LIMIT_OK=true
DRY_RUN_GOT_PATH=true
DRY_RUN_GOT_AWARED_PATH=true
DRY_RUN_GOT_PRUNE_PLAN=true
DRY_RUN_GOAL_STATUS=PASS
```

launch 侧关键日志：

```text
Selected goal: -0.40, -0.77, -0.04, Nearest-> id: 702, x: -0.49, y: -0.89, z: -0.38
Selected start: 0.06, 0.00, -0.04, Nearest-> id: 974, x: 0.07, y: -0.01, z: -0.14
Path found from: 974 to 702
Recieve new global plan.
Decision from -- d_planning_waitdone -- to -- d_align_heading --
prune_plan_ready nearest_index=0 size=7 pcl_size=7 global_plan_size=14 robot=(0.063,0.001,-0.041)
initial_heading_check: yaw_error=-1.931 abs=1.931 threshold=0.800 aligned=0
DWA goal reach the global path end at: -0.40, -0.77, -0.04
safe_cmd_vel reason=pass x=0.030 y=0.000 yaw=-0.200
P2P move base cancelled.
safe_cmd_vel reason=timeout ... x=0.000 y=0.000 yaw=0.000
```

本项结论：

```text
dry-run goal path direction: PASS
/dddmr_go2/dry_run_cmd_vel nonzero and capped: PASS
/dddmr_go2/safe_cmd_vel capped and returns to zero after cancel: PASS
```

### 12.3 结束状态

timeout-limited launch exit code：

```text
124，符合 RUN_SECONDS=70 的预期 timeout 行为。
```

结束后复查：

```text
docker ps: 无残留容器
ps: 未发现 go2_xt16 / dddmr / ros2 launch / rviz2 残留进程
```

### 12.4 当前剩余项

已完成：

```text
live /lidar_points preflight: PASS
clean observation-only TF tilt health: PASS
timeout wrapper 参数转发: fixed and verified
dry-run goal path direction + /dddmr_go2/dry_run_cmd_vel 合理性: PASS
```

仍未完成，不能写成已通过：

```text
1. 新版 pass-through standard odom 的现场轴向 probe。
2. RViz Fixed Frame=base_link 的 XT16 前方物体方向人工检查。
3. RViz 人工设置 initial pose 后再次复查 map->odom / map->base_link roll/pitch。
4. supervised live 0.30m/s 短目标。
```

下一步仍必须保持：

```text
无运动：先做 RViz/base_link 点云方向与 initial pose 后 TF tilt 复查；
需要用户明确批准和现场监督后，才允许 supervised live 0.30m/s 短目标。
```

## 13. 2026-07-03 base_link 点云方向数值检查与当前 TF tilt 复查

本节不是替代 RViz 人工验收，而是给 `Fixed Frame=base_link` 的前方物体检查增加数值证据。
仍需要人工在 RViz 中确认“真实前方物体”出现在 `base_link +X` 方向。

新增脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/summarize_go2_xt16_base_cloud.py
```

用途：

```text
订阅 /lidar_points；
读取 base_link <- hesai_lidar TF；
将点云采样转换到 base_link；
统计 front/back/left/right 点数、前方带状区域点数、最近前方点；
不发布任何 topic，不发送任何运动命令。
```

静态验证：

```text
python3 -m py_compile scripts/summarize_go2_xt16_base_cloud.py scripts/check_go2_xt16_tf_health.py: PASS
Docker overlay summarize_go2_xt16_base_cloud.py --help: PASS
```

### 13.1 observation-only 启动

启动命令：

```bash
RUN_SECONDS=25 RVIZ=false \
DDDMR_DOCKER_NAME=go2_xt16_base_cloud_probe_20260703_... \
./scripts/dddmr_docker_go2_xt16.sh navigation-live-source \
  start_move_base:=false \
  start_go2_nav_cmd_gate:=false
```

启动节点只包含：

```text
go2_odom_standardizer
static_transform_publisher: base_link->base_footprint
static_transform_publisher: base_link->go2_imu
static_transform_publisher: go2_imu->hesai_lidar
mcl_feature
dddmr_pg_map_server_node
mcl_3dl
```

未启动：

```text
global_planner_node
p2p_move_base_node
go2_nav_cmd_gate
go2_sport_cmd_vel_dry_run
go2_sport_cmd_vel_adapter
rviz2
```

### 13.2 base_link 点云方向数值摘要

命令：

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    python3 scripts/summarize_go2_xt16_base_cloud.py \
      --timeout-sec 8 \
      --max-points 20000 \
      --min-range 0.20 \
      --max-range 8.0 \
      --z-min -1.5 \
      --z-max 2.0 \
      --front-band-y-abs 0.75'
```

结果：

```text
BASE_CLOUD_TOPIC=/lidar_points
BASE_CLOUD_SOURCE_FRAME=hesai_lidar
BASE_CLOUD_TARGET_FRAME=base_link
BASE_CLOUD_TOTAL_POINTS=64000
BASE_CLOUD_SAMPLED_POINTS=16000
BASE_CLOUD_KEPT_POINTS=14861
BASE_CLOUD_TRANSFORM_XYZ=0.145430,0.000000,0.133120
BASE_CLOUD_TRANSFORM_RPY_DEG=0.000,0.000,90.000
BASE_CLOUD_MIN_X=-5.951439
BASE_CLOUD_MAX_X=6.421242
BASE_CLOUD_MIN_Y=-7.786440
BASE_CLOUD_MAX_Y=4.903735
BASE_CLOUD_CENTROID_X=-0.093851
BASE_CLOUD_CENTROID_Y=-0.197872
BASE_CLOUD_FRONT_COUNT=7969
BASE_CLOUD_BACK_COUNT=6892
BASE_CLOUD_LEFT_COUNT=7814
BASE_CLOUD_RIGHT_COUNT=7047
BASE_CLOUD_FRONT_BAND_COUNT=2133
BASE_CLOUD_BACK_BAND_COUNT=1500
BASE_CLOUD_NEAREST_FRONT_X=0.293125
BASE_CLOUD_NEAREST_FRONT_Y=0.027027
BASE_CLOUD_NEAREST_FRONT_Z=0.109463
BASE_CLOUD_NEAREST_FRONT_RANGE_XY=0.294369
BASE_CLOUD_STATUS=PASS
BASE_CLOUD_NOTE=manual_rviz_front_object_check_still_required
```

解释：

```text
base_link <- hesai_lidar 静态 TF 为 xyz=(0.14543,0,0.13312), yaw=90deg，与 launch 配置一致。
转换到 base_link 后，当前 live 点云在 +X 前方有有效点，最近前方点约 x=0.293m, y=0.027m。
这能证明 TF/点云转换链可用，但不能替代“把已知物体放在狗正前方并用 RViz 观察”的人工验收。
```

### 13.3 同一 observation-only run 的 TF tilt 复查

命令：

```bash
docker run --rm --network=host \
  --volume /home/lin/new2/dddmr_navigation:/root/dddmr_navigation \
  dddmr_go2_xt16:x64 \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /root/dddmr_navigation/scripts/setup_go2_dds_env.sh && \
    cd /root/dddmr_navigation && \
    python3 scripts/check_go2_xt16_tf_health.py --timeout-sec 8'
```

结果：

```text
TF_HEALTH_BASE_LIDAR_STATUS=PASS
TF_HEALTH_ODOM_BASE_MAX_ROLL_PITCH_DEG=2.920
TF_HEALTH_ODOM_BASE_STATUS=PASS
TF_HEALTH_MAP_ODOM_MAX_ROLL_PITCH_DEG=4.153
TF_HEALTH_MAP_ODOM_THRESHOLD_DEG=3.000
TF_HEALTH_MAP_ODOM_STATUS=FAIL
TF_HEALTH_MAP_BASE_MAX_ROLL_PITCH_DEG=1.982
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=FAIL
```

解释：

```text
本次 observation-only 下 map->base_link 仍在 6deg 阈值内；
但 map->odom roll/pitch 达到 4.153deg，超过 3deg 阈值。
因此“RViz 人工设置 initial pose 后再次复查 map->odom / map->base_link roll/pitch”仍是必做项。
不能再只引用第 10 节的旧 PASS 作为当前 live tilt 结论。
```

### 13.4 结束状态

timeout-limited launch exit code：

```text
124，符合 RUN_SECONDS=25 的预期 timeout 行为。
```

结束后复查：

```text
docker ps: 无残留容器
ps: 未发现 go2_xt16 / dddmr / ros2 launch / rviz2 残留进程
```

### 13.5 当前剩余项更新

已推进但仍需人工验收：

```text
RViz Fixed Frame=base_link 的 XT16 前方物体方向检查：
  已有 base_link 数值点云摘要 PASS；
  仍需人工放置已知前方物体并用 RViz 确认显示在 +X 前方。
```

仍未完成，不能写成已通过：

```text
1. 新版 pass-through standard odom 的现场轴向 probe。
2. RViz Fixed Frame=base_link 的 XT16 前方物体方向人工检查。
3. RViz 人工设置 initial pose 后再次复查 map->odom / map->base_link roll/pitch。
4. supervised live 0.30m/s 短目标。
```

## 14. manual/supervised gate 静态审计

本节只审计本地脚本、launch 和配置文件，不连接运动接口、不启动 Docker、不发布任何 topic。

新增脚本：

```text
scripts/check_go2_xt16_manual_gate_status.sh
```

脚本覆盖：

```text
bash -n:
  scripts/dddmr_docker_go2_xt16.sh
  scripts/run_go2_xt16_navigation_supervised_live.sh
  scripts/check_go2_xt16_sport_live_readiness.sh
  scripts/run_go2_sport_adapter_supervised_probe.sh
  scripts/record_go2_xt16_nav_debug_bag.sh

Python ast.parse:
  scripts/check_go2_odom_axis_consistency.py
  scripts/check_go2_xt16_tf_health.py
  scripts/probe_go2_xt16_dry_run_goal.py
  scripts/probe_go2_xt16_plan_candidates.py
  scripts/summarize_go2_xt16_base_cloud.py
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/check_go2_xt16_manual_gate_status.sh
scripts/check_go2_xt16_manual_gate_status.sh
```

结果：

```text
GO2_XT16_STATIC_SHELL_STATUS=PASS
GO2_XT16_STATIC_PYTHON_STATUS=PASS
GO2_XT16_STATIC_NO_MOTION_DEFAULTS=PASS
GO2_XT16_STATIC_LIVE_GATES=PASS
GO2_XT16_PLANNER_CONFIG_MAX_VEL_X=0.30
GO2_XT16_PLANNER_CONFIG_MIN_VEL_X=0.30
GO2_XT16_LIVE_WRAPPER_DEFAULT_MAX_X=0.10
GO2_XT16_LIVE_PROBE_HARD_CAP_X=0.10
GO2_XT16_MANUAL_GATE_STATUS=PASS
GO2_XT16_MANUAL_GATE_NOTE=static_audit_only_no_robot_or_motion_commands
```

已确认的静态安全门：

```text
1. go2_xt16_navigation.launch 默认不启动 embedded live Sport adapter。
2. go2_xt16_navigation.launch 默认 enable_sport_output=false。
3. go2_xt16_navigation.launch 默认 allow_real_request_topic=false。
4. p2p /cmd_vel remap 到 /dddmr_go2/dry_run_cmd_vel。
5. navigation-live-source 会显式关闭 dry-run adapter 和 embedded Sport adapter，
   只作为速度源，由外层 supervised wrapper 决定是否进入真实 /api/sport/request。
6. run_go2_sport_adapter_supervised_probe.sh --live 要求
   GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2。
7. run_go2_xt16_navigation_supervised_live.sh --live 要求
   GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV。
8. live navigation wrapper 要求先提供 GO2_SPORT_PROBE_SUMMARY。
```

关键结论：

```text
local planner 配置里的速度仍是 0.30m/s；
supervised live navigation wrapper 默认上限已更新为 0.30m/s；
live sport probe 仍是 0.10m/s adapter-handshake，不作为导航验收速度；
2026-07-04 已在现场监督下完成 0.30m/s 短目标调试运行，操作者确认机器狗按导航点方向运动且定位正常。
```

### 14.1 剩余项重新归类

截至 2026-07-04 现场验证后，四项 gate 已重新归类为已完成：

```text
1. RViz Fixed Frame=base_link 的 XT16 前方物体方向人工检查：PASS。

2. RViz 人工设置 initial pose 后 TF tilt 复查：PASS。

3. 新版 pass-through standard odom 的现场轴向 probe：PASS。

4. supervised live 0.30m/s 短目标：PASS。
```

## 15. 2026-07-03 剩余验收清单固化

新增 repo-local handoff checklist：

```text
/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_checklist.md
```

该清单把第 14.1 节的剩余项拆成 4 个 gate：

```text
Gate 1: RViz Fixed Frame=base_link 的前方已知物体人工检查。
Gate 2: RViz 设置 initial pose 后重跑 TF tilt health。
Gate 3: raw odom 与 /dddmr_go2/robot_odom_standard 的受控轴向 probe。
Gate 4: supervised live short goal。
```

清单明确约束：

```text
1. checklist 本身不授权运动。
2. odom axis probe 的脚本只读，但仍需要受控物理位移或明确批准的现场运动来源。
3. 当前 supervised live navigation wrapper 默认上限为 x<=0.30m/s；
   单独的 Sport live probe 仍是 x<=0.10m/s adapter-handshake。
4. 第 13.3 节的旧 map->odom=4.153deg FAIL 已被 2026-07-04
   人工 initial pose 后的 Gate2 TF health PASS 取代。
5. checklist 的 pass evidence 已改成 verifier 实际读取的 KEY=VALUE 字段。
```

Gate 1 / Gate 2 / Gate 3 / Gate 4 的关键字段现在明确为：

```text
SCREENSHOT=<absolute existing non-empty PNG/JPEG screenshot file>
TF_HEALTH_LOG=<absolute existing output/log path>
TF_HEALTH_LOG must contain TF_HEALTH_MAP_ODOM_STATUS=PASS / TF_HEALTH_MAP_BASE_STATUS=PASS / TF_HEALTH_STATUS=PASS
RAW_STANDARD_ODOM_MATERIAL_MATCH=true
ODOM_AXIS_ROTATION_CHECK=no_90deg_rotation
RAW_ODOM_AXIS_LOG=<absolute existing output/log path>
RAW_ODOM_AXIS_LOG must contain ODOM_AXIS_TOPIC=/utlidar/robot_odom and ODOM_AXIS_STATUS=PASS
STANDARD_ODOM_AXIS_LOG=<absolute existing output/log path>
STANDARD_ODOM_AXIS_LOG must contain ODOM_AXIS_TOPIC=/dddmr_go2/robot_odom_standard and ODOM_AXIS_STATUS=PASS
GO2_XT16_SPORT_LIVE_READINESS=PASS
SPORT_LIVE_READINESS_LOG=<absolute existing output/log path>
SPORT_LIVE_READINESS_LOG must contain RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
SPORT_PROBE_RESULT=GO2_SPORT_ADAPTER_live_COMPLETE
SPORT_PROBE_SUMMARY=<absolute existing summary env path>
SPORT_PROBE_SUMMARY must contain MODE=live / RESULT=GO2_SPORT_ADAPTER_live_COMPLETE / REQUEST_TOPIC=/api/sport/request
SPORT_PROBE_SUMMARY referenced echo log must be an absolute path and contain api_id: 1008 and api_id: 1003
SPORT_PROBE_HAS_MOVE_1008=true
SPORT_PROBE_HAS_STOPMOVE_1003=true
LIVE_NAV_SHORT_GOAL_STATUS=PASS
LIVE_NAV_SUMMARY=<absolute existing summary env path>
LIVE_NAV_SUMMARY must contain MODE=live / RESULT=GO2_XT16_MIXED_LIVE_NAV_STOPPED / REQUEST_TOPIC=/api/sport/request / SPORT_MAX_X<=0.30
LIVE_NAV_LOG=<absolute existing log path>
LIVE_NAV_LOG must contain RESULT: GO2_XT16_MIXED_LIVE_NAV_RUNNING and SUMMARY_LOG=
LIVE_NAV_MAX_X_MPS=0.30
NO_RESIDUAL_RUNTIME_STATUS=PASS
NO_RESIDUAL_RUNTIME_LOG=<absolute existing output/log path>
NO_RESIDUAL_RUNTIME_LOG must contain GO2_XT16_RUNTIME_DOCKER_STATUS=PASS / GO2_XT16_RUNTIME_PROCESS_STATUS=PASS / GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
```

同时修正：

```text
docs/go2_xt16_sport_live_runbook.md
```

把 live probe 文档中的 yaw hard cap 与当前脚本保持一致：

```text
abs(yaw) <= 0.35
GO2_SPORT_MAX_YAW default = 0.25
```

静态复查：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/check_go2_xt16_manual_gate_status.sh
scripts/check_go2_xt16_manual_gate_status.sh
```

结果：

```text
GO2_XT16_STATIC_SHELL_STATUS=PASS
GO2_XT16_STATIC_PYTHON_STATUS=PASS
GO2_XT16_EVIDENCE_SCHEMA_STATUS=PASS
GO2_XT16_STATIC_NO_MOTION_DEFAULTS=PASS
GO2_XT16_STATIC_LIVE_GATES=PASS
GO2_XT16_MANUAL_GATE_STATUS=PASS
```

本节仍然不代表验收完成；它只是把剩余人工/监督 gate 的命令、证据字段和通过条件固化到 repo 文档中，并确认 current evidence 与 example 模板都包含 verifier 所需字段。

## 16. 2026-07-03 acceptance evidence verifier

新增 evidence template：

```text
/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env.example
```

新增 verifier：

```text
/home/lin/new2/dddmr_navigation/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh
```

用途：

```text
1. 只解析 KEY=VALUE 文本，不 source evidence 文件。
2. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
3. 在 4 个剩余 gate 没有直接证据前，输出 GO2_XT16_ACCEPTANCE_STATUS=NOT_READY。
4. 只有 RViz 前方物体、initial-pose TF tilt、odom axis、supervised live short goal
   全部有 PASS 证据时，才允许输出 GO2_XT16_ACCEPTANCE_STATUS=PASS。
```

静态语法检查：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh
bash -n scripts/check_go2_xt16_manual_gate_status.sh
```

结果：

```text
PASS
```

用 template 运行 verifier：

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env.example \
  --allow-incomplete
```

结果符合预期，当前仍未完成：

```text
GO2_XT16_ACCEPTANCE_GATE1_RVIZ_FRONT_OBJECT_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE1_RVIZ_FRONT_OBJECT_REASON=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS_not_PASS
GO2_XT16_ACCEPTANCE_GATE2_INITIAL_POSE_TF_TILT_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE2_INITIAL_POSE_TF_TILT_REASON=TF_HEALTH_MAP_ODOM_STATUS_not_PASS
GO2_XT16_ACCEPTANCE_GATE3_ODOM_AXIS_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE3_ODOM_AXIS_REASON=RAW_ODOM_AXIS_STATUS_not_PASS
GO2_XT16_ACCEPTANCE_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REASON=GO2_XT16_SPORT_LIVE_READINESS_not_PASS
GO2_XT16_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_REASON=manual_or_supervised_gate_evidence_incomplete
```

当时的静态审计脚本也调用该 verifier 的 template 模式：

```bash
scripts/check_go2_xt16_manual_gate_status.sh
```

关键结果：

```text
GO2_XT16_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_VERIFIER_STATUS=PASS
GO2_XT16_MANUAL_GATE_STATUS=PASS
```

解释：

```text
ACCEPTANCE_STATUS=NOT_READY 表示人工/监督 gate 没有完成；
ACCEPTANCE_VERIFIER_STATUS=PASS 只表示 verifier 本身可运行，并且能正确拒绝空 template。
```

## 17. 2026-07-03 verifier self-test

为避免 verifier 只会拒绝空模板、不能证明完整 evidence 可被接受，本轮给：

```text
/home/lin/new2/dddmr_navigation/scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh
```

增加了：

```bash
--self-test
```

该 self-test 只创建临时本地 KEY=VALUE 文件，不 source ROS，不启动 Docker，不连接 live topic，不发布任何 topic。

覆盖七类合成 evidence：

```text
1. synthetic complete evidence:
   4 个 gate 都给出 PASS，LIVE_NAV_MAX_X_MPS=0.30。
   预期 verifier 输出 GO2_XT16_ACCEPTANCE_STATUS=PASS。

2. synthetic high-speed evidence:
   其他字段保持 PASS，但 LIVE_NAV_MAX_X_MPS=0.50。
   预期 verifier 拒绝，并报告 LIVE_NAV_MAX_X_MPS_missing_or_above_0.30。

3. synthetic missing-artifact evidence:
   其他字段保持 PASS，但 SCREENSHOT 指向不存在的文件。
   预期 verifier 拒绝，并报告 SCREENSHOT_missing_or_empty_file。

4. synthetic relative-artifact evidence:
   其他字段保持 PASS，但 SCREENSHOT 使用相对路径。
   预期 verifier 拒绝，并报告 SCREENSHOT_not_absolute。

5. synthetic bad-screenshot evidence:
   SCREENSHOT 指向存在且非空的普通文本文件。
   预期 verifier 拒绝，并报告 SCREENSHOT_not_png_or_jpeg。

6. synthetic bad-report evidence:
   REPORT_PATH 指向非当前目标报告。
   预期 verifier 拒绝，并报告 REPORT_PATH_mismatch。

7. synthetic bad-content evidence:
   artifact 文件存在，但 TF_HEALTH_LOG 缺少 TF_HEALTH_MAP_ODOM_STATUS=PASS。
   预期 verifier 拒绝，并报告 TF_HEALTH_LOG_missing_MAP_ODOM_PASS。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh
bash -n scripts/check_go2_xt16_manual_gate_status.sh
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh --self-test
scripts/check_go2_xt16_manual_gate_status.sh
```

结果：

```text
GO2_XT16_ACCEPTANCE_SELFTEST_SYNTHETIC_PASS=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_STATUS_REPORT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_SPEED_CAP_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_ABSOLUTE_PATH_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_SCREENSHOT_TYPE_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_REPORT_LINK_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_ARTIFACT_CONTENT_REJECT=PASS
GO2_XT16_ACCEPTANCE_SELFTEST_STATUS=PASS
GO2_XT16_ACCEPTANCE_VERIFIER_SELFTEST_STATUS=PASS
GO2_XT16_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_VERIFIER_STATUS=PASS
GO2_XT16_MANUAL_GATE_STATUS=PASS
```

解释：

```text
verifier 已证明：
1. 完整且限速合规的 synthetic evidence 会被接受；
2. 0.50m/s synthetic live evidence 会被拒绝；
3. 缺失 artifact 文件的 synthetic evidence 会被拒绝；
4. 相对 artifact 路径的 synthetic evidence 会被拒绝；
5. 非 PNG/JPEG 的 screenshot synthetic evidence 会被拒绝；
6. REPORT_PATH 不指向当前报告的 synthetic evidence 会被拒绝；
7. artifact 文件内容缺少关键 PASS 标记的 synthetic evidence 会被拒绝；
8. template 可以正确显示 NOT_READY。

因此当前报告仍未完成；完成条件仍是把真实人工/监督验收结果写入 evidence 文件并跑出
GO2_XT16_ACCEPTANCE_STATUS=PASS。
```

## 18. 2026-07-03 current evidence file

新增当前状态 evidence 文件：

```text
/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
```

该文件不是模板，而是当前报告的验收状态快照。当前写入：

```text
Gate 1 RViz base_link 前方已知物体检查: NOT_READY
Gate 2 initial-pose TF tilt 复查: FAIL / NOT_READY
Gate 3 raw + standard odom axis probe: NOT_RUN
Gate 4 supervised live short goal: NOT_RUN
NO_RESIDUAL_RUNTIME_STATUS=PASS
NO_RESIDUAL_RUNTIME_LOG=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/no_motion_runtime_clean_20260704_current.txt
remaining manual/supervised artifact path fields: TODO until each gate is actually run
```

其中 Gate 2 的当前依据仍是第 13.3 节：

```text
TF_HEALTH_MAP_ODOM_STATUS=FAIL
TF_HEALTH_MAP_BASE_STATUS=PASS
TF_HEALTH_STATUS=FAIL
map->odom roll/pitch=4.153deg > 3.000deg
```

更新后的静态审计脚本现在验证 current evidence 文件，而不是只验证 `.example` 模板：

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env \
  --allow-incomplete
scripts/check_go2_xt16_manual_gate_status.sh
```

结果：

```text
GO2_XT16_ACCEPTANCE_EVIDENCE_FILE=docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_ACCEPTANCE_REPORT_LINK_STATUS=PASS
GO2_XT16_ACCEPTANCE_GATE1_RVIZ_FRONT_OBJECT_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE1_RVIZ_FRONT_OBJECT_REASON=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS_not_PASS
GO2_XT16_ACCEPTANCE_GATE2_INITIAL_POSE_TF_TILT_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE2_INITIAL_POSE_TF_TILT_REASON=TF_HEALTH_MAP_ODOM_STATUS_not_PASS
GO2_XT16_ACCEPTANCE_GATE3_ODOM_AXIS_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE3_ODOM_AXIS_REASON=RAW_ODOM_AXIS_STATUS_not_PASS
GO2_XT16_ACCEPTANCE_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REASON=GO2_XT16_SPORT_LIVE_READINESS_not_PASS
GO2_XT16_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_ACCEPTANCE_REASON=manual_or_supervised_gate_evidence_incomplete
GO2_XT16_ACCEPTANCE_VERIFIER_STATUS=PASS
GO2_XT16_MANUAL_GATE_STATUS=PASS
```

补充静态检查：

```text
bash -n scripts/check_go2_xt16_manual_gate_status.sh: PASS
bash -n scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh: PASS
git check-ignore: evidence 文件未被 .gitignore 忽略
```

解释：

```text
current evidence 文件让“现在为什么还不能完成”成为可机读状态；
后续只有把 4 个 gate 的真实验收结果写入该文件，并让 verifier 输出
GO2_XT16_ACCEPTANCE_STATUS=PASS，才能把本报告标记为完成。
```

## 19. 2026-07-04 compact status-report mode

新增 verifier compact status mode：

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env \
  --status-report \
  --allow-incomplete
```

用途：

```text
给 CI / reviewer 一个简短的当前状态摘要；
不改变原 verifier 的完成判定；
不 source evidence，不 source ROS，不启动 Docker，不发布任何 topic。
```

运行结果：

```text
GO2_XT16_STATUS_REPORT_EVIDENCE_FILE=docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_STATUS_REPORT_REPORT_LINK=PASS
GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT=NOT_READY
GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT_REASON=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS_not_PASS
GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT=NOT_READY
GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT_REASON=TF_HEALTH_MAP_ODOM_STATUS_not_PASS
GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS=NOT_READY
GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS_REASON=RAW_ODOM_AXIS_STATUS_not_PASS
GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL=NOT_READY
GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REASON=GO2_XT16_SPORT_LIVE_READINESS_not_PASS
GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY
GO2_XT16_STATUS_REPORT_REASON=manual_or_supervised_gate_evidence_incomplete
```

同步验证：

```text
bash -n scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh: PASS
bash -n scripts/check_go2_xt16_manual_gate_status.sh: PASS
scripts/check_go2_xt16_axis_tilt_acceptance_evidence.sh --self-test: PASS
normal verifier output: GO2_XT16_ACCEPTANCE_STATUS=NOT_READY
status-report output: GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY
```

解释：

```text
status-report 只是更短的状态视图；
当前仍然没有完成 4 个真实人工/监督 gate。
```

## 20. 2026-07-04 report completion checker

新增报告完成度检查脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/check_go2_xt16_report_completion.sh
```

用途：

```text
1. 检查本报告文件存在。
2. 检查 current evidence 文件存在。
3. 检查 current evidence 的 REPORT_PATH 指向本报告。
4. 检查 status-report 已记录 REPORT_PATH link PASS。
5. 检查 current evidence 已记录 NO_RESIDUAL_RUNTIME_STATUS=PASS。
6. 检查 NO_RESIDUAL_RUNTIME_LOG 存在并包含 Docker/process/clean 三条 PASS。
7. 检查报告已记录 current evidence / status-report / PASS 完成规则。
8. 检查报告已记录 evidence schema 静态审计 PASS。
9. 检查报告已记录缺失 artifact 会被 verifier 拒绝。
10. 检查报告已记录相对 artifact path 会被 verifier 拒绝。
11. 调用 acceptance verifier 的 status-report 输出作为完成判定来源。
12. 根据 verifier 输出校验顶部 acceptance / completion 状态，而不是硬编码 NOT_READY。
13. 自测覆盖 synthetic PASS、synthetic NOT_READY 和未完成时非零退出。
14. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_report_completion.sh --self-test
scripts/check_go2_xt16_report_completion.sh --allow-incomplete
scripts/check_go2_xt16_manual_gate_status.sh
```

当前结果：

```text
GO2_XT16_REPORT_PATH=/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md
GO2_XT16_REPORT_EVIDENCE_FILE=/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_REPORT_FILE_STATUS=PASS
GO2_XT16_REPORT_TOP_AUTHORITY_BLOCK=PASS
GO2_XT16_REPORT_TOP_REPORT_SHAPE_STATUS=PASS
GO2_XT16_REPORT_TOP_RUNTIME_CLEAN_STATUS=PASS
GO2_XT16_REPORT_TOP_ACCEPTANCE_STATUS=PASS
GO2_XT16_REPORT_TOP_COMPLETION_STATUS=PASS
GO2_XT16_REPORT_SECTION_18_CURRENT_EVIDENCE=PASS
GO2_XT16_REPORT_SECTION_19_STATUS_REPORT=PASS
GO2_XT16_REPORT_SECTION_21_RUNTIME_CLEAN=PASS
GO2_XT16_REPORT_SECTION_22_EVIDENCE_UPDATER=PASS
GO2_XT16_REPORT_SECTION_23_GATE_UPDATE_BUILDER=PASS
GO2_XT16_REPORT_SECTION_24_ACCEPTANCE_PIPELINE=PASS
GO2_XT16_REPORT_SECTION_25_GATE_GAPS=PASS
GO2_XT16_REPORT_STATUS_REPORT_LINK_RECORDED=PASS
GO2_XT16_REPORT_CURRENT_STATUS_RECORDED=PASS
GO2_XT16_REPORT_CURRENT_GATE_GAPS_RECORDED=PASS
GO2_XT16_REPORT_PASS_COMPLETION_RULE_RECORDED=PASS
GO2_XT16_REPORT_EVIDENCE_SCHEMA_RECORDED=PASS
GO2_XT16_REPORT_REPORT_LINK_REJECT_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_ARTIFACT_REJECT_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_ABSOLUTE_PATH_REJECT_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_SCREENSHOT_TYPE_REJECT_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_ARTIFACT_CONTENT_REJECT_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_RECORDED=PASS
GO2_XT16_REPORT_EVIDENCE_UPDATER_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_GATE_UPDATE_BUILDER_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_GATE_GAPS_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_ACCEPTANCE_PIPELINE_SELFTEST_RECORDED=PASS
GO2_XT16_REPORT_EVIDENCE_REPORT_PATH_STATUS=PASS
GO2_XT16_REPORT_EVIDENCE_RUNTIME_STATUS=PASS
GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=PASS
GO2_XT16_STATUS_REPORT_REPORT_LINK=PASS
GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT=NOT_READY
GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT_REASON=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS_not_PASS
GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT=NOT_READY
GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT_REASON=TF_HEALTH_MAP_ODOM_STATUS_not_PASS
GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS=NOT_READY
GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS_REASON=RAW_ODOM_AXIS_STATUS_not_PASS
GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL=NOT_READY
GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REASON=GO2_XT16_SPORT_LIVE_READINESS_not_PASS
GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY
GO2_XT16_STATUS_REPORT_REASON=manual_or_supervised_gate_evidence_incomplete
GO2_XT16_GATE_GAPS_EVIDENCE_FILE=/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_MISSING=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS,RVIZ_FIXED_FRAME,RVIZ_FRONT_OBJECT_DIRECTION,SCREENSHOT:missing
GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_MISSING=TF_HEALTH_MAP_ODOM_STATUS,TF_HEALTH_STATUS,TF_HEALTH_LOG:missing
GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_MISSING=RAW_ODOM_AXIS_STATUS,STANDARD_ODOM_AXIS_STATUS,RAW_STANDARD_ODOM_MATERIAL_MATCH,ODOM_AXIS_ROTATION_CHECK,RAW_ODOM_AXIS_LOG:missing,STANDARD_ODOM_AXIS_LOG:missing
GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_MISSING=GO2_XT16_SPORT_LIVE_READINESS,SPORT_LIVE_READINESS_LOG:missing,SPORT_PROBE_RESULT,SPORT_PROBE_SUMMARY:missing,SPORT_PROBE_HAS_MOVE_1008,SPORT_PROBE_HAS_STOPMOVE_1003,LIVE_NAV_SHORT_GOAL_STATUS,LIVE_NAV_SUMMARY:missing,LIVE_NAV_LOG:missing,LIVE_NAV_MAX_X_MPS
GO2_XT16_GATE_GAPS_REMAINING_GATES=gate1,gate2,gate3,gate4
GO2_XT16_GATE_GAPS_OVERALL=NOT_READY
GO2_XT16_GATE_GAPS_STATUS=PASS
GO2_XT16_REPORT_SHAPE_STATUS=PASS
GO2_XT16_REPORT_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY
GO2_XT16_REPORT_COMPLETION_REASON=acceptance_evidence_not_ready
GO2_XT16_REPORT_COMPLETION_CHECK_STATUS=PASS
```

completion checker 自测结果：

```text
GO2_XT16_REPORT_COMPLETION_SELFTEST_SYNTHETIC_PASS=PASS
GO2_XT16_REPORT_COMPLETION_SELFTEST_SYNTHETIC_NOT_READY=PASS
GO2_XT16_REPORT_COMPLETION_SELFTEST_INCOMPLETE_EXIT=PASS
GO2_XT16_REPORT_COMPLETION_SELFTEST_STATUS=PASS
```

解释：

```text
REPORT_SHAPE_STATUS=PASS 表示报告结构、current evidence 和完成规则都已经记录；
REPORT_COMPLETION_STATUS=NOT_READY 表示真实验收证据仍未完成。
```

因此本报告当前状态仍是：

```text
documented and auditable, but not complete.
```

## 21. 2026-07-04 no-motion runtime clean checker

新增残留运行态检查脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/check_go2_xt16_no_motion_runtime_clean.sh
```

用途：

```text
1. 只运行 docker ps 和 ps。
2. 检查 go2_xt16 / dddmr_navigation 相关容器。
3. 检查 RViz、navigation launch、p2p_move_base、planner、MCL、Sport adapter、
   ros2 topic pub/echo 等残留进程。
4. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
scripts/check_go2_xt16_no_motion_runtime_clean.sh
scripts/check_go2_xt16_manual_gate_status.sh
```

结果：

```text
GO2_XT16_RUNTIME_DOCKER_STATUS=PASS
GO2_XT16_RUNTIME_PROCESS_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
GO2_XT16_REPORT_EVIDENCE_RUNTIME_LOG_STATUS=PASS
GO2_XT16_RUNTIME_CLEAN_CHECK_STATUS=PASS
```

本轮补充的可引用 runtime-clean artifact：

```text
/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/no_motion_runtime_clean_20260704_current.txt
```

同时 full static audit 仍保持：

```text
GO2_XT16_STATIC_SHELL_STATUS=PASS
GO2_XT16_STATIC_PYTHON_STATUS=PASS
GO2_XT16_EVIDENCE_SCHEMA_STATUS=PASS
GO2_XT16_STATIC_NO_MOTION_DEFAULTS=PASS
GO2_XT16_STATIC_LIVE_GATES=PASS
GO2_XT16_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY
GO2_XT16_REPORT_COMPLETION_SELFTEST_CHECK_STATUS=PASS
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_CHECK_STATUS=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_CHECK_STATUS=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_CHECK_STATUS=PASS
GO2_XT16_MANUAL_GATE_STATUS=PASS
```

解释：

```text
当前本机没有残留 Go2 XT16 / DDDMR navigation / RViz / Sport adapter 运行态；
但这只证明运行态干净，不代表 4 个真实人工/监督 gate 已完成。
```

## 22. 2026-07-04 evidence update helper

新增 evidence 更新脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh
```

用途：

```text
1. 现场完成 Gate 1-4 后，只把新增 KEY=VALUE 写到一个小 update 文件。
2. updater 拒绝未知 key、重复 key 和 malformed 行。
3. updater 不 source update 文件，不 source ROS，不启动 Docker，不检查 live topic，
   不发布任何 topic。
4. updater 先把更新应用到临时 evidence，再调用 acceptance verifier；
   verifier 能解析后才写回 current evidence，并保留 .bak 备份。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh
scripts/update_go2_xt16_axis_tilt_acceptance_evidence.sh --self-test
scripts/check_go2_xt16_manual_gate_status.sh
```

自测结果：

```text
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_DRY_RUN=PASS
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_APPLY=PASS
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_UNKNOWN_KEY_REJECT=PASS
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_STATUS=PASS
GO2_XT16_EVIDENCE_UPDATER_SELFTEST_CHECK_STATUS=PASS
```

解释：

```text
该 helper 不让报告变成 PASS；
它只降低人工把 gate evidence 写入 docs/go2_xt16_axis_tilt_acceptance_evidence.env 时的字段错误风险。
最终仍必须由 acceptance verifier 输出 GO2_XT16_ACCEPTANCE_STATUS=PASS。
```

## 23. 2026-07-04 artifact-to-update builder

新增 artifact-to-update 生成脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/build_go2_xt16_axis_tilt_gate_update.sh
```

用途：

```text
1. 从已经捕获的 gate artifact 生成 KEY=VALUE update 文件，所有 artifact 路径必须是绝对路径。
2. Gate 1 校验 screenshot 是 PNG/JPEG，且 fixed frame/direction 为 base_link/+X。
3. Gate 2 校验 TF_HEALTH_MAP_ODOM_STATUS=PASS、TF_HEALTH_MAP_BASE_STATUS=PASS、
   TF_HEALTH_STATUS=PASS。
4. Gate 3 校验 raw/standard odom log 的 topic 和 ODOM_AXIS_STATUS=PASS。
5. Gate 4 校验 readiness、sport probe summary、request echo、live nav summary/log、
   runtime-clean log，以及 0.30m/s navigation wrapper cap。
6. 只读本地 artifact，不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/build_go2_xt16_axis_tilt_gate_update.sh
scripts/build_go2_xt16_axis_tilt_gate_update.sh --self-test
scripts/check_go2_xt16_manual_gate_status.sh
```

自测结果：

```text
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE1=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE2=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE3=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_GATE4=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_BAD_ARTIFACT_REJECT=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_RELATIVE_ARTIFACT_REJECT=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_STATUS=PASS
GO2_XT16_GATE_UPDATE_BUILDER_SELFTEST_CHECK_STATUS=PASS
```

解释：

```text
该 builder 不写 current evidence，也不让报告变成 PASS；
它只把真实 gate artifact 转成 updater 可消费的 KEY=VALUE 文件。
最终仍必须把四个 gate 的真实 artifact 写入 evidence，并由 acceptance verifier 输出
GO2_XT16_ACCEPTANCE_STATUS=PASS。
```

## 24. 2026-07-04 synthetic acceptance pipeline self-test

新增端到端 synthetic pipeline 自测脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/check_go2_xt16_acceptance_pipeline_selftest.sh
```

用途：

```text
1. 创建临时 synthetic gate artifacts 和临时 evidence 文件。
2. 调用 artifact-to-update builder 生成四个 gate 的 KEY=VALUE update 文件。
3. 调用 evidence updater 把四个 update 写入临时 evidence。
4. 调用 acceptance verifier，要求 synthetic evidence 输出
   GO2_XT16_ACCEPTANCE_STATUS=PASS。
5. 调用 status-report，要求 GO2_XT16_STATUS_REPORT_OVERALL=PASS。
6. 不修改 docs/go2_xt16_axis_tilt_acceptance_evidence.env。
7. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/check_go2_xt16_acceptance_pipeline_selftest.sh
scripts/check_go2_xt16_acceptance_pipeline_selftest.sh
scripts/check_go2_xt16_manual_gate_status.sh
```

自测结果：

```text
GO2_XT16_ACCEPTANCE_PIPELINE_BUILDER_STATUS=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_UPDATER_STATUS=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_VERIFIER_STATUS=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_STATUS_REPORT=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_STATUS=PASS
GO2_XT16_ACCEPTANCE_PIPELINE_SELFTEST_CHECK_STATUS=PASS
```

解释：

```text
synthetic pipeline PASS 只证明本地证据流水线能接受完整合成证据；
它不替代真实 RViz 前方物体、initial-pose TF、odom axis、supervised live short goal
四个 gate 的现场 artifact。
当前报告仍必须以 docs/go2_xt16_axis_tilt_acceptance_evidence.env 的真实 verifier 输出为准。
```

## 25. 2026-07-04 gate evidence gap summary

新增 gate evidence gap 摘要脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh
```

用途：

```text
1. 读取 current evidence，把四个 gate 的缺口展开成字段级列表。
2. 检查 artifact path 是否是绝对路径、是否存在且非空。
3. 对 screenshot 检查 PNG/JPEG magic。
4. 对 TF / odom / sport / live nav / runtime-clean artifact 检查必要 PASS 标记。
5. 不替代 acceptance verifier；最终完成判定仍只看 GO2_XT16_ACCEPTANCE_STATUS=PASS。
6. 不 source evidence，不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh
scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh --self-test
scripts/summarize_go2_xt16_axis_tilt_gate_gaps.sh \
  --evidence docs/go2_xt16_axis_tilt_acceptance_evidence.env
scripts/check_go2_xt16_manual_gate_status.sh
```

自测结果：

```text
GO2_XT16_GATE_GAPS_SELFTEST_SYNTHETIC_PASS=PASS
GO2_XT16_GATE_GAPS_SELFTEST_SYNTHETIC_NOT_READY=PASS
GO2_XT16_GATE_GAPS_SELFTEST_RELATIVE_ARTIFACT=PASS
GO2_XT16_GATE_GAPS_SELFTEST_STATUS=PASS
```

当前 current evidence 缺口摘要：

```text
GO2_XT16_GATE_GAPS_EVIDENCE_FILE=docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_MISSING=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS,RVIZ_FIXED_FRAME,RVIZ_FRONT_OBJECT_DIRECTION,SCREENSHOT:missing
GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_MISSING=TF_HEALTH_MAP_ODOM_STATUS,TF_HEALTH_STATUS,TF_HEALTH_LOG:missing
GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_MISSING=RAW_ODOM_AXIS_STATUS,STANDARD_ODOM_AXIS_STATUS,RAW_STANDARD_ODOM_MATERIAL_MATCH,ODOM_AXIS_ROTATION_CHECK,RAW_ODOM_AXIS_LOG:missing,STANDARD_ODOM_AXIS_LOG:missing
GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=NOT_READY
GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_MISSING=GO2_XT16_SPORT_LIVE_READINESS,SPORT_LIVE_READINESS_LOG:missing,SPORT_PROBE_RESULT,SPORT_PROBE_SUMMARY:missing,SPORT_PROBE_HAS_MOVE_1008,SPORT_PROBE_HAS_STOPMOVE_1003,LIVE_NAV_SHORT_GOAL_STATUS,LIVE_NAV_SUMMARY:missing,LIVE_NAV_LOG:missing,LIVE_NAV_MAX_X_MPS
GO2_XT16_GATE_GAPS_REMAINING_GATES=gate1,gate2,gate3,gate4
GO2_XT16_GATE_GAPS_OVERALL=NOT_READY
GO2_XT16_GATE_GAPS_STATUS=PASS
GO2_XT16_GATE_GAPS_CHECK_STATUS=PASS
```

解释：

```text
gap summary 让“下一步现场要补哪些字段和 artifact”变成可机读输出；
但它不是完成条件。
当前报告仍未完成，原因仍是四个真实人工/监督 gate 没有当前 PASS artifact。
```

## 26. 2026-07-04 no-motion status snapshot artifact

新增 no-motion status snapshot collector：

```text
/home/lin/new2/dddmr_navigation/scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh
```

用途：

```text
1. 汇总 acceptance status-report、gate gap summary、report completion audit、
   residual runtime clean check。
2. 可直接输出到 stdout，也可写入一个绝对路径 artifact。
3. 写入 artifact 时先写临时文件，再原子替换目标文件。
4. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
5. 不改变 current evidence，也不改变 acceptance verifier 的 PASS 规则。
```

当前生成的 snapshot artifact：

```text
/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/status_snapshot_20260704_current.txt
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh
scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh --self-test
scripts/collect_go2_xt16_axis_tilt_status_snapshot.sh \
  --output /home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/status_snapshot_20260704_current.txt
scripts/check_go2_xt16_report_completion.sh --allow-incomplete
scripts/check_go2_xt16_manual_gate_status.sh
```

collector 自测结果：

```text
GO2_XT16_STATUS_SNAPSHOT_SELFTEST_WRITE=PASS
GO2_XT16_STATUS_SNAPSHOT_SELFTEST_CONTENT=PASS
GO2_XT16_STATUS_SNAPSHOT_SELFTEST_STATUS=PASS
```

snapshot 写入结果：

```text
GO2_XT16_STATUS_SNAPSHOT_OUTPUT=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/status_snapshot_20260704_current.txt
GO2_XT16_STATUS_SNAPSHOT_WRITE_STATUS=PASS
```

snapshot artifact 关键 marker：

```text
GO2_XT16_STATUS_SNAPSHOT_CREATED_AT=2026-07-04T07:13:43+08:00
GO2_XT16_STATUS_SNAPSHOT_WORKDIR=/home/lin/new2/dddmr_navigation
GO2_XT16_STATUS_SNAPSHOT_REPORT=/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md
GO2_XT16_STATUS_SNAPSHOT_EVIDENCE=/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_STATUS_SNAPSHOT_MODE=no_motion_local_static
GO2_XT16_STATUS_SNAPSHOT_FORBIDDEN_INTERFACES=/cmd_vel,/dddmr_go2/dry_run_cmd_vel,/api/sport/request,/lowcmd
GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY
GO2_XT16_GATE_GAPS_REMAINING_GATES=gate1,gate2,gate3,gate4
GO2_XT16_GATE_GAPS_OVERALL=NOT_READY
GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY
GO2_XT16_RUNTIME_CLEAN_STATUS=PASS
GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS
```

report completion checker 现在还会验证：

```text
1. snapshot artifact 文件存在且非空。
2. artifact 内有 GO2_XT16_STATUS_SNAPSHOT_STATUS=PASS。
3. artifact 内的 GO2_XT16_STATUS_REPORT_OVERALL 与 current evidence 一致。
4. artifact 内的 GO2_XT16_GATE_GAPS_OVERALL 与 current evidence 一致。
5. artifact 内的 GO2_XT16_RUNTIME_CLEAN_STATUS=PASS。
```

当前结论仍然是：

```text
GO2_XT16_REPORT_SHAPE_STATUS=PASS
GO2_XT16_REPORT_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY
```

解释：

```text
status snapshot 只是把当前静态状态固化成可引用 artifact；
它不替代 Gate 1-4 的真实现场 artifact。
报告最终完成条件仍然是 acceptance verifier 对 current evidence 输出
GO2_XT16_ACCEPTANCE_STATUS=PASS。
```

## 27. 2026-07-04 manual gate handoff artifact

新增 manual gate handoff 生成脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/build_go2_xt16_axis_tilt_manual_handoff.sh
```

用途：

```text
1. 读取 current evidence，并嵌入 status-report 与 gate gap summary。
2. 汇总 Gate 1-4 需要现场捕获的 artifact 路径类型和内容要求。
3. 给出每个 gate 完成后的 artifact-to-update builder 命令。
4. 给出 updater 的 --dry-run 命令，要求 dry-run 通过后再真正写 evidence。
5. 明确 handoff 不授权运动；Gate 4 仍必须先有明确现场审批和监督。
6. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

当前生成的 handoff artifact：

```text
/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/manual_gate_handoff_20260704_current.md
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/build_go2_xt16_axis_tilt_manual_handoff.sh
scripts/build_go2_xt16_axis_tilt_manual_handoff.sh --self-test
scripts/build_go2_xt16_axis_tilt_manual_handoff.sh \
  --output /home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/manual_gate_handoff_20260704_current.md
scripts/check_go2_xt16_report_completion.sh --allow-incomplete
scripts/check_go2_xt16_manual_gate_status.sh
```

handoff 自测结果：

```text
GO2_XT16_MANUAL_HANDOFF_SELFTEST_WRITE=PASS
GO2_XT16_MANUAL_HANDOFF_SELFTEST_CONTENT=PASS
GO2_XT16_MANUAL_HANDOFF_SELFTEST_STATUS=PASS
```

handoff 写入结果：

```text
GO2_XT16_MANUAL_HANDOFF_OUTPUT=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/manual_gate_handoff_20260704_current.md
GO2_XT16_MANUAL_HANDOFF_WRITE_STATUS=PASS
```

handoff artifact 关键内容：

```text
GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY
GO2_XT16_GATE_GAPS_OVERALL=NOT_READY
Gate 1: RViz Base_Link Front Object
Gate 2: Initial Pose Then TF Tilt Recheck
Gate 3: Raw And Standard Odom Axis
Gate 4: Supervised Live Short Goal
GO2_XT16_ACCEPTANCE_STATUS=PASS
GO2_XT16_REPORT_COMPLETION_STATUS=PASS
```

report completion checker 现在还会验证：

```text
1. handoff artifact 文件存在且非空。
2. handoff 内的 GO2_XT16_STATUS_REPORT_OVERALL 与 current evidence 一致。
3. handoff 内的 GO2_XT16_GATE_GAPS_OVERALL 与 current evidence 一致。
4. handoff 记录最终验收条件 GO2_XT16_ACCEPTANCE_STATUS=PASS。
5. handoff 记录高风险 motion interface 标记 /api/sport/request。
```

当前结论仍然是：

```text
GO2_XT16_REPORT_SHAPE_STATUS=PASS
GO2_XT16_REPORT_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY
```

解释：

```text
handoff artifact 让现场人员能按 gate 填证据；
它不是 gate 证据本身，也不替代任何人工/监督验收。
报告最终完成条件仍然是 current evidence 通过 acceptance verifier。
```

## 28. 2026-07-04 static artifact refresh runner

新增静态 artifact 刷新脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh
```

用途：

```text
1. 刷新 no-motion runtime-clean log。
2. 刷新 status snapshot artifact。
3. 刷新 manual gate handoff artifact。
4. 刷新后运行 report completion audit 和 manual gate static audit。
5. 不改变 current evidence，不让报告变成 PASS。
6. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh
scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh --self-test
scripts/refresh_go2_xt16_axis_tilt_static_artifacts.sh
```

self-test 结果：

```text
GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_RUNTIME=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_SNAPSHOT=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_HANDOFF=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_SELFTEST_STATUS=PASS
```

刷新结果：

```text
GO2_XT16_STATIC_ARTIFACT_REFRESH_RUNTIME_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_SNAPSHOT_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_HANDOFF_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_REPORT_CHECK_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_MANUAL_AUDIT_STATUS=PASS
GO2_XT16_STATIC_ARTIFACT_REFRESH_STATUS=PASS
```

刷新后的 artifact 仍然记录：

```text
GO2_XT16_STATUS_REPORT_OVERALL=NOT_READY
GO2_XT16_GATE_GAPS_REMAINING_GATES=gate1,gate2,gate3,gate4
GO2_XT16_GATE_GAPS_OVERALL=NOT_READY
GO2_XT16_REPORT_COMPLETION_STATUS=NOT_READY
```

解释：

```text
refresh runner 只保证本地状态 artifact 是当前生成的；
它不会补齐任何 gate，也不会替代现场 artifact。
报告最终完成条件仍然是四个 gate 全部写入 current evidence 后，
acceptance verifier 输出 GO2_XT16_ACCEPTANCE_STATUS=PASS。
```

## 29. 2026-07-04 report top status sync helper

新增报告顶层状态同步脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/sync_go2_xt16_axis_tilt_report_status.sh
```

用途：

```text
1. 从 current evidence verifier 的 status-report 读取 acceptance 状态。
2. 从 current evidence 读取 NO_RESIDUAL_RUNTIME_STATUS。
3. 只同步报告顶部“当前权威状态（2026-07-04）”代码块中的四行：
   report shape / runtime clean / acceptance / completion。
4. 默认会在真实写入前创建备份；--dry-run 只报告是否需要改写。
5. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
6. 不改变 current evidence，不让验收自动变成 PASS。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/sync_go2_xt16_axis_tilt_report_status.sh
scripts/sync_go2_xt16_axis_tilt_report_status.sh --self-test
scripts/sync_go2_xt16_axis_tilt_report_status.sh --dry-run
scripts/check_go2_xt16_report_completion.sh --allow-incomplete
scripts/check_go2_xt16_manual_gate_status.sh
```

self-test 结果：

```text
GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_PASS_UPDATE=PASS
GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_NOT_READY_UPDATE=PASS
GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_MALFORMED_REJECT=PASS
GO2_XT16_REPORT_STATUS_SYNC_SELFTEST_STATUS=PASS
```

当前 dry-run 结果：

```text
GO2_XT16_REPORT_STATUS_SYNC_REPORT=/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md
GO2_XT16_REPORT_STATUS_SYNC_EVIDENCE=/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_REPORT_STATUS_SYNC_RUNTIME_CLEAN=PASS
GO2_XT16_REPORT_STATUS_SYNC_ACCEPTANCE=NOT_READY
GO2_XT16_REPORT_STATUS_SYNC_COMPLETION=NOT_READY
GO2_XT16_REPORT_STATUS_SYNC_CHANGED=false
GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS
```

解释：

```text
CHANGED=false 表示报告顶部当前权威状态已经和 current evidence/verifier 一致。
但 acceptance/completion 仍然是 NOT_READY。
该同步器只防止报告顶层摘要和 evidence 脱节；
它不替代 RViz known-object、initial-pose TF、odom axis、supervised live short goal 四个 gate。
```

## 30. 2026-07-04 completion requirements audit

新增完成要求审计脚本：

```text
/home/lin/new2/dddmr_navigation/scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh
```

用途：

```text
1. 把 acceptance verifier 和 gate gap summary 的当前输出合成 requirement-to-evidence 矩阵。
2. 明确四个必须 gate：RViz known-object、initial-pose TF tilt、odom axis、supervised live short goal。
3. 输出 READY_TO_MARK_COMPLETE；只有 acceptance verifier 为 PASS 时才是 PASS。
4. --require-complete 在当前 NOT_READY 状态下必须失败，防止误把报告标记完成。
5. 不 source ROS，不启动 Docker，不检查 live topic，不发布任何 topic。
6. 不把旧 live/readiness 日志自动提升为 current evidence。
```

运行命令：

```bash
cd /home/lin/new2/dddmr_navigation
bash -n scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh
scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh --self-test
scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh
scripts/audit_go2_xt16_axis_tilt_completion_requirements.sh --require-complete
```

self-test 结果：

```text
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_CURRENT_RUN=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_REQUIRE_COMPLETE_BEHAVIOR=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_SELFTEST_STATUS=PASS
```

当前 audit 结果：

```text
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REPORT=/home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_EVIDENCE=/home/lin/new2/dddmr_navigation/docs/go2_xt16_axis_tilt_acceptance_evidence.env
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_MODE=no_motion_local_static
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FORBIDDEN_INTERFACES=/cmd_vel,/dddmr_go2/dry_run_cmd_vel,/api/sport/request,/lowcmd
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REQUIRED_GATES=gate1,gate2,gate3,gate4
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE1_RVIZ_FRONT_OBJECT_STATUS=NOT_READY
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE1_RVIZ_FRONT_OBJECT_REASON=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS_not_PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE1_RVIZ_FRONT_OBJECT_MISSING=RVIZ_BASE_LINK_FRONT_OBJECT_STATUS,RVIZ_FIXED_FRAME,RVIZ_FRONT_OBJECT_DIRECTION,SCREENSHOT:missing
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE2_INITIAL_POSE_TF_TILT_STATUS=NOT_READY
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE2_INITIAL_POSE_TF_TILT_REASON=TF_HEALTH_MAP_ODOM_STATUS_not_PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE2_INITIAL_POSE_TF_TILT_MISSING=TF_HEALTH_MAP_ODOM_STATUS,TF_HEALTH_STATUS,TF_HEALTH_LOG:missing
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE3_ODOM_AXIS_STATUS=NOT_READY
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE3_ODOM_AXIS_REASON=RAW_ODOM_AXIS_STATUS_not_PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE3_ODOM_AXIS_MISSING=RAW_ODOM_AXIS_STATUS,STANDARD_ODOM_AXIS_STATUS,RAW_STANDARD_ODOM_MATERIAL_MATCH,ODOM_AXIS_ROTATION_CHECK,RAW_ODOM_AXIS_LOG:missing,STANDARD_ODOM_AXIS_LOG:missing
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=NOT_READY
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REASON=GO2_XT16_SPORT_LIVE_READINESS_not_PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_MISSING=GO2_XT16_SPORT_LIVE_READINESS,SPORT_LIVE_READINESS_LOG:missing,SPORT_PROBE_RESULT,SPORT_PROBE_SUMMARY:missing,SPORT_PROBE_HAS_MOVE_1008,SPORT_PROBE_HAS_STOPMOVE_1003,LIVE_NAV_SHORT_GOAL_STATUS,LIVE_NAV_SUMMARY:missing,LIVE_NAV_LOG:missing,LIVE_NAV_MAX_X_MPS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FINAL_ACCEPTANCE_STATUS=NOT_READY
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REMAINING_GATES=gate1,gate2,gate3,gate4
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE=NOT_READY
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REASON=manual_or_supervised_gate_evidence_incomplete
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS=PASS
```

解释：

```text
该 audit 是“能否标记完成”的直接证据矩阵。
当前 READY_TO_MARK_COMPLETE=NOT_READY；
因此 /home/lin/new2/go2_xt16_axis_tilt_solution_20260703.md 仍不能标记完成。
只有四个 gate 全部写入 current evidence，并且 acceptance verifier 输出 PASS，
--require-complete 才允许通过。
```

## 31. 2026-07-04 Gate4 0.30m/s final acceptance update

本节记录 2026-07-04 现场监督后的最终 current evidence 状态。前文旧的
`NOT_READY` 输出只保留为历史记录；当前完成判定以本节、current evidence 和
本地 verifier/audit 输出为准。

现场 Gate4 结论：

```text
0.10m/s: 机器狗动不起来，不能作为 Gate4 运动验收速度。
0.30m/s: 机器狗能正确按导航点方向运动，定位正常。
```

已写入 current evidence 的 Gate4 artifact：

```text
SPORT_LIVE_READINESS_LOG=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/gate4_sport_readiness_20260704_074918.txt
SPORT_PROBE_SUMMARY=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/go2_sport_adapter_live_20260704_075049_summary.env
LIVE_NAV_SUMMARY=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/go2_xt16_nav_live_20260704_075358_summary.env
LIVE_NAV_LOG=/home/lin/new2/dddmr_navigation/run_logs/go2_xt16_axis_tilt/gate4_live_nav_0p30_acceptance_20260704_075358.log
LIVE_NAV_MAX_X_MPS=0.30
```

0.30m/s live request echo 摘要：

```text
REQUEST_ECHO_API_1008_COUNT=380
REQUEST_ECHO_API_1003_COUNT=174
REQUEST_ECHO_MAX_ABS_X=0.300000
REQUEST_ECHO_MAX_ABS_Z=0.165022
```

current verifier status-report：

```text
GO2_XT16_STATUS_REPORT_REPORT_LINK=PASS
GO2_XT16_STATUS_REPORT_GATE1_RVIZ_FRONT_OBJECT=PASS
GO2_XT16_STATUS_REPORT_GATE2_INITIAL_POSE_TF_TILT=PASS
GO2_XT16_STATUS_REPORT_GATE3_ODOM_AXIS=PASS
GO2_XT16_STATUS_REPORT_GATE4_SUPERVISED_LIVE_SHORT_GOAL=PASS
GO2_XT16_STATUS_REPORT_OVERALL=PASS
GO2_XT16_STATUS_REPORT_REASON=all_required_evidence_present
```

current gate-gap summary：

```text
GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_STATUS=PASS
GO2_XT16_GATE_GAPS_GATE1_RVIZ_FRONT_OBJECT_MISSING=none
GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_STATUS=PASS
GO2_XT16_GATE_GAPS_GATE2_INITIAL_POSE_TF_TILT_MISSING=none
GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_STATUS=PASS
GO2_XT16_GATE_GAPS_GATE3_ODOM_AXIS_MISSING=none
GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=PASS
GO2_XT16_GATE_GAPS_GATE4_SUPERVISED_LIVE_SHORT_GOAL_MISSING=none
GO2_XT16_GATE_GAPS_REMAINING_GATES=none
GO2_XT16_GATE_GAPS_OVERALL=PASS
```

report status sync：

```text
GO2_XT16_REPORT_STATUS_SYNC_ACCEPTANCE=PASS
GO2_XT16_REPORT_STATUS_SYNC_COMPLETION=PASS
GO2_XT16_REPORT_STATUS_SYNC_CHANGED=false
GO2_XT16_REPORT_STATUS_SYNC_STATUS=PASS
```

completion requirements audit：

```text
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE1_RVIZ_FRONT_OBJECT_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE2_INITIAL_POSE_TF_TILT_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE3_ODOM_AXIS_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_REQUIRED=supervised_live_short_goal_under_0p30mps_cap_PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_GATE4_SUPERVISED_LIVE_SHORT_GOAL_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_FINAL_ACCEPTANCE_STATUS=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REMAINING_GATES=none
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_READY_TO_MARK_COMPLETE=PASS
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_REASON=all_required_gate_evidence_verified
GO2_XT16_COMPLETION_REQUIREMENTS_AUDIT_STATUS=PASS
```

完成判定：

```text
GO2_XT16_ACCEPTANCE_STATUS=PASS
GO2_XT16_REPORT_ACCEPTANCE_STATUS=PASS
GO2_XT16_REPORT_COMPLETION_STATUS=PASS
GO2_XT16_REPORT_COMPLETION_REASON=all_required_evidence_present
```
