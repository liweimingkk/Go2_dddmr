# Go2 XT16 坡道与楼梯导航测试阶段实施方案

方案日期：2026-07-11

适用范围：当前 `dddmr_navigation` 工作区、Unitree Go2 EDU、顶部 Hesai
XT16、口部雷达、现有 DDDMR 三维建图/定位/规划链路。

本文是实施和验收方案，不代表坡道或楼梯能力已经通过实机验收。任何
受监督运动测试仍需单独获得现场运动授权，并遵守 `AGENTS.md` 的安全边界。

## 1. 结论和设计前提

目标不是把当前参数直接放宽，而是补齐地形模型、图边约束、局部支撑检查、
楼梯走廊行为和故障门控，使系统按以下顺序进入测试：

1. 坡道离线测试；
2. 坡道无运动回放和现场静止测试；
3. 坡道受监督实机测试；
4. 人工标注楼梯走廊的离线和无运动测试；
5. 正常导航步态下的受监督上楼测试；
6. 负障碍和停止能力通过后，再测试下楼。

本方案接受以下项目约束：

- Go2 上下楼梯继续使用当前正常 Sport 导航步态；
- 不新增 gait 切换命令，也不要求足步规划器控制每只脚；
- 最终执行接口仍然只使用当前 `Move(1008)` 和 `StopMove(1003)`；
- 楼梯期间必须监控 `sportmodestate.gait_type`，发生非预期变化即判失败；
- 虽然不切换机器狗步态，导航软件仍必须进入专用的“楼梯走廊状态”，限制
  路径、方向、速度、转向、重规划和恢复行为。

“正常步态能够通过楼梯”解决的是底层运动能力问题，不能替代导航层对楼梯
几何、入口对准、落差、动态障碍和错误路径的判断。

## 2. 测试就绪的定义

本项目使用三个不同状态，避免把“能出路径”误认为“已经可用”。

| 状态 | 定义 |
| --- | --- |
| 离线测试就绪 | 合成测试、历史地图和 bag 回放通过，不产生真实运动输出 |
| 现场测试就绪 | 只读预检、现场静止感知、故障注入和安全门通过，可以申请一次受监督运动测试 |
| 使用验收通过 | 在确定的地形能力包络内重复完成上/下行，且定位、支撑、停止和人工接管证据完整 |

本方案完成 P0～P1 后，目标是达到“坡道现场测试就绪”，P2 才执行受监督
现场验证；完成 P0～P3 后，目标是达到“人工标注楼梯的现场测试就绪”，P4
才执行受监督楼梯验证。任何阶段都不能仅凭一次通过宣布通用三维导航可用。

## 3. 当前阻塞项

### 3.1 地面分割阈值不是通行能力

当前 `ground_slope_tolerance: 0.3` 约为 17.2 度，但代码只有在坡角和
`ground_dz_tolerance` 同时超限时才拒绝点。它是 LeGO-LOAM 地面分类条件，
不能作为 Go2 的最大安全坡度、最大台阶或最大下落高度。

分割阈值和通行阈值必须分离：

- `segmentation.*` 只决定点云分类；
- `terrain.*` 决定地形几何和可信度；
- `capability.*` 表示经过现场验证的机器人能力；
- `planner.*` 使用能力包络内留有安全余量的限制。

### 3.2 地面图会产生跨层和跳级连接

`a_star_on_pc.cpp` 当前以三维半径搜索邻居，稀疏时扩大搜索半径，唯一的
坡度拒绝逻辑仍被注释。系统没有统一检查：

- 有符号上坡和下坡坡度；
- 上台阶和下台阶高度；
- 横坡；
- 边中间是否持续有地面支撑；
- 两个节点是否属于同一地面层；
- 楼梯边是否只连接相邻踏步。

`global_planner.cpp` 又会在两个节点间做直线插值，所以离散踏步可能被输出为
穿过立板或悬空的斜线路径。

### 3.3 当前地图分辨率不足以单独承担楼梯几何

当前 `complete_map_voxel_size` 是 0.2 m，地图服务器对完整地图和 ground
使用同一体素大小。这个分辨率可以继续服务全局定位和普通导航，但不足以稳定
保留常见踏步的踏面、立板和相邻级关系。

方案不把整张定位地图直接改成高分辨率，而是增加楼梯/坡道 ROI 地形图：

- 定位和普通全局图继续使用当前分辨率；
- 已标注的坡道和楼梯区域额外保存 0.03～0.05 m 的 `terrain_ground`；
- 两张图使用相同 map frame、地图版本和 SHA256 标识；
- 地形 ROI 缺失或版本不匹配时，楼梯能力保持禁用。

0.03～0.05 m 是地形建模的离线起始范围，不是已经验收的最终值。

### 3.4 普通局部 DWA 不能直接穿越楼梯

当前未来轨迹主要积分 `x/y/yaw`，没有根据前方地形更新未来 z、地面法向和
支撑面。固定实体 cuboid 又把腿部区域、踏面和立板统一按障碍点检查，结果是：

- 坡脚、坡顶和踏步可能使所有候选轨迹被拒绝；
- 如果粗暴删除所有楼梯点，又会删除真实墙体、平台边缘或楼梯上的异物；
- 控制持续失败后可能进入原地旋转 recovery，而台阶上不允许这种行为。

### 3.5 负障碍和命令门不完整

当前碰撞模型擅长检查“前方有点”，但缺少“轨迹下方没有可靠地面”的拒绝条件。
现有命令门主要检查命令和定位新鲜度，没有地形可信度、落差、横向误差、
姿态、进度和 gait 状态门控。

下坡、楼梯下降、平台边缘和坑必须在这一缺口关闭之后才能进入运动测试。

## 4. 目标架构

```text
mapground + terrain_ground + mapcloud
XT16 ground/non-ground + raw mouth LiDAR + odom/IMU
                  |
                  v
      TerrainModel / OnlineTerrainLayer
  slope, normal, support, surface, stair, drop, age
          |                         |
          v                         v
 TerrainEdgeValidator       TerrainSupportCritic
          |                         |
          v                         v
 terrain-aware A*       surface-following local trajectory
          \                         /
           v                       v
        Ramp/Stair Traversal Supervisor
    NORMAL/APPROACH/ALIGN/COMMITTED/LANDING/FAULT
                         |
                         v
             terrain-aware Go2 command gate
                         |
                         v
        existing Sport Move(1008) / StopMove(1003)
                  normal gait remains active
```

核心原则是默认拒绝：

- 连续地面只有全部边约束通过才可通行；
- 普通离散高度变化默认不可通行；
- 只有位于已标注走廊、在线几何确认成功的相邻楼梯踏步可以开放；
- `UNKNOWN` 不等于 `DROP`，但测试模式下两者都不能产生运动；
- 任何地形快照、定位或传感器超时都不得回退为 all-clear。

## 5. 地形数据合同

### 5.1 内部数据结构

在 `perception_3d` 中增加不可变、带版本的 `TerrainSnapshot`，不要继续复用
`PointXYZI.intensity` 存放多种语义。

建议结构如下：

```cpp
enum class TerrainClass {
  UNKNOWN, FLAT, RAMP, STAIR_TREAD, STAIR_RISER, EDGE, DROP
};

struct TerrainNode {
  uint32_t ground_index;
  Eigen::Vector3f normal;
  float slope_rad;
  float roughness_m;
  float support_ratio;
  float confidence;
  int32_t surface_id;
  int32_t staircase_id;
  int32_t step_index;
  TerrainClass terrain_class;
  uint32_t flags;
};

struct StaircaseModel {
  int32_t id;
  std::string map_hash;
  Eigen::Vector3f up_axis;
  float width_m;
  float riser_height_m;
  float tread_depth_m;
  int32_t step_count;
  float confidence;
  bool allow_up;
  bool allow_down;
};
```

`TerrainSnapshot` 必须包含地图哈希、版本号、时间戳、节点数组和楼梯模型。
节点与 `mapground` 索引严格同序；生成后只读，规划器在一次搜索中使用同一版本。

### 5.2 调试和运行 topic

第一版可用 `PointCloud2` 和结构化状态消息发布：

```text
/dddmr_terrain/traversability_cloud
/dddmr_terrain/support_cloud
/dddmr_terrain/unknown_cloud
/dddmr_terrain/drop_cloud
/dddmr_terrain/stair_markers
/dddmr_terrain/status
/dddmr_terrain/traversal_state
/dddmr_terrain/rejection_reason
/dddmr_terrain/speed_limit
```

`status` 至少包含：地形类别、surface/stair/step ID、坡度、支撑率、置信度、
横向/航向误差、数据年龄、允许方向和机器可读拒绝原因。

拒绝原因固定为枚举，例如：

```text
STALE, UNKNOWN, SLOPE, CROSS_SLOPE, STEP_UP, STEP_DOWN, DROP,
NO_SUPPORT, LAYER_MISMATCH, SKIP_STEP, OUTSIDE_CORRIDOR,
LOW_CONFIDENCE, BAD_ALIGNMENT, DYNAMIC_OBSTACLE, NO_LANDING
```

## 6. 具体改造模块

### 6.1 地图和人工走廊标注

修改 `dddmr_pg_map_server`：

- 分离 `complete_map_voxel_size` 和 `complete_ground_voxel_size`；
- 增加 `terrain_roi_voxel_size`；
- 发布 `map1/terrain_ground`；
- 地图产物写入分辨率、坐标系、源地图哈希和生成时间；
- 高分辨率 ROI 不存在时不影响平地导航，但禁止楼梯模式。

新增站点文件：

```text
src/dddmr_beginner_guide/config/go2_xt16_stair_corridors.yaml
```

每个楼梯记录：

- map SHA256；
- 走廊多边形和中心轴；
- 上/下入口平台及出口平台；
- 实测宽度、踏步高度/深度范围和级数；
- 是否允许上行、是否允许下行；
- 正常步态手动通过证据编号；
- 现场能力参数版本。

第一轮测试设置 `require_manual_corridor: true`。自动楼梯检测只负责在线确认或
拒绝，不允许在未知区域自行开放楼梯。这能先验证规划和控制链路，再逐步推广到
任意楼梯识别。

### 6.2 静态和在线地形层

新增：

```text
src/dddmr_perception_3d/include/perception_3d/terrain_model.h
src/dddmr_perception_3d/src/graph/terrain_model.cpp
src/dddmr_perception_3d/include/perception_3d/terrain_layer.h
src/dddmr_perception_3d/plugins/terrain_layer.cpp
```

并在插件 XML 和 CMake 中注册。

静态实例负责：

- 邻域 PCA 法向；
- 平面残差和粗糙度；
- 连续支撑率；
- 地表连通分量和 `surface_id`；
- 人工楼梯走廊中的踏步聚类和 `step_index`；
- 未知、边缘和落差区域；
- 构造版本化 `TerrainSnapshot`。

在线实例负责：

- 订阅实时 `/ground_cloud`，而不只使用 `segmented_cloud_pure`；
- 使用 odom/IMU 将局部观测投影到重力对齐地形帧；
- 融合 raw `/utlidar/cloud_base` 作为近脚地形观测；
- 对口部雷达做腿部/机体 mask、时间戳和 TF 检查；
- 区分 `OBSERVED_FREE`、`UNKNOWN` 和 `DROP`；
- 把无支撑、落差、过期区域生成为局部 virtual lethal evidence；
- 发布在线地形状态和诊断信息。

当前口部雷达固定 ROI 可以保留用于已有建图复现，但不能继续作为楼梯在线安全
判断。新地形输入必须单独发布，不能把 ROI 内所有点无条件追加成 ground。

### 6.3 统一图边验证器

可复用的纯地形边策略放在 `perception_3d`，因为 global planner 已经依赖
`perception_3d`；不能让 `static_layer` 反向依赖 global planner 形成包循环：

```text
src/dddmr_perception_3d/include/perception_3d/terrain_edge_policy.h
src/dddmr_perception_3d/src/graph/terrain_edge_policy.cpp
```

global planner 再增加面向 A* 和 KD-tree 的薄封装：

```text
src/dddmr_global_planner/include/global_planner/terrain_edge_validator.h
src/dddmr_global_planner/src/terrain_edge_validator.cpp
```

在以下三处使用同一 `TerrainEdgePolicy` 验证规则：

- `a_star_on_pc.cpp` 的实时邻居展开；
- `static_layer.cpp` 的预图建边；
- `a_star_on_pre_graph.cpp` 的预图搜索复核。

这样可以避免 direct A* 和 pre-graph 在同一张地图上给出不同通行结果。

连续坡面边必须满足：

- 相同 `surface_id`；
- 有符号上/下坡度分别小于已验收限制；
- 横坡、粗糙度和法向变化在限制内；
- 沿边按固定间距采样，最低支撑率达标；
- 中间没有未知、落差或致命障碍；
- 边距离不能因为图稀疏而绕过上述检查。

楼梯边必须满足：

- 同一 `staircase_id`；
- `step_index` 只相差 1；
- 位于人工走廊内；
- 方向与楼梯轴一致；
- 踏步高度和深度位于该走廊的实测范围；
- 在线地形确认有效；
- 禁止跳两级、跨楼层和半径接近但无支撑的捷径。

路径代价使用非负项：距离、坡度、横坡、粗糙度、风险和踏步惩罚。每条被拒绝
的边保存机器可读原因，不能只输出 `No path found`。

同时修改 `global_planner.cpp`：

- 连续坡道路径投影到拟合面；
- 楼梯输出踏面中心和相邻踏步 transition waypoint，不再穿立板插值；
- start/goal 按 z、surface、入口/平台选择，禁止跨层最近点；
- raw goal 只有通过支撑检查才可追加；
- DWA 子路径为空、断裂或版本不一致时立即返回失败，不能拼接原路径尾部。

### 6.4 楼梯几何与障碍语义

不能全局过滤楼梯点。正确做法是在已确认楼梯走廊内分类：

- `STAIR_TREAD`：支撑地形；
- 与模型一致的 `STAIR_RISER`：允许进入腿部运动包络，但不能进入机身包络；
- 侧墙、扶手、顶部净空、人员、箱体和走廊外点：继续作为致命障碍；
- 几何不匹配或置信度不足：整段走廊关闭。

修改静态层、旋转雷达层和 cluster marking：

- 只对“当前已确认楼梯 + 走廊内 + 与预期几何匹配”的 tread/riser 做语义分流；
- 原始点始终保留在调试 topic，不能静默删除；
- 动态障碍继续优先于楼梯通行；
- 不再只按 XY 把不同高度层的障碍投影到一起，至少要求相同 `surface_id` 和
  地面法向平面距离一致。

### 6.5 地形感知的局部轨迹和 critic

扩展 `DDSimpleTrajectoryGeneratorTheory`，或新增：

```text
src/dddmr_local_planner/trajectory_generators/include/trajectory_generators/terrain_following_dd_theory.h
src/dddmr_local_planner/trajectory_generators/theories/terrain_following_dd_theory.cpp
```

其行为是：

- `x/y/yaw` 采样接口保持不变；
- 每个未来 pose 从当前 `TerrainSnapshot` 获取支撑 z、法向和不确定度；
- 坡道按拟合地面更新未来 z/姿态；
- 楼梯按 transition waypoint 和身体安全包络计算，不假设机体沿第一块踏面
  无限外推；
- 地形快照缺失或版本变化时整条轨迹无效。

新增并注册 `mpc_critics::TerrainSupportModel`：

```text
src/dddmr_local_planner/mpc_critics/include/mpc_critics/terrain_support_model.h
src/dddmr_local_planner/mpc_critics/models/terrain_support_model.cpp
```

该 critic：

- 拒绝低支撑、未知、落差、跨走廊、跳级和姿态不匹配轨迹；
- 检查入口方向、横向误差和出口平台；
- 与 `CollisionModel` 分工：support critic 管地形接触，collision critic 继续管
  机身、墙体、人员和异物；
- 楼梯模式用“机身障碍包络 + 地形兼容腿部包络”，不再把完整腿部空间视为
  一个不可进入的刚性实体 cuboid。

### 6.6 正常步态楼梯监督状态机

在 `dddmr_p2p_move_base` 新增导航层状态机：

```text
src/dddmr_p2p_move_base/include/p2p_move_base/stair_traversal_supervisor.h
src/dddmr_p2p_move_base/src/stair_traversal_supervisor.cpp
```

`StairTraversalSupervisor` 不发送 gait 命令，只约束普通 Move/Stop 指令：

```text
NORMAL
  -> STAIR_PRECHECK
  -> STAIR_APPROACH
  -> STAIR_ALIGN
  -> STAIR_COMMITTED
  -> STAIR_LANDING_VERIFY
  -> NORMAL

任意状态 -> STAIR_FAULT_LATCH
```

状态要求：

- `STAIR_PRECHECK`：停在平台，确认完整走廊、方向、入口、出口和在线几何；
- `STAIR_APPROACH`：普通规划到达楼梯中心线前；
- `STAIR_ALIGN`：只允许在平平台修正朝向和横向误差；
- `STAIR_COMMITTED`：锁定楼梯模型，只允许沿轴前进和经过验证的极小连续 yaw；
- `STAIR_LANDING_VERIFY`：机器人完整进入出口平台并确认全身支撑后才交回普通导航；
- `STAIR_FAULT_LATCH`：StopMove 后保持锁存，禁止自动恢复，等待人工处理。

`STAIR_COMMITTED` 中明确禁止：

- 原地旋转；
- 横移；
- 普通局部绕障；
- 自动倒退；
- rotate/position recovery；
- 改走另一组踏步；
- 未经验证的自由重规划。

发生动态障碍时停止并锁存，不在楼梯中尝试绕行。是否能在具体踏步上稳定
StopMove，必须先单独验证；不能以平地停止能力推断楼梯停止能力。

### 6.7 速度、进度和命令安全门

当前配置的局部生成器 `min_vel_x` 为 0.30 m/s，已有现场记录表明更低的
Move 请求可能不能产生有效前进。因此不能简单把楼梯限速设成 0.10 m/s：速度
上限低于生成器最低样本时，会出现没有有效前进候选。

先测量正常步态的稳定可执行下限 `v_exec_min`，再分别取得：

```text
v_exec_min <= v_ramp_up_safe_max
v_exec_min <= v_ramp_down_safe_max
v_exec_min <= v_stair_up_safe_max
v_exec_min <= v_stair_down_safe_max
```

任一不等式不成立，对应场景保持禁用。不得通过让规划器输出“机器人实际上不动”
的速度来伪造低速安全。

扩展 `go2_nav_cmd_gate.py` 及纯函数策略测试：

- 订阅地形状态、遍历状态、定位、标准 odom、`/sportmodestate` 和 `/lowstate`；
- 所有输入必须有独立超时；
- ramp 根据上下行方向应用不同速度、加速度和 yaw 限制；
- stair committed 只允许已验证方向的正向 x，`y=0`；
- yaw 默认是 0，仅在专项试验证明后开放极小连续修正；
- 命令速度与 odom 进度长期不一致时判停滞或打滑；
- `mode`/`gait_type` 非预期变化时立即锁存；
- 所有阻断都输出明确原因；
- 故障后不自动恢复为运动状态。

最终 Sport adapter 继续只转换安全门后的 Twist，不承担地形判断，也不新增 gait
API。测试证据必须证明全程只有 Move/StopMove 请求且 `gait_type` 保持不变。
历史探针中 `mode/gait_type` 可能在运动时仍保持 0，因此不能只凭该字段证明没有
切换步态；还必须审计 `/api/sport/request`，确认不存在任何 gait 变更 API。

### 6.8 三维目标到达和恢复行为

当前到达判断把 x、y、z 合成距离，但参数名仍是 `xy_goal_tolerance`，当前 Go2
配置又设为 1.0 m。改为：

```text
goal_xy_tolerance
goal_z_tolerance
goal_yaw_tolerance
goal_surface_match_required
```

坡顶、楼梯入口和出口必须匹配指定 surface/landing，不能只因为三维欧氏距离进入
1 m 范围就成功。

恢复行为也要地形感知：

- 平地可保留现有受控 recovery；
- 坡道只允许经过验证的恢复集合；
- 楼梯入口对准后以及 committed 状态禁止自动 recovery；
- 空路径、空局部轨迹、地形过期和路径版本冲突全部 fail closed。

## 7. 参数合同

发布默认值必须保持地形运动禁用。以下数值只作为离线算法起点；坡度、踏步和
速度上限必须由现场能力证据写入 site profile。

```yaml
terrain:
  enabled: false
  fail_closed: true
  normal_radius_m: 0.30
  min_neighbors: 8
  max_plane_residual_m: 0.04
  edge_sample_spacing_m: 0.05
  support_radius_m: 0.10
  min_support_ratio: 0.80
  max_unknown_ratio: 0.0
  max_age_s: 0.20

  ramp:
    enabled: false
    max_up_slope_deg: 0.0
    max_down_slope_deg: 0.0
    max_cross_slope_deg: 0.0

  generic:
    max_step_up_m: 0.0
    max_step_down_m: 0.0

  stair:
    enabled: false
    require_manual_corridor: true
    require_online_confirmation: true
    min_confidence: 0.90
    confirm_frames: 5
    max_heading_error_deg: 8.0
    max_lateral_error_m: 0.10
    max_step_index_delta: 1
    allow_in_place_rotation: false
    allow_replanning_while_committed: false
```

这里的 `0.0` 表示尚未验收并拒绝通行，不是“无限制”。楼梯 riser/tread 范围、
速度和姿态阈值放在每个站点的 capability profile 中。

能力参数生成流程：

1. 测量坡角、楼梯高度/深度/宽度和表面材料；
2. 单独验证正常步态的速度死区、Move 跟踪和 StopMove 稳定性；
3. 通过受监督手动运动得到上/下行能力包络；
4. 规划限制设置在已验证包络内并保留安全余量；
5. profile 记录原始证据、日期、地图哈希和软件 commit；
6. 地图或机器人参数变化后自动失效，必须重新验收。

## 8. 分阶段实施路线

### P0：先关闭通用规划安全缺口

实现：

- `TerrainSnapshot` 和拒绝原因；
- direct A* 的 `TerrainEdgeValidator`；
- start/goal surface 和 z 选择；
- raw goal 支撑验证；
- DWA 空子路径 fail closed；
- xy/z/yaw 分离到达容差；
- 合成坡、台阶、落差和跨层测试。

完成条件：在没有显式能力 profile 时，平地回归不退化，所有坡、台阶、落差和
跨层样本均被拒绝；没有真实运动输出。

### P1：坡道离线和无运动测试

实现：

- 连续坡面法向、支撑、横坡和粗糙度；
- surface-following 全局路径；
- `TerrainSupportModel`；
- 地形状态接入命令门；
- 坡道专用配置、RViz 图层、录包和分析脚本。

完成条件：不同坡度上下行、横坡、坡脚和坡顶合成场景通过；真实坡道 bag 中路径
没有悬空边、跨层边或全轨迹误拒绝；传感器过期能在时限内阻断 safe cmd。

### P2：坡道受监督现场测试

顺序：

1. 现场只读传感器、TF 和 Docker 预检；
2. 机器人静止在坡底、坡中、坡顶，核对 roll/pitch/z 和地形分类；
3. 5 度、10 度，再到现场目标角度；
4. 每个角度先上坡、后下坡，横坡单独测试；
5. 每项至少连续 5 次无安全失败后才提升等级。

现场角度和次数是测试分级，不代表 Go2 的通用额定能力。任何一次出现定位跳变、
无支撑路径、打滑、超姿态、异常停止或人工急停，当前等级停止晋级。

### P3：人工标注楼梯的离线和无运动测试

实现：

- 高分辨率 `terrain_ground`；
- 手工 staircase corridor 和 map hash；
- 相邻踏步边、上下平台和在线几何确认；
- tread/riser 条件分流；
- 楼梯 transition waypoint；
- `StairTraversalSupervisor`；
- committed 状态的命令门和 recovery 禁用；
- 正常 gait 不变断言。

完成条件：清空楼梯时存在合法路径；跳两级、跨层、走廊外和下方无支撑路径全部
拒绝；在楼梯或出口放置人员/箱体时所有相关轨迹拒绝；全程不产生 gait API。

### P4：正常步态楼梯受监督测试

运动测试必须另行授权，并按以下顺序：

1. 平地验证正常 gait 的 `v_exec_min`、加减速和 StopMove；
2. 单级台阶上行；
3. 三阶短楼梯上行；
4. 完整楼梯上行；
5. 单级下降；
6. 三阶短楼梯下降；
7. 完整楼梯下降。

每一级别至少连续 5 次通过再升级。下楼必须在负障碍、出口平台和停止故障注入
全部通过后开放。失败后只能 StopMove、锁存和人工接管，不允许自动倒退或旋转。

### P5：从站点方案推广到通用地形

在 P4 形成稳定证据后再增加：

- 无人工标注的楼梯候选检测；
- 多传感器长期跟踪和置信度融合；
- 自动生成 staircase corridor；
- 多层建筑的 transition graph；
- 地图更新后的走廊重新验证；
- 更多表面材料、光照、遮挡、人员和载荷条件。

自动检测只能提出候选，是否开放仍受 capability profile、在线确认和 fail-closed
门控控制。

## 9. 测试矩阵和验收指标

| 阶段 | 场景 | 必须通过 | 失败出口 |
| --- | --- | --- | --- |
| G0 单元测试 | 平地、5/10/15/20 度坡、台阶、坑、断边、重叠楼层、稀疏/NaN 点云 | 所有合法边连通；所有越限、无支撑、跨层和跳级边拒绝 | 任一危险边被接受即停止 |
| G1 合成 ROS 集成 | 地图、TF 和地形 topic，无运动输出 | action 结果、frame、路径和拒绝原因确定；空路径不拼接 | 成功但路径为空/断裂，或输出 safe cmd |
| G2 bag 回放 | 真实坡、楼梯、平台边缘、口部雷达遮挡 | 分类稳定；路径无 unsupported edge；动态障碍优先；超时 fail closed | UNKNOWN/DROP 被当可通行 |
| G3 现场静止 | 坡底/中/顶、楼梯入口/平台 | TF/odom 姿态和实测一致；支撑/落差与现场几何一致 | 数据过期、跨层或分类抖动 |
| G4 受监督坡道 | 上坡、下坡、横坡、坡顶停留 | 每级连续通过；无滑移、碰撞、定位丢失和错误到达 | 任一安全事件立即降级 |
| G5 受监督楼梯 | 单级、短梯、完整楼梯，先上后下 | 入口对准、相邻踏步、出口平台、gait 不变、零 recovery | 几何不足、动态障碍、gait 变化或无支撑均锁存 |

核心指标：

- `unsupported_edge_count == 0`；
- 已知危险合成样本 `false_traversable_count == 0`；
- 楼梯边 `abs(step_index_delta) == 1`；
- 10 Hz 输入下地形处理 95 分位延迟目标小于 100 ms；
- 地形数据过期后不超过 0.30 s 阻断运动；
- 楼梯 committed 状态原地旋转、横移、自动 recovery 次数为 0；
- path、terrain snapshot 和 map hash 始终一致；
- `sportmodestate.gait_type` 全程不变；
- Sport 请求仅包含批准的 Move/StopMove；
- 目标 surface、z 和 landing 验证通过后才报告成功。

延迟指标是基于当前 10 Hz XT16 链路的工程目标，必须由实测 CPU 负载确认。

## 10. 故障停止条件

出现任一条件，命令门立即输出零、触发 StopMove 并进入故障锁存：

- 定位不再是 `TRACKING`；
- TF、odom、XT16、口部雷达、地形状态或决策心跳超时；
- 地图哈希或 terrain snapshot 版本不一致；
- 楼梯几何、走廊、入口或出口置信度不足；
- 前方或出口平台出现动态障碍；
- 横向误差、航向误差、roll/pitch 或角速度超过已验证阈值；
- 命令速度与实测速度持续不一致；
- 出现打滑、停滞、错误高度方向或异常足端接触；
- 路径出现无支撑边、跳级边或跨层边；
- 所有局部轨迹被拒绝；
- 穿越距离、时间或踏步数超过预期；
- Sport mode/gait 非预期变化；
- 人工急停。

故障后禁止自动原地旋转、倒退、重规划或继续前进。现场人员负责把机器人从不稳定
位置安全回收。故障处理本身不授权任何自动运动。

## 11. 必须记录的证据

扩展 `scripts/record_go2_xt16_nav_debug_bag.sh`，至少记录：

```text
/lidar_points
/ground_cloud
/segmented_cloud_pure
/utlidar/cloud_base
/dddmr_go2/robot_odom_standard
/localization_status
/tf
/tf_static
/lowstate
/sportmodestate
/global_path
/awared_global_path
/dddmr_go2/p2p_decision
/dddmr_go2/dry_run_cmd_vel
/dddmr_go2/safe_cmd_vel
/api/sport/request              # 仅经授权的实机阶段
/dddmr_terrain/*
```

每次测试同时保存：

- git commit、容器镜像和配置哈希；
- map/terrain/corridor/capability profile 哈希；
- 实测坡角或楼梯尺寸；
- 表面材料、载荷和环境；
- 试验方向、次数和失败样本；
- 最大 roll/pitch、横向误差、速度误差、停止距离和地形延迟；
- gait 前后值和 Sport API ID 清单；
- 人工观察和急停记录。

没有上述证据的单次“走过去了”不能提升能力等级。

## 12. 完成地形能力后，DDDMR 的进一步优化

### P0：与地形改造同步处理

1. 修正 `StaticGraph` 所有权。

   当前类持有裸 `graph_ptr_`，而全局规划器又进行值拷贝，存在浅拷贝、悬空或
   重复释放风险。改成 RAII 容器或不可变 `shared_ptr<const StaticGraph>`，并增加
   拷贝/更新测试。

2. 修正图更新路径。

   `A_Star_on_Graph::updateGraph()` 没有完整更新成员 point cloud；pre-graph 模式
   地图更新时又可能解引用未创建的 direct planner。按模式更新并使用版本快照。

3. 修正 DynamicGraph 边界和隐式插入。

   `initial()` 当前使用 `i <= n`，会比分配目标多一个节点；`operator[]` 查询还会
   隐式创建键。改为定长 vector、显式边界检查和只读查询。

4. 修正多层 start/goal 和路径尾姿态。

   垂直搜索找到 ground 后仍可能返回失败；raw goal 未验证即追加；最后路径点的
   朝向依赖未赋值的 next pose。增加多层、空路径和单节点路径测试。

5. 将所有规划失败变成结构化结果。

   action 返回具体 reason、地图版本和拒绝统计，避免依赖日志文本判断
   `No path found`、碰撞、无支撑或数据过期。

### P1：可维护性和可观测性

1. 单一机器人几何来源。

   当前 cuboid 在多个 generator 和 critic 配置中重复。建立一个 Go2 几何 profile，
   启动时展开并校验，避免某个插件使用旧尺寸。

2. 参数 schema 和启动即失败。

   对分辨率、频率、半径、速度上下限、topic、frame 和 timeout 做统一校验；无效
   参数不使用危险默认值继续运行。

3. 地图 promotion manifest。

   地图必须记录传感器合同、mouth fusion 状态、点数/z 分布、surface 数、哈希、
   生成配置和通过的合成回归。导航只加载已 promotion 的地图。

4. 统一健康状态。

   聚合 TF、定位、点云频率、perception、global/local planner、terrain、cmd gate 和
   Sport adapter 状态，形成一个可机器读取的 readiness topic。

5. 统一速度仲裁。

   现有 speed limit 共享单个标量，改成各层提交带来源和时间戳的限制，最终取最小
   有效值；任何 safety layer 过期输出 STOP，而不是恢复为无限制。

### P2：性能和长期鲁棒性

1. 版本化只读快照，缩短锁范围。

   当前 A* 搜索会长时间持有 ground mutex。感知构建新快照后原子替换，规划只读
   固定版本；完成时发现版本改变则重试，减少感知阻塞。

2. XT16 数据利用。

   完成 ring 和硬件逐点 timestamp 的使用验证，监控去畸变、稀疏远距坡面和
   坡脚/坡顶地面连续性。

3. 定位置信度进入规划与门控。

   除 `TRACKING` 文本外，暴露 6DoF 协方差、创新量、地图匹配分数和跳变原因；
   坡道和楼梯使用更严格的 z/roll/pitch 质量门。

4. 确定性合成 CI。

   将墙体、门洞、坡道、横坡、单台阶、多级楼梯、坑、断崖、上下层、动态障碍和
   传感器超时放入 CI。每次修改 planner/perception/config 都必须运行。

5. 计算预算和降级策略。

   记录地形构建、图更新、A*、局部评分的平均值和 95/99 分位时间；超预算时停止
   开放新地形，不得跳过安全检查维持速度。

## 13. 建议提交拆分

为避免一次改动同时触碰感知、规划和真实运动链路，按可独立验证的提交拆分：

1. `fix planner fail-closed invariants`；
2. `add terrain model and synthetic fixtures`；
3. `enforce terrain-aware graph edges`；
4. `add terrain support critic and goal semantics`；
5. `publish high-resolution terrain ROI`；
6. `add ramp dry-run profile and evidence tooling`；
7. `add annotated stair corridor model`；
8. `add normal-gait stair traversal supervisor`；
9. `gate Go2 commands on terrain state`；
10. `add supervised ramp/stair runbooks`。

每个提交都应运行 `./scripts/check-repository.sh`、目标包构建、对应单元测试和合成
回归。运动脚本和真实 Sport 输出变更单独审查，不能和纯算法提交混合。

## 14. 最终完成标准

坡道进入测试阶段需要同时满足：

- 图边有 signed slope、cross-slope、support、surface 和 drop 约束；
- 局部轨迹使用未来地形支撑；
- 负障碍和地形超时接入命令门；
- xy/z/yaw 到达条件正确；
- 合成、bag 和现场静止测试通过；
- 已生成现场 capability profile；
- 才能申请受监督运动测试。

楼梯进入测试阶段需要额外满足：

- 高分辨率 terrain ROI 和地图哈希一致；
- 人工走廊、踏步序号、上下平台和方向完整；
- 仅相邻踏步可连边；
- tread/riser 与真实障碍条件分流；
- 楼梯 committed 状态禁止旋转、绕行、倒退和 recovery；
- 正常 gait 可执行速度区间与安全速度区间有交集；
- `gait_type` 不变和仅 Move/StopMove 的自动证据通过；
- 上楼通过后，负障碍和故障停止通过才能申请下楼测试。

按照这一方案，机器狗不需要切换楼梯步态；需要新增的是地形理解、楼梯走廊、
局部支撑和导航安全状态，而不是新的底层步态控制器。
