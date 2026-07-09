# Go2 Sport Yaw / DDDMR Live Navigation 问题分析与解决方案

生成时间：2026-07-03  
适用对象：`go2_sport_yaw_issue_packet_20260703_112809.tar.gz` 中的当前代码与日志

---

## 1. 结论

当前 live navigation 卡住的主因不是建图、定位、RViz goal、DDS 或 `/api/sport/request` 发布链路失败，而是：

```text
DDDMR 在初始航向对齐 / 恢复行为阶段输出纯 yaw-only cmd_vel；
Go2 Sport API 在当前高层状态下对 Move(x=0,y=0,z=...) 的整机转向响应很弱；
因此机器人长期停留在 heading alignment / rotate recovery，不能稳定进入正常前进控制。
```

已验证事实：

```text
1. RViz goal 能进入 p2p_move_base。
2. 目标 (-0.75,-1.02) 一类局部目标能规划出 path。
3. Docker 内 DDDMR 发布 /dddmr_go2/dry_run_cmd_vel。
4. Host adapter 能把 Twist 转成 /api/sport/request 的 Move(api_id=1008)。
5. 当前 live 请求流主要是 yaw-only：{"x":0.0,"y":0.0,"z":-0.15}。
6. 纯 yaw-only Move(z=±0.25) 有符号正确但幅值很小，且负 yaw 有明显 rebound。
7. 加小前进量 x=0.05,z=-0.25 后，Go2 出现第一组明显整机 yaw 响应。
```

当前不建议继续直接跑完整 RViz live navigation。下一步应先做 **Go2 yaw alignment 命令形态修正**，并且修正必须是**有状态门控的**，不能简单把所有 yaw-only 命令都注入 `x=0.05`。

---

## 2. 证据摘要

### 2.1 RViz live navigation 已经能规划，但输出是 yaw-only

`go2_xt16_nav_live_20260703_103844` 中：

```text
Goal: (-0.75, -1.02, 0.00)
Selected start: node 1518
Selected goal: node 244
Path found from: 1518 to 244
Local planner: Recieve new global plan.
DWA: DWA goal reach the global path end at: -0.75, -1.02, 0.00
```

但是 Sport request echo 中 Move 请求几乎全部为：

```json
{"x":0.0,"y":0.0,"z":-0.15}
```

这说明问题点已经越过“规划路径是否存在”，进入“heading alignment 命令是否能让 Go2 真实转向”。

---

### 2.2 纯 yaw-only 的 Go2 Sport Move 响应弱

探针结果：

| Probe | 命令 | Move-window yaw | 全窗口 yaw | 解释 |
|---|---:|---:|---:|---|
| `104850` | `x=0,y=0,z=+0.25` | `+0.0326 rad` | `+0.0325 rad` | 符号正确，但响应小 |
| `105752` | `x=0,y=0,z=-0.25` | `-0.0213 rad` | `-0.0059 rad` | 响应小，Stop 后回弹明显 |
| `110122` | `BalanceStand` 后 `x=0,y=0,z=-0.25` | `-0.0188 rad` | `-0.0016 rad` | BalanceStand 预命令未解决 |
| `112316` | `x=0.03,y=0,z=-0.25` | `-0.0131 rad` | `+0.0039 rad` | `x=0.03` 太弱 |
| `112501` | `x=0.05,y=0,z=-0.25` | `-0.0719 rad` | `-0.0436 rad` | 第一组有意义的整机响应 |

其中 `x=0.05,z=-0.25` 的位移约 `0.0327 m`，Stop 后保留 yaw 约 `-0.048 rad`。这证明小前进量能更容易激活 Go2 的可行走姿态/步态响应。

---

## 3. 根因分析

### 3.1 DDDMR 的 state machine 对差速底盘假设较强

`p2p_move_base.cpp` 中初始航向对齐状态：

```cpp
else if(STATE_->isCurrentDecision("d_align_heading")){
  if(LP_->isInitialHeadingAligned()){
    STATE_->setDecision("d_controlling");
  }
  else{
    base_trajectory::Trajectory best_traj;
    dddmr_sys_core::PlannerState PS =
      LP_->computeVelocityCommand("differential_drive_rotate_shortest_angle", best_traj);
    ...
    publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
  }
}
```

配置里 `differential_drive_rotate_shortest_angle.rotation_speed = 0.15`，所以输出会是类似：

```text
linear.x = 0
linear.y = 0
angular.z = ±0.15
```

这对轮式差速底盘合理，但对当前 Go2 Sport API 高层控制不够可靠。

---

### 3.2 当前 adapter 是透明映射，不会修正 yaw-only

`go2_sport_cmd_vel_adapter.py` 当前转换逻辑基本是：

```python
x = msg.linear.x
 y = msg.linear.y
 yaw = msg.angular.z
```

然后限幅后发布：

```json
{"x": x, "y": y, "z": yaw}
```

所以 DDDMR 的 yaw-only 会原样变成 Go2 Sport `Move(0,0,z)`。当前实机证据显示这种命令无法稳定产生整机转向。

---

### 3.3 不能盲目给所有 yaw-only 加 x

虽然 `x=0.05,z=-0.25` 有效果，但不能直接做：

```text
凡是 yaw-only，一律变成 x=0.05 + yaw
```

原因是日志里也出现过规划失败和 recovery rotate：

```text
No path found from: 1518 to 1441
No path found from: 1518 to 1380
recovery_behaviors: rotate_inplace
```

这些 recovery rotate 也会输出 yaw-only。如果 adapter 无状态地给 recovery yaw-only 注入前进量，机器人可能在“无可行路径 / 规划失败 / 恢复行为”中向前挪动，这是不安全的。

所以修正必须满足：

```text
只允许在 heading alignment 类状态中做 yaw arc shim；
禁止在 recovery / no path / state unknown 下做前进注入；
state 不新鲜时宁可停止或保持纯 yaw，不允许偷偷前进。
```

---

## 4. 推荐架构

保留当前 mixed runtime：

```text
Docker:
  DDDMR navigation + RViz
  /cmd_vel remap -> /dddmr_go2/dry_run_cmd_vel

Host:
  go2_sport_cmd_vel_adapter.py
  /dddmr_go2/dry_run_cmd_vel -> /api/sport/request
```

新增一个状态门控链路：

```text
p2p_move_base
  -> /dddmr_go2/p2p_decision      # std_msgs/String，例如 d_align_heading

adapter:
  subscribe /dddmr_go2/dry_run_cmd_vel
  subscribe /dddmr_go2/p2p_decision
  if yaw-only and decision in allowed states:
      optional x-forward yaw arc shim
  else if yaw-only and decision not allowed:
      no forward injection; live mode optionally StopMove
```

最终输出：

```text
allowed heading alignment:
  Twist(0,0,±yaw) -> Move(x=0.05, y=0, z=±yaw)

recovery / no path / unknown state:
  Twist(0,0,±yaw) -> no x injection
  live policy: either pure yaw only, or safer StopMove/zero
```

建议第一版 live policy 使用更保守的：

```text
state 不允许 shim 时，yaw-only 直接 StopMove/zero。
```

这样 recovery rotate 不会导致机器人向前挪动。

---

## 5. Codex 修改任务

### 5.1 在 `p2p_move_base` 发布当前决策状态

修改文件：

```text
src/dddmr_p2p_move_base/src/p2p_move_base.cpp
src/dddmr_p2p_move_base/include/p2p_move_base/p2p_move_base.h
package.xml / CMakeLists 如需要增加 std_msgs 依赖
```

新增 publisher：

```cpp
#include <std_msgs/msg/string.hpp>

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
```

初始化：

```cpp
decision_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/dddmr_go2/p2p_decision", 1);
```

在 `executeCycle()` 每一轮末尾或每次 decision 更新后发布：

```cpp
void P2PMoveBase::publishDecisionState()
{
  if (!decision_pub_) return;
  std_msgs::msg::String msg;
  msg.data = STATE_->getCurrentDecision();
  decision_pub_->publish(msg);
}
```

最低要求：在 `executeCb()` 循环中每次 `executeCycle()` 后发布一次：

```cpp
bool done = executeCycle(goal_handle);
publishDecisionState();
```

状态字符串应能看到：

```text
d_planning
d_planning_waitdone
d_align_heading
d_controlling
d_align_goal_heading
d_recovery_waitdone
```

---

### 5.2 给 adapter 增加 yaw arc shim 参数

修改文件：

```text
src/ddmr_beginner_guide/scripts/go2_sport_cmd_vel_adapter.py
src/ddmr_beginner_guide/scripts/go2_sport_cmd_vel_dry_run.py
```

> 注意：真实路径应是 `src/dddmr_beginner_guide/...`，上面如果 Codex 看到少了一个 `d`，以实际仓库路径为准。

新增参数：

```python
self.enable_yaw_arc_shim = bool(
    self.declare_parameter("enable_yaw_arc_shim", False).value
)
self.yaw_arc_shim_mode = self.declare_parameter(
    "yaw_arc_shim_mode", "off"  # off | preview | live
).get_parameter_value().string_value
self.yaw_arc_forward_x = float(
    self.declare_parameter("yaw_arc_forward_x", 0.05).value
)
self.yaw_arc_min_abs_yaw = float(
    self.declare_parameter("yaw_arc_min_abs_yaw", 0.15).value
)
self.yaw_arc_trigger_abs_yaw = float(
    self.declare_parameter("yaw_arc_trigger_abs_yaw", 0.03).value
)
self.yaw_arc_allowed_decisions = list(
    self.declare_parameter(
        "yaw_arc_allowed_decisions",
        ["d_align_heading", "d_align_goal_heading"],
    ).value
)
self.decision_topic = self.declare_parameter(
    "decision_topic", "/dddmr_go2/p2p_decision"
).get_parameter_value().string_value
self.decision_timeout_sec = float(
    self.declare_parameter("decision_timeout_sec", 0.30).value
)
self.zero_yaw_only_when_shim_disallowed = bool(
    self.declare_parameter("zero_yaw_only_when_shim_disallowed", True).value
)
self.max_continuous_yaw_arc_sec = float(
    self.declare_parameter("max_continuous_yaw_arc_sec", 2.0).value
)
```

新增 decision subscriber：

```python
from std_msgs.msg import String

self.current_decision = ""
self.last_decision_time = None
self.create_subscription(String, self.decision_topic, self.decision_cb, 10)
```

```python
def decision_cb(self, msg: String) -> None:
    self.current_decision = msg.data
    self.last_decision_time = self.get_clock().now()
```

---

### 5.3 adapter 转换逻辑

把 `to_sport_command()` 拆成两步：

```text
1. raw Twist -> clamped Sport command
2. optional yaw arc shim
```

伪代码：

```python
def to_sport_command(self, msg: Twist) -> Tuple[float, float, float]:
    x, y, yaw = self.map_axis_and_clamp(msg)
    x2, y2, yaw2, shim_info = self.apply_yaw_arc_shim(x, y, yaw)
    self.last_shim_info = shim_info
    return x2, y2, yaw2
```

核心判断：

```python
def is_decision_fresh_and_allowed(self) -> bool:
    if self.last_decision_time is None:
        return False
    age = (self.get_clock().now() - self.last_decision_time).nanoseconds / 1e9
    if age > self.decision_timeout_sec:
        return False
    return self.current_decision in self.yaw_arc_allowed_decisions
```

```python
def apply_yaw_arc_shim(self, x, y, yaw):
    yaw_only = (
        abs(x) <= self.linear_deadband and
        abs(y) <= self.linear_deadband and
        abs(yaw) >= self.yaw_arc_trigger_abs_yaw
    )

    if not self.enable_yaw_arc_shim or self.yaw_arc_shim_mode == "off" or not yaw_only:
        return x, y, yaw, "none"

    allowed = self.is_decision_fresh_and_allowed()

    if not allowed:
        if self.zero_yaw_only_when_shim_disallowed:
            return 0.0, 0.0, 0.0, f"blocked_state={self.current_decision}"
        return x, y, yaw, f"pass_no_shim_state={self.current_decision}"

    shim_yaw = math.copysign(max(abs(yaw), self.yaw_arc_min_abs_yaw), yaw)
    shim_yaw = self.clamp(shim_yaw, -self.max_yaw, self.max_yaw)
    shim_x = self.clamp(self.yaw_arc_forward_x, 0.0, self.max_x)

    if self.yaw_arc_shim_mode == "preview":
        # 只记录，不改变真实输出
        return x, y, yaw, f"preview x={shim_x} yaw={shim_yaw} state={self.current_decision}"

    if self.yaw_arc_shim_mode == "live":
        return shim_x, 0.0, shim_yaw, f"live x={shim_x} yaw={shim_yaw} state={self.current_decision}"

    return x, y, yaw, "none"
```

日志必须同时打印原始和转换后的命令：

```text
adapter command:
  decision=d_align_heading
  original_sport={"x":0.0,"y":0.0,"z":-0.15}
  transformed_sport={"x":0.05,"y":0.0,"z":-0.15}
  shim=live
```

---

### 5.4 连续 yaw arc 时间限制

为了避免机器人在对齐阶段长时间画弧前进，adapter 需要一个连续 yaw arc 限制。

规则：

```text
如果 live shim 连续激活超过 max_continuous_yaw_arc_sec，立刻 StopMove，并 latch fault。
只有收到非 yaw-only 正常控制、zero cmd_vel、或新 goal reset 事件后才解除。
```

第一版没有 goal reset topic 时，可以简化为：

```text
连续 yaw arc 超时后 StopMove；
需要重启 adapter 或手动 reset parameter/service 才继续。
```

这比在 heading alignment 中无限小弧线漂移安全。

建议默认：

```yaml
max_continuous_yaw_arc_sec: 2.0
```

---

### 5.5 launch 增加 adapter 参数

修改：

```text
src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
scripts/run_go2_xt16_navigation_supervised_live.sh
```

新增 launch args：

```xml
<arg name="go2_sport_enable_yaw_arc_shim" default="false"/>
<arg name="go2_sport_yaw_arc_shim_mode" default="off"/>
<arg name="go2_sport_yaw_arc_forward_x" default="0.05"/>
<arg name="go2_sport_yaw_arc_min_abs_yaw" default="0.15"/>
<arg name="go2_sport_yaw_arc_trigger_abs_yaw" default="0.03"/>
<arg name="go2_sport_decision_topic" default="/dddmr_go2/p2p_decision"/>
<arg name="go2_sport_decision_timeout_sec" default="0.30"/>
<arg name="go2_sport_zero_yaw_only_when_shim_disallowed" default="true"/>
<arg name="go2_sport_max_continuous_yaw_arc_sec" default="2.0"/>
```

传给 adapter：

```xml
<param name="enable_yaw_arc_shim" value="$(var go2_sport_enable_yaw_arc_shim)" type="bool"/>
<param name="yaw_arc_shim_mode" value="$(var go2_sport_yaw_arc_shim_mode)" type="str"/>
<param name="yaw_arc_forward_x" value="$(var go2_sport_yaw_arc_forward_x)" type="float"/>
<param name="yaw_arc_min_abs_yaw" value="$(var go2_sport_yaw_arc_min_abs_yaw)" type="float"/>
<param name="yaw_arc_trigger_abs_yaw" value="$(var go2_sport_yaw_arc_trigger_abs_yaw)" type="float"/>
<param name="decision_topic" value="$(var go2_sport_decision_topic)" type="str"/>
<param name="decision_timeout_sec" value="$(var go2_sport_decision_timeout_sec)" type="float"/>
<param name="zero_yaw_only_when_shim_disallowed" value="$(var go2_sport_zero_yaw_only_when_shim_disallowed)" type="bool"/>
<param name="max_continuous_yaw_arc_sec" value="$(var go2_sport_max_continuous_yaw_arc_sec)" type="float"/>
```

`run_go2_xt16_navigation_supervised_live.sh` 默认仍然关闭 shim：

```text
enable_yaw_arc_shim=false
shim_mode=off
```

只有明确环境变量才开启：

```bash
GO2_ENABLE_YAW_ARC_SHIM=true
GO2_YAW_ARC_SHIM_MODE=preview   # 先 preview
```

live 模式必须要求新的确认词，例如：

```bash
GO2_YAW_ARC_SHIM_CONFIRM=I_AM_SUPERVISING_GO2_YAW_ARC_SHIM
```

---

## 6. 命令形态验证计划

在重新跑 RViz live navigation 前，先做低层 supervised probe。

### 6.1 先测试 navigation 实际 yaw 幅值对应的 arc

当前 live navigation 输出 yaw 是：

```text
z = -0.15
```

而目前真正有效的探针是：

```text
x=0.05,z=-0.25
```

所以必须补测：

```bash
GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE \
GO2_YAW_PROBE_X=0.05 \
GO2_YAW_PROBE_Y=0.0 \
GO2_YAW_PROBE_YAW=-0.15 \
GO2_YAW_PROBE_DURATION=0.6 \
RVIZ=false \
PUBLISH_STATIC_TF=true \
scripts/run_go2_yaw_feedback_probe.sh --live
```

再测正方向：

```bash
GO2_YAW_PROBE_CONFIRM=I_AM_SUPERVISING_GO2_YAW_PROBE \
GO2_YAW_PROBE_X=0.05 \
GO2_YAW_PROBE_Y=0.0 \
GO2_YAW_PROBE_YAW=0.15 \
GO2_YAW_PROBE_DURATION=0.6 \
RVIZ=false \
PUBLISH_STATIC_TF=true \
scripts/run_go2_yaw_feedback_probe.sh --live
```

通过标准：

```text
1. yaw 符号正确。
2. Move-window yaw 绝对值 >= 0.03 rad。
3. Stop 后 0.5s 保留 yaw 绝对值 >= 0.02 rad。
4. 位移 <= 0.05 m。
5. StopMove 后速度明显归零。
```

如果 `z=±0.15` 太弱，再依次测试：

```text
x=0.05,z=±0.20
x=0.05,z=±0.25
```

然后把以下三者保持一致：

```text
DDDMR rotation_speed
adapter max_yaw
adapter yaw_arc_min_abs_yaw
```

不要出现：

```text
planner 只给 0.15，但实际需要 0.25 才能转
```

除非 adapter 明确、可追踪地把 yaw-only 最小幅值提升到 `0.25`，并且日志中清楚记录。

---

### 6.2 测试 state-gated adapter shim，不接 RViz goal

先不要跑完整导航。用一个 synthetic publisher 模拟：

```text
/dddmr_go2/p2p_decision = d_align_heading
/dddmr_go2/dry_run_cmd_vel = Twist angular.z=-0.15
```

预期：

```text
/api/sport/request: {"x":0.05,"y":0.0,"z":-0.15}
```

然后模拟 recovery：

```text
/dddmr_go2/p2p_decision = d_recovery_waitdone
/dddmr_go2/dry_run_cmd_vel = Twist angular.z=-0.15
```

预期：

```text
不注入 x；
若 zero_yaw_only_when_shim_disallowed=true，则发布 StopMove / zero。
```

再模拟 state timeout：

```text
停止发布 /dddmr_go2/p2p_decision 超过 0.3s
继续发 yaw-only cmd_vel
```

预期：

```text
不注入 x；
live 模式 StopMove。
```

---

## 7. RViz live navigation 重试条件

只有满足以下条件后，才允许重新跑完整 RViz click goal：

```text
1. x=0.05,z=±0.15 或 ±0.20/±0.25 的双方向 probe 通过。
2. adapter preview 模式确认只在 d_align_heading / d_align_goal_heading 注入 x。
3. recovery/no path 下 yaw-only 不会注入 x。
4. /api/sport/request echo 能看到 transformed command。
5. StopMove cleanup 仍然工作。
6. 现场人员准备急停。
```

第一次 RViz 目标要求：

```text
1. 目标非常近：0.3m ~ 0.5m。
2. 目标方向与当前朝向误差小，尽量 < 30 deg。
3. 只在地图连通区域内。
4. 不靠近墙、桌腿、楼梯、地图边缘。
5. 运行最长 5~8 秒，超时立即 StopMove。
```

---

## 8. 关于 `heading_align_angle` 的建议

当前：

```yaml
heading_align_angle: 0.5   # 约 28.6 deg
```

这意味着如果路径起始方向和机身朝向差超过约 29 度，就会进入 `d_align_heading`，输出 rotate-shortest-angle，也就是当前问题触发点。

短期可以保留 `0.5`，前提是 yaw arc shim 已经验证可用。

不建议直接把它设成 `3.14` 来跳过 heading alignment。虽然这样可能让 DWA 直接输出 forward+turn，但会带来两个问题：

```text
1. DWA 会在很大航向误差下尝试前进，碰撞风险更高。
2. 问题会从 heading alignment 转移到 local control，调试更难。
```

如果需要降低对齐停留时间，可以小幅放宽：

```yaml
heading_align_angle: 0.7   # 约 40 deg
```

但这应在低速短目标测试通过后再做。

---

## 9. 关于 Sport gait / mode API 的建议

当前探针中 `/sportmodestate.mode` 和 `gait_type` 一直是 `0`，即使 `x=0.05,z=-0.25` 也一样。这说明不能仅凭这些字段判断 Move 是否被执行。

但 adapter 仍建议增加 `/api/sport/response` 订阅，用于记录请求是否被高层控制接受：

```text
/api/sport/request 证明我们发了什么；
/api/sport/response 证明机器人高层接口如何回应。
```

可以新增 probe，但不要直接接入导航：

```text
1. FreeWalk(api_id=2045) 后 Move(x=0.05,z=±0.15)
2. ClassicWalk(api_id=2049, parameter={"data":true}) 后 Move(x=0.05,z=±0.15)
3. SpeedLevel(api_id=1015, parameter={"data":1}) 后 Move(...)
```

这些只能在单独 supervised probe 中测试，不应作为默认启动序列。原因是 gait/mode API 的真实效果与机器人固件、遥控器/APP 状态、高层模式有关，不能在未验证时混入导航闭环。

---

## 10. 长期更优方案

adapter shim 是工程上最快的修法，但它的缺点是：

```text
adapter 改变了 planner 已经碰撞检查过的速度命令；
如果没有 state gate，风险较高；
即使有 state gate，local planner 仍然以 rotate-in-place 轨迹做部分判断。
```

更优雅的长期方案是在 DDDMR 内部增加 Go2 专用 heading alignment 轨迹生成器：

```text
trajectory_generators::Go2ArcAlignTrajectoryGenerator
```

它直接输出小前进量 + yaw 的弧线轨迹，并让 local planner / collision critic 对这条弧线做碰撞检查。

配置类似：

```yaml
trajectory_generators:
  ros__parameters:
    plugins:
      - differential_drive_simple
      - go2_arc_align
    go2_arc_align:
      plugin: "trajectory_generators::Go2ArcAlignTrajectoryGenerator"
      forward_x: 0.05
      rotation_speed: 0.15
      controller_frequency: 10.0
      sim_time: 0.6
```

然后把：

```cpp
LP_->computeVelocityCommand("differential_drive_rotate_shortest_angle", best_traj)
```

改成参数化：

```cpp
LP_->computeVelocityCommand(STATE_->heading_align_trajectory_generator_, best_traj)
```

live Go2 配置使用：

```yaml
heading_align_trajectory_generator: "go2_arc_align"
```

这是更规范的最终方向，但实现成本比 adapter shim 高。当前阶段建议先做 state-gated adapter shim + 探针验证。

---

## 11. 最小可执行修改清单

Codex 可以按下面顺序做：

```text
A. p2p_move_base:
   1. 发布 /dddmr_go2/p2p_decision。
   2. 每个控制周期发布当前 STATE_->getCurrentDecision()。

B. go2_sport_cmd_vel_adapter.py:
   1. 增加 decision subscriber。
   2. 增加 yaw arc shim 参数。
   3. 实现 preview/live 两种模式。
   4. 只允许 d_align_heading / d_align_goal_heading 注入 x。
   5. recovery / stale / unknown state 不注入 x；live 默认 StopMove/zero。
   6. 增加连续 yaw arc 超时保护。
   7. 日志打印 original 和 transformed Sport command。

C. go2_sport_cmd_vel_dry_run.py:
   1. 同步实现 preview 逻辑。
   2. 不发布 /api/sport/request。

D. launch/scripts:
   1. 暴露 yaw arc shim 参数。
   2. 默认关闭。
   3. live 开启需要额外确认词。

E. probe:
   1. 补测 x=0.05,z=±0.15。
   2. 如不足，补测 ±0.20 和 ±0.25。
   3. 选定 yaw 后同步更新 planner rotation_speed、adapter max_yaw、yaw_arc_min_abs_yaw。

F. RViz live:
   1. 先 preview，不动狗。
   2. 再 synthetic live shim probe，不点 RViz goal。
   3. 最后短距离 0.3~0.5m click goal。
```

---

## 12. 当前风险评级

```text
建图 / map: PASS
localization: PASS
global planner: PASS for connected local goal
Docker -> Host cmd path: PASS
/api/sport/request output: PASS
Go2 yaw sign: PASS
pure yaw-only turning: FAIL / too weak
x=0.05 yaw arc: PARTIAL PASS, only tested negative yaw at z=-0.25
blind adapter shim: NOT SAFE
state-gated yaw arc shim: RECOMMENDED NEXT
full live navigation: HOLD until shim probe passes
```

最终建议：

```text
不要继续直接用 yaw-only Move 跑 RViz live navigation。
先实现 state-gated yaw arc shim，并用精确命令探针确认 ±方向都有效。
确认后再做短距离、低速、可中断的 live click goal。
```
