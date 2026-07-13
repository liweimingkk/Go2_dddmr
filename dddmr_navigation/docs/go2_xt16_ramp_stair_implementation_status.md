# Go2 XT16 坡道与楼梯导航实施状态

状态日期：2026-07-11

本文记录 `go2_xt16_ramp_stair_test_stage_plan.md` 的软件落地范围。所有地形运动
能力在发布配置中仍默认关闭；本文不表示坡道或楼梯已经通过 dry-run、bag、现场
静止或实机运动验收。

## 已落地的软件链路

- 地图服务器分离普通 map/ground 与高分辨率 terrain ROI，并对实际加载的全部
  PCD 工件计算确定性 SHA-256；配置哈希只用于核验，不能覆盖实测身份。
- map/ground 按两侧消息代际成对提交，更新期间清除旧地形快照；空点云、混代
  快照和哈希不一致均 fail-closed。
- 感知层提供不可变 `TerrainSnapshot`、平面法向、坡度、粗糙度、支撑、surface、
  人工楼梯走廊、踏步、平台和在线确认。
- direct A*、pre-graph 和静态建图共用地形边策略；检查有符号上下坡、横坡、
  支撑、surface、通用台阶、落差、相邻踏步、走廊、方向、地图与快照版本。
- 全局路径、DWA 拼接、start/goal、raw goal、路径尾姿态和规划失败均按
  fail-closed 处理；`GetPlan` action 返回结构化状态、地图代际、快照版本和边拒绝
  统计。
- 地形局部轨迹按未来支撑更新 z/法向；`TerrainSupportModel` 拒绝未知、落差、
  无在线支撑、跨 surface、走廊外和跳级轨迹。
- 预期楼梯立板只可在人工走廊、同一楼梯、相邻踏步和腿部包络中语义放行；
  机身、侧墙、扶手、异物和已确认动态障碍仍为致命障碍。
- `StairTraversalSupervisor` 实现入口预检、接近、平台对准、committed、出口平台
  验证和故障锁存。楼梯中禁止横移、倒退、普通 recovery、自由重规划和原地旋转。
- Go2 继续使用正常导航步态。软件不发送 gait API；只允许既有 Move(1008) 和
  StopMove(1003)，并监控 mode/gait、请求 API、地形、姿态、速度和进度。
- 到达条件分离 XY、Z、yaw，并可要求目标 surface、楼梯踏步或指定平台匹配。
- 站点 profile 和实际运行 YAML 形成离线闭环；非 `safe-disabled` 目标缺少
  `--runtime-config` 会直接失败。

## 当前测试入口

发布模板和 Go2 运行 YAML 均为 `safe-disabled`。填写真实地图、ROI、坡道/楼梯
测量值和能力证据后，先执行纯离线校验：

```bash
python3 dddmr_navigation/scripts/validate_go2_xt16_terrain_site_profile.py \
  --profile /path/to/measured-site-profile.yaml \
  --runtime-config \
    dddmr_navigation/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml \
  --target ramp
```

楼梯先使用 `stair-up`；只有上楼、负障碍、出口平台和停止故障证据通过后，才可
校验并申请 `stair-down` 测试。配置校验通过只表示软件合同完整，不会启动 ROS 或
授权运动。

## 必须由后续测试补齐

- 真实站点 map SHA-256、terrain ROI、坡角、横坡、表面粗糙度和楼梯几何。
- 正常步态可执行最低速度、各方向安全上限、yaw 修正、StopMove 和停止距离。
- 合成 ROS、真实 bag、现场静止分类、TF/姿态、延迟和 CPU 预算。
- 坡道按上行、下行和横坡分别验收；楼梯按单级、短梯、完整楼梯先上后下验收。
- committed 状态停止稳定性、动态障碍、gait 不变和仅 Move/StopMove 的证据。

## 已知限制与安全结论

1. 在线层目前对前方缺少 fresh ground 的区域保守地标为不可用支撑，因此不会把
   UNKNOWN 当可通行；但基于口部雷达射线的显式 `OBSERVED_FREE/UNKNOWN/DROP`
   负障碍分类尚未完成现场验证。下坡和下楼不能仅凭当前单元测试开放。
2. 旧 XYZI 障碍流没有可靠目标跟踪标签。已确认动态障碍会拒绝，但与预期立板
   几何完全重合的动态物体仍需要后续 tracker 才能可靠区分。
3. map 与 ground 是两个独立 topic，没有共同 source epoch。当前保证两侧均有新
   消息且进程内原子交换；要彻底抵抗跨 topic 多代乱序，需要发布端共同 epoch/hash
   或合并消息。
4. 楼梯静态图会随有效在线快照更新立板语义，启用前必须用 bag 测量重建开销。
5. 发布配置中的坡度、台阶、楼梯速度和方向能力保持零或关闭。禁止为了让校验
   通过而填写未由现场证据支持的非零数值。

因此，当前结果是“软件与离线测试入口已具备、默认安全关闭”。坡道现场测试就绪
仍依赖 profile、bag 和现场静止门；楼梯上行还需相同证据，下行额外依赖负障碍与
停止能力验证。
