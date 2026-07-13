# Go2 XT16 地形站点配置离线校验

`go2_xt16_ramp_stair_site_profile.template.yaml` 是坡道和人工楼梯走廊的站点
配置模板。它默认关闭所有地形运动能力，坐标、地图哈希和能力证据均为空，不能
直接作为 ROS 参数或实机配置使用。

先复制模板并填写真实测量值，再做纯离线校验：

```bash
cd /home/kkkkkkq/new2_success/new2
python3 dddmr_navigation/scripts/validate_go2_xt16_terrain_site_profile.py \
  --profile /path/to/measured-site-profile.yaml \
  --runtime-config dddmr_navigation/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml \
  --target ramp
```

可选目标为 `safe-disabled`、`ramp`、`stair-up` 和 `stair-down`。默认
`safe-disabled` 只证明所有能力开关保持关闭。其他目标仅证明配置合同完整，不会
启动 ROS、发布命令或代替 dry-run、bag 回放和人工受监督测试。
`ramp`、`stair-up` 和 `stair-down` 必须提供 `--runtime-config`；缺少实际
运行参数文件时直接失败。`safe-disabled` 可以只检查模板；若同时提供运行参数，
还会确认实际 Go2 配置中没有暗开的地形能力：

```bash
python3 dddmr_navigation/scripts/validate_go2_xt16_terrain_site_profile.py \
  --runtime-config dddmr_navigation/src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml \
  --target safe-disabled
```

成功输出中的 `CONFIG_CONTRACT_READY=TRUE` 只针对所选配置目标；脚本固定输出
`LIVE_MOTION_READY=NOT_ASSESSED`，避免把离线校验通过误解成实机运动许可。

楼梯目标会额外要求：

- 地图、ROI、TerrainLayer 与每条走廊使用同一个真实地图 SHA256；
- corridor、上下 landing 多边形和中心都位于 ROI 内；
- `step_count` 明确定义为踢面（riser）数量；最高踏面与上平台同高；
- 上下平台总高差必须满足
  `step_count * riser_height_m`，最大误差为
  `max(0.02 m, 10% * riser_height_m)`；站点可以设置更严格但不能更宽的误差；
- 第一踢面中心必须位于走廊内、处于两平台之间，其 z 为下平台加半个踢面高度；
- 上行轴、级数、踢面高度、踏面深度和所有预期踢面中心互相一致；
- 每个开放方向都有正常导航步态的人工证据；
- 正常步态在测试开始时锁存，状态受到监控，Sport 请求受到审计；
- 禁止步态变更请求、楼梯原地旋转和 committed 状态重规划；
- 地形层、图边策略、局部支撑、楼梯监督器、命令门和 gait monitor 的运行时
  开关一致。

非禁用目标还会把站点 profile 与 `go2_xt16_navigation.yaml` 逐项比对，包括：

- map SHA256、高分辨率 ROI 和 `navigation_ground`；
- 本地/全局 TerrainLayer、人工走廊和完整楼梯几何；
- global planner 的坡度、粗糙度、法向变化、支撑、置信度和方向限制；
- terrain trajectory generator、P2P 主生成器、terrain support critic；
- 三维目标 surface 匹配和楼梯 supervisor；
- 只允许预期踢面的 collision passthrough 语义；
- 本地/全局 LiDAR 的预期 riser marking、地图哈希和高度范围。

命令门参数来自 launch 参数而不是该 YAML，因此这里仍只校验站点合同中的
`terrain_command_gate_enabled`；它是否按同一参数启动必须在后续 dry-run 中
确认。正常步态 monitor 同样必须通过运行时 topic 和 Sport 请求审计确认。

模板中的 `0.0` 坡度、粗糙度和通用台阶限制表示“尚未开放”，不是无限制。
连续坡面拟合残差是模型/离散误差容限，不会放宽通用离散台阶能力；坡道和楼梯
测试阶段 `generic.max_step_up_m/down_m` 仍必须保持 `0.0`。
只有在现场能力证据存在、规划限制不超过已验证包络时，目标校验才会通过。
