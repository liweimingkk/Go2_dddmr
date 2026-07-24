# Go2 XT16 原始 P2P 多点导航

## 链路

该功能是原始 DDDMR/P2PMoveBase 路径跟踪之上的顺序任务层：

```text
固定初始位姿
  -> MCL 确认记录起点
  -> 导航点
  -> DDDMR 全局规划
  -> P2PMoveBase 路径跟踪
  -> 位置到达
  -> 最终 yaw 原地对向
  -> 连续双容差与零速度复核
  -> 停留
  -> 下一导航点
```

调度器不会生成速度。每个导航点都作为 `PToPMoveBase` Action 目标交给原始
全局规划器和 P2P 闭环控制器；只有 Action 成功，且当前位置/最终朝向连续满足
容差、`safe_cmd_vel` 连续为零后，才开始停留并执行下一点。

## 1. 录制固定起点和导航点

在仓库根目录执行：

```bash
./dddmr_navigation/scripts/run_go2_xt16_navigation_test.sh \
  --record bags/p2p_missions/route_a.json \
  --initial-pose bags/p2p_missions/go2_start.json
```

如果当前目录是 `dddmr_navigation/scripts`，可执行：

```bash
./run_go2_xt16_navigation_test.sh \
  --record bags/p2p_missions/route_a.json \
  --initial-pose bags/p2p_missions/go2_start.json
```

录制模式不会启用真实 Unitree Sport 输出。先在 RViz 使用
`3D Pose Estimate` 设置正确位置，确认定位为 `TRACKING/HEALTHY`。

键盘操作：

- `i`：保存固定初始位姿；覆盖已有文件时必须输入 `OVERWRITE`。
- `Enter`、空格或 `a`：记录当前位置，并输入该点的 `dwell_sec`。
- `r`：用当前位置替换指定点。
- `d`：删除指定点。
- `u`：撤销最后一点。
- `l`：列出导航点。
- `s`：保存。
- `q`：保存成功后退出。

导航点的 `z` 是地图地面高度。录制器默认从 MCL 的 `base_link` 高度减去
`0.32 m` 机身高度。

## 2. 无实机输出演练

```bash
./dddmr_navigation/scripts/run_go2_xt16_navigation_test.sh \
  --multi-dry-run bags/p2p_missions/route_a.json
```

程序会加载记录的固定起点并发布到 `/initial_3d_pose`。只有新的
`/mcl_pose` 确认接近该起点、定位持续 `TRACKING/HEALTHY`、全局规划器
`/weighted_ground` 和 P2P Action/开关服务全部就绪后，任务才进入
`READY`。随后必须输入：

```text
EXECUTE route_a
```

dry-run 会运行完整的规划、跟踪、最终朝向、复核和停留状态机，但不向
`/api/sport/request` 发布真实运动请求。

多点模式会关闭 `clicked2goal.py` 并在确认前锁住 P2P 目标，避免 RViz
误点覆盖任务。原有单目标 `--dry-run` 和 `--live` 行为保持不变。

## 3. 现场监督执行

```bash
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
  ./dddmr_navigation/scripts/run_go2_xt16_navigation_test.sh \
  --multi-live bags/p2p_missions/route_a.json
```

如果当前目录是 `dddmr_navigation/scripts`：

```bash
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
  ./run_go2_xt16_navigation_test.sh \
  --multi-live bags/p2p_missions/route_a.json
```

实机模式沿用原脚本的 DDS、里程计时间同步、地图、局部感知、Go2 状态和
Sport 输出安全检查。任务达到 READY 后仍需输入
`EXECUTE <mission_id>`。定位失效、P2P 失败、单点超时、停留期间移动或总监控
超时都会取消任务、禁用 P2P 目标并执行现有停车清理。

## 文件格式与状态

任务格式与 SCAN 多点任务的版本 1 JSON 兼容，因此已有 `route_a.json` 和其
引用的初始位姿文件可直接用于原始 P2P 多点演练；两类任务都要求文件位于
配置的 `DDDMR_BAGS_DIR` 下。

```json
{
  "version": 1,
  "mission_id": "route_a",
  "initial_pose_file": "go2_start.json",
  "waypoints": [
    {
      "id": "wp_001",
      "x": 1.0,
      "y": 2.0,
      "z": 0.0,
      "yaw": 1.5708,
      "dwell_sec": 2.0
    }
  ]
}
```

- `/p2p_multi_point/state`：任务状态。
- `/p2p_multi_point/current_waypoint`：当前点的零基序号。
- `/p2p_multi_point/arm`：READY 后解锁并提交第一点。
- `/p2p_multi_point/cancel`：取消 Action 并锁住 P2P 目标。

任务状态依次包含 `LOCALIZING`、`WAITING_*`、`READY`、`ARMING`、
`NAVIGATING`、`ALIGNING`、`SETTLING`、`DWELLING` 和
`COMPLETE/FAILED`。
