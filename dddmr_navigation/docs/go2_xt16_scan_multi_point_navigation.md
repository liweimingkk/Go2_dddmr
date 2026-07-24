# Go2 XT16 SCAN 多点导航

## 作用范围

该功能是 SCAN 路径跟踪之上的任务调度层，不会把离散导航点直接转换为速度：

```text
任务导航点
  -> DDDMR 全局规划
  -> SCAN 参考路径与局部避障
  -> 位置到达
  -> 最终 yaw 原地对向
  -> 停留
  -> 下一导航点
```

任务只按顺序执行一次。任何定位丢失、规划超时、传感器/规划器安全门异常或
单点执行超时都会撤销任务命令许可并停车，不跳过失败点。

## 1. 录制固定初始位姿和导航点

在仓库根目录或 `dddmr_navigation` 目录执行：

```bash
./dddmr_navigation/scripts/run_go2_xt16_scan_navigation.sh \
  --record bags/scan_missions/route_a.json \
  --initial-pose bags/scan_missions/go2_start.json
```

如果当前目录是 `dddmr_navigation`，也可以使用：

```bash
./scripts/run_go2_xt16_scan_navigation.sh \
  --record bags/scan_missions/route_a.json \
  --initial-pose bags/scan_missions/go2_start.json
```

录制运行不会启用真实 Unitree Sport 输出。首先在 RViz 使用
`3D Pose Estimate` 校准固定起点，确认点云与地图重合且定位状态为
`TRACKING/HEALTHY`。

键盘操作：

- `i`：保存固定初始位姿；覆盖已有文件必须输入 `OVERWRITE`。
- `Enter`、空格或 `a`：记录当前位置，并强制输入该点 `dwell_sec`。
- `r`：用当前位置替换指定导航点并重新输入 `dwell_sec`。
- `d`：删除指定点。
- `u`：撤销最后一个点。
- `l`：列出所有点。
- `s`：原子保存任务。
- `q`：保存成功后退出。

任务中的 `z` 是 `/goal_pose_3d` 使用的地图地面高度。录制器会从当前
`base_link` 高度中减去配置的 `0.32 m` 机身高度。

## 2. 无实机输出演练

```bash
./dddmr_navigation/scripts/run_go2_xt16_scan_navigation.sh \
  --multi-dry-run bags/scan_missions/route_a.json
```

程序会把固定初始位姿发布到 RViz `3D Pose Estimate` 使用的同一
`/initial_3d_pose` 话题。收到地图地面后会先等待 MCL 完成地面树初始化；
如果没有在新的 `/mcl_pose` 中确认到接近记录值的位置和朝向，则限频重发，
不会把自动全局定位得到的其他位置误判为 READY。确认固定起点且定位重新达到
`TRACKING/HEALTHY` 后，还会等待全局规划器发布 `/weighted_ground`，并要求
SCAN 机身位姿连续稳定。执行前若输入短暂中断，状态会退回
`WAITING_INPUTS`，恢复后重新进入 READY，不会直接变成永久 FAILED。
监督脚本在提交服务请求前会再次等待实时 READY。随后必须输入：

```text
EXECUTE route_a
```

dry-run 会执行完整的逐点规划、路径跟踪状态机和停留计时，但不会向
`/api/sport/request` 发布真实运动请求。

多点模式会把任务目标接入独立的
`/scan_multi_point/goal_pose_3d`，并断开 RViz 手动目标和
`/clicked_point` 输入，避免执行期间误点目标覆盖当前任务路线。普通单目标
`--dry-run/--live` 模式仍使用原有目标话题。

## 3. 现场监督执行

```bash
./dddmr_navigation/scripts/run_go2_xt16_scan_navigation.sh \
  --multi-live bags/scan_missions/route_a.json
```

实机模式继续要求现有 Sport Move/StopMove 探针证据、现场监督、清空运行区域
和物理停止手段。定位 READY 后还会要求任务专用的
`EXECUTE <mission_id>` 确认。在收到该确认之前，多点任务门保持关闭。

## 文件格式

固定初始位姿是版本化 JSON，包含 `map` 位姿、单位四元数和 36 项协方差。
任务文件引用该文件，并保存有序导航点：

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

每点都必须显式提供 `dwell_sec`，允许值为 `0..3600` 秒。任务及初始位姿
必须位于 `bags/` 下。功能按约定不绑定地图指纹，因此切换地图后必须由操作员
确认坐标系仍与录制时一致。

## 状态和停止

- `/scan_multi_point/state`：任务状态。
- `/scan_multi_point/current_waypoint`：当前点的零基序号。
- `/scan_multi_point/enabled`：命令门许可。
- `/scan_multi_point/arm`：READY 后启用任务。
- `/scan_multi_point/cancel`：撤销任务并停车。

`Ctrl-C`、`FAILED` 和 `COMPLETE` 都会关闭任务命令门。实机监督脚本还会执行
现有 StopMove 清理流程。
