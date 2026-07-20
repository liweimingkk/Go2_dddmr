# AGENTS.md

This repository is a clean DDDMR Navigation workspace for a Unitree Go2 EDU with a Hesai XT16 LiDAR. Treat this file as the first recovery and operating context for future Codex sessions.

## Operating Boundaries

- Do not publish Go2 motion commands unless the user explicitly approves a supervised motion task.
- Do not publish `/cmd_vel`, `/api/sport/request`, `/lowcmd`, or start autonomous navigation closed loop from this workspace by default.
- Do not modify Go2 network settings, system services, startup files, firmware, calibration, safety parameters, or `/home/unitree/xt16_ws` unless explicitly approved.
- Start every live XT16 task with read-only checks: point-cloud samples, TF availability, ROS domain/interface, and Docker/RMW state.
- For mapping, prove `/lidar_points` first, then launch mapping, then save/check map artifacts. Do not jump straight to navigation.

## Upstream XT16 Issue Record

Use these upstream author replies as the primary XT16 guidance:

- Issue 58: <https://github.com/dfl-rlab/dddmr_navigation/issues/58>
- Issue 61: <https://github.com/dfl-rlab/dddmr_navigation/issues/61>

General author-first rule:

- When this Go2 XT16 line hits a symptom that resembles an upstream issue, start from the maintainer's advice before inventing a new workaround.
- Prefer the upstream diagnosis order: verify sensor samples and TF, verify odometry and timestamps, inspect ground/map/perception topics in RViz, then tune config or patch code.
- Treat the local issue digest as an index, not as a substitute for the current repo: `../dddmr_navigation_issues_qa_2026-07-01.md`.
- If an upstream issue mentions an issue branch or old commit that no longer exists, reuse the diagnostic idea but do not depend on the unavailable branch.
- If a proposed fix would publish motion commands, modify Go2 services/network/startup, or change the XT16 driver workspace, stop and ask for explicit approval.

Author issue playbook for Go2 XT16:

- XT16 support and crashes, issues 58/7/42: XT16 is treated as supported. Start from C16-style parameters adapted to XT16. If `lego_loam_bor` exits with `-8`, `-11`, allocator errors, `double free`, or point-count assertions, first check Docker/PCL/GTSAM build compatibility and exact scan parameters before changing algorithms.
- Blank ground under robot, issue 61: enable or confirm `patch_first_ring_to_baselink`, keep the robot stationary, tune ground FOV until the first frame is not hollow, then move slowly. Do not tune while driving first.
- Robot spins in place or does not move to goal, issues 73/30/33/44/53: assume trajectories are being rejected until proven otherwise. Check terminal rejection logs, cuboid points, `segmented_cloud_pure`, `perception_3d_ros/dGraph`, global marking, and whether ground points are being treated as obstacles.
- `/cmd_vel` is zero, issues 30/63: confirm a goal was sent and a global path exists. Then verify local planner odom topic, fresh TF, cuboid, and obstacle inputs. The stack may rotate-align before forward motion; this matters for non-differential bases.
- No global path, issues 3/8/25/33/37: check whether start and goal are close to valid ground, whether ground is connected/dense enough, and whether obstacles/cuboid block the path. Temporarily remove the lidar plugin from local/global perception only as a no-motion isolation test.
- Existing Fast-LIO/LIO-SAM/PCD map integration, issues 31/37/39/52: do not assume vanilla LIO-SAM can replace DDDMR LeGO-LOAM directly. If using external localization, provide `map->base_link` TF plus `mapcloud/mapground`; remove or bypass `mcl_3dl/mcl_feature` only deliberately. For no wheel odom, use FAST-LIO or another lidar odom source as `/odom`.
- Mapping save and map reuse, issues 36/50: save LeGO-LOAM map artifacts first, then point navigation config at that directory. For a single full map, use `0_feature.pcd` and `0_ground.pcd`, enlarge `sub_map_search_radius`, and set warmup distance very large.
- Duplicate or stale nodes, issues 32/44: before launching, check for leftover navigation, MCL, TF, or planner nodes. Duplicate `mcl_3dl` or multiple `odom->base_link` publishers can break localization and TF.
- 3D odom or IMU drift, issue 72: IMU must follow right-hand coordinates, X forward and Z up. Feed 3D odom into `mcl_3dl`, not `mcl_feature`; record minimal `/odom`, IMU, and lidar bags when debugging.
- Stairs, ramps, and traversability, issues 13/59/71: global planning can use discontinuous ground if the ground graph connects, but local perception may mark stairs or ramp points as obstacles. Check whether `segmented_cloud_pure` contains traversable ground before changing planner behavior.
- Operator/person artifacts in XT16 maps, issue 61: because XT16 is 16-line, follow the author's point-cloud editing recommendation instead of defaulting to deep-learning removal.
- Wireless operation, issue 70: for robot-side compute, prefer Ethernet to XT16 and Wi-Fi/hotspot or remote desktop to the laptop. Avoid streaming heavy point clouds over weak wireless links when RViz can run on the robot-side computer.

Issue 58 conclusion:

- Hesai XT16 is not treated as unsupported. The maintainer says most LiDARs are supported and recommends using the Leishen C16 config as the template.
- Start from `src/dddmr_lego_loam/lego_loam_bor/config/loam_c16_config.yaml`.
- Change the LiDAR spec fields for XT16, especially:
  - `num_vertical_scans`
  - `num_horizontal_scans`
  - `vertical_angle_bottom`
  - `vertical_angle_top`
  - `scan_period`
- If `lego_loam_bor` crashes with errors such as `exit code -8`, `double free or corruption`, or allocator/thread-like failures, check the Docker and binary dependency environment before tuning algorithms.
- The maintainer specifically attributes a common crash class to PCL/GTSAM cross-compile argument mismatch and recommends using the official Docker image.
- If the crash persists, upstream asked for a short bag, about 2 minutes, containing point cloud and odometry topics, plus logs and config.
- The issue author later reported successful real Go2 deployment with Docker x86.
- The `issue58` branch mentioned in the issue is not currently available on GitHub; do not depend on it.

Issue 61 conclusion:

- For blank ground under a horizontally mounted top XT16, first check `patch_first_ring_to_baselink: true`.
- If that is already true, launch SLAM while the robot is stationary and tune ground FOV parameters before moving.
- The first frame's ground must not be hollow before starting motion.
- After the stationary first-frame ground looks correct, move the robot slowly and check whether ground construction remains correct.
- For people and operator artifacts in the map: because XT16 is only 16 lines, upstream says the deep-learning removal approach is not effective. Use point-cloud editing tools to delete unwanted map points.
- Upstream released the point-cloud deleting tool under `src/dddmr_perception_3d`; see `src/dddmr_perception_3d/README.md` and `ros2 launch perception_3d pc_delete_utils.launch`.

## Local XT16 Contract

The live Go2 XT16 stream previously verified in this project is:

- topic: `/lidar_points`
- type: `sensor_msgs/msg/PointCloud2`
- frame: `hesai_lidar`
- width: `32000`
- height: `1`
- point step: `26`
- fields: `x`, `y`, `z`, `intensity`, `ring`, `timestamp`
- `ring`: `UINT16`, range `0..15`
- points per ring: `2000`
- `timestamp`: `FLOAT64`, per-frame span around `0.1s`
- expected rate: about `10 Hz`

This is the active navigation contract verified on 2026-07-10. The prior
`64000`-point (`4000` points/ring) mode delivered only about `5 Hz` on this
deployment. Do not configure a 4000-column range image for the active
2000-points/ring stream; the empty alternating columns reduce segmentation
continuity and downstream update rate.

Do not accept publisher presence alone as readiness. Require actual samples and field decoding.

## Clean Docker Path

Use the upstream Docker environment rather than host-built mixed PCL/GTSAM binaries.

Treat Docker as the authoritative build and runtime environment for the Go2 XT16
navigation stack:

- After changing navigation source or configuration, run
  `./scripts/dddmr_docker_go2_xt16.sh build-navigation`.
- Validate installed artifacts under `.docker_go2_xt16_install/`; the Docker
  navigation entry points source this install base.
- Do not use a host-side `colcon build`, `build/`, or `install/` result as the
  navigation acceptance result. A host build failure does not prove that the
  Docker image lacks dependencies; reproduce it through the Docker wrapper
  before diagnosing a dependency problem.

This clean workspace adds only a thin Go2 layer:

- upstream base image: `dddmr:x64`
- Go2 wrapper image: `dddmr_go2_xt16:x64`
- wrapper script: `scripts/dddmr_docker_go2_xt16.sh`
- Docker quickstart: `docs/go2_xt16_docker_quickstart.md`

The Go2 wrapper image only adds CycloneDDS RMW support needed for this Go2 ROS 2 setup. Do not replace the upstream Docker build unless there is a specific dependency reason.

Default preflight:

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

Build LeGO-LOAM inside Docker:

```bash
./scripts/dddmr_docker_go2_xt16.sh build-lego
```

Run no-motion mapping only after preflight passes:

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=false ./scripts/dddmr_docker_go2_xt16.sh mapping
```

If the live graph lacks `base_link -> go2_imu -> hesai_lidar`, a local static TF may be enabled for perception-only mapping smoke tests:

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh mapping
```

This static TF path is not motion control.

## XT16 Mapping Parameters

The local XT16 LeGO-LOAM config is:

```text
src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_live.yaml
```

It carries the issue 58/61 parameter lessons:

- `num_vertical_scans: 16`
- `num_horizontal_scans: 2000`
- `vertical_angle_bottom: -15.0`
- `vertical_angle_top: 15.0`
- `scan_period: 0.1`
- `cloud_qos_reliability: "reliable"`
- `use_ring_channel: true`
- `reverse_ring_order: true`
- `use_point_timestamp: true`
- `patch_first_ring_to_baselink: true`
- ground FOV defaults: bottom `-0.2617994`, top `-0.087266`
- `enable_loop_closure: false` for the first live smoke tests

Treat these as starting parameters, not final calibration. If ground is hollow, tune ground FOV while stationary before any movement.

## Current Local Observation

On 2026-06-29, the clean Docker preflight passed on live `/lidar_points` with 3 valid XT16 samples at about 10 Hz. Starting `lego_loam_go2_xt16_live.launch` without static TF showed missing footprint-to-sensor TF. Restarting with `publish_static_tf:=true` removed the TF error but clean upstream `lego_loam` exited with code `-11` shortly after receiving live XT16 data.

Interpret that failure in the issue 58 frame:

- The sensor contract is valid.
- The next debugging target is `lego_loam_bor` runtime compatibility or the minimal XT16 code adaptations, not the XT16 driver.
- Before sending anything upstream, capture:
  - exact Docker image tag and build path
  - `loam_go2_xt16_live.yaml`
  - launch command
  - crash log
  - a short bag with `/lidar_points` and odometry if available

## Ground Blank And People Artifacts Workflow

For issue 61 style outdoor maps:

1. Verify `/lidar_points` samples.
2. Start SLAM with the robot stationary.
3. Confirm `patch_first_ring_to_baselink: true`.
4. Tune ground FOV until the first frame ground is not hollow.
5. Move slowly only after the stationary first frame is correct.
6. Save map artifacts.
7. Use `pc_delete_utils.launch` to remove people/operator artifacts from PCD maps.

Do not propose deep-learning people removal for XT16-only maps unless there is a new sensor or dataset that changes the upstream 16-line limitation.

## Recovery Checklist

When resuming work:

1. Read this file first.
2. Check `git status --short --branch`.
3. Confirm there is no stale `dddmr_go2_xt16_mapping` container.
4. Run Docker preflight before any mapping launch.
5. Keep live runs timeout-limited unless the user asks to leave them running.
6. Record commands, exit codes, and key logs in the response or a run note.
