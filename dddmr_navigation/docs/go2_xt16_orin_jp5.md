# Go2 Orin JetPack 5 Deployment

This is the onboard, no-TensorRT port of the complete DDDMR source workspace.
It runs independently from every pre-existing robot workspace.

## Fixed platform

- NVIDIA Orin NX, JetPack 5.1.1 / L4T R35.3.1
- Ubuntu 20.04, ROS 2 Foxy, Python 3.8
- CycloneDDS 0.10.2 with the Foxy `rmw_cyclonedds` 0.7.11 overlay
- Unitree ROS 2 v0.3.0 message packages only
- PCL 1.15.0 and GTSAM 4.2a9
- TensorRT disabled with `-DTRT_ENABLED=OFF`

The image deliberately uses the multi-architecture `ubuntu:20.04` parent.
Mapping and navigation are CPU-only, so the 5+ GB JetPack development image,
CUDA, TensorRT, PyTorch, and semantic model assets are not runtime
dependencies. The container still runs through the Jetson NVIDIA runtime for
onboard graphics compatibility.

The deployment root is:

```text
/home/unitree/go2_dddmr_full/
├── bags/
└── dddmr_navigation/
```

Do not reuse or overwrite any other DDDMR directory on the robot. Do not alter
`/home/unitree/xt16_ws`, the Hesai driver, network configuration, system
services, startup files, firmware, or calibration.

## Offline build network

The robot has no internet route. On the development computer, start the
temporary allow-listed build proxy:

```bash
python3 scripts/serve_orin_build_proxy.py \
  --bind 192.168.123.222 \
  --allow-client 192.168.123.18
```

Pass `HTTP_PROXY=http://192.168.123.222:3128` and the matching HTTPS proxy only
to `build-go2-image`. The proxy rejects every client except the robot and
rejects target ports other than 80 and 443. Stop it immediately after the
image is built; no robot route or DNS change is required.

## Build

The robot user needs `sudo` for Docker. Setting `DDDMR_DOCKER_USE_SUDO=1`
re-executes the complete wrapper through `sudo`, including nested read-only
time-synchronization helpers.

```bash
cd /home/unitree/go2_dddmr_full/dddmr_navigation

DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
DDDMR_BUILD_JOBS=2 \
HTTP_PROXY=http://192.168.123.222:3128 \
HTTPS_PROXY=http://192.168.123.222:3128 \
./scripts/dddmr_docker_go2_xt16.sh build-go2-image

DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
GO2_NET_IFACE=eth0 \
GO2_DDS_EXTRA_IFACES=wlan0 \
./scripts/dddmr_docker_go2_xt16.sh build-all
```

The final image is `dddmr_go2_xt16:orin-jp5.1.1`. ARM build, install, and log
trees are isolated in:

```text
.docker_go2_xt16_orin_build/
.docker_go2_xt16_orin_install/
.docker_go2_xt16_orin_log/
```

The stock JetPack 5 host caps UDP receive buffers below the workspace's strict
16 MiB x64 minimum. The Orin wrapper therefore sets
`GO2_DDS_RCVBUF_MIN=default` inside its containers while still asking
CycloneDDS for a 16 MiB maximum. This does not change host sysctls or robot
network settings. The x64 default remains a fail-closed 16 MiB minimum, and
the live point-cloud preflight below remains mandatory on Orin.

## Read-only acceptance

First prove that actual XT16 samples cross the container boundary:

```bash
DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
GO2_NET_IFACE=eth0 \
GO2_DDS_EXTRA_IFACES=wlan0 \
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

When the Orin provides the `Go2-Orin` operator hotspot, keep `eth0` as the
primary Go2/XT16 interface and explicitly add `wlan0` for the remote RViz
computer. CycloneDDS otherwise selects only one interface from a multi-homed
host. The laptop should bind CycloneDDS only to its Wi-Fi interface and use the
same `ROS_DOMAIN_ID`.

When an extra interface is present, `scripts/setup_go2_dds_env.sh`
automatically detects the IPv4 address of `GO2_NET_IFACE` and pins the
`rt/utlidar/*` and `rt/uslam/*` DDS readers to that address. This prevents the
Go2's writers from selecting the operator-hotspot address as the return path,
while all other mapping topics remain available on both `eth0` and `wlan0`.
Set `GO2_DDS_PRIMARY_ADDRESS` explicitly only when the primary interface address
cannot be detected.

On the laptop, start the dedicated navigation RViz configuration with:

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/run_go2_xt16_laptop_navigation_rviz.sh
```

The launcher detects the interface routed to `192.168.50.1`, binds CycloneDDS
only to that laptop interface, and uses
`rviz/go2_xt16_navigation_laptop.rviz`. The laptop configuration replaces the
optional DDDMR 3D pose tools with standard RViz 2D tools while preserving
`initial_3d_pose` and `goal_pose_3d`. Do not click `2D Goal Pose` until the
robot-side navigation readiness checks have passed; a goal can move the Go2
when the supervised live adapter is active.

For a read-only live view while the Orin is mapping, expose `wlan0` to the
mapping container in addition to the primary XT16/Go2 `eth0` interface:

```bash
cd ~/go2_dddmr_full/dddmr_navigation
sudo env \
  DDDMR_PLATFORM=orin-jp5 \
  DDDMR_DOCKER_NAME=go2_xt16_mouth_mapping_field \
  GO2_NET_IFACE=eth0 \
  GO2_DDS_EXTRA_IFACES=wlan0 \
  RVIZ=false \
  MAP_RVIZ=false \
  STOP_AFTER_SAVE=false \
  ./scripts/run_go2_xt16_mouth_mapping_save_to_nav.sh --start-only
```

Then start the dedicated mapping viewer on the laptop:

```bash
cd /home/kkkkkkq/new2_success/new22/new2/dddmr_navigation
./scripts/run_go2_xt16_laptop_mapping_rviz.sh
```

The laptop launcher waits for real `/lego_loam_map` and
`/lego_loam_ground` publishers, receives one sample from each, and then opens
RViz. The view shows the accumulated map, accumulated ground, mouth-ground
contribution, key poses, and pose-graph edges. It contains no goal, initial
pose, point-publish, navigation, or Unitree command tool. Raw
`/lidar_points` is intentionally omitted because forwarding the full 10 Hz
XT16 stream over operator Wi-Fi is unnecessary for normal map monitoring.
Use `--check-only` to validate DDS discovery and samples without opening RViz.

Then run tests:

```bash
DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
./scripts/dddmr_docker_go2_xt16.sh test-all
```

A mapping smoke test remains stationary and timeout-limited:

```bash
DDDMR_PLATFORM=orin-jp5 \
DDDMR_DOCKER_USE_SUDO=1 \
GO2_NET_IFACE=eth0 \
GO2_DDS_EXTRA_IFACES=wlan0 \
RUN_SECONDS=20 \
RVIZ=false \
PUBLISH_STATIC_TF=true \
./scripts/dddmr_docker_go2_xt16.sh mapping
```

Do not run a live navigation adapter or publish `/cmd_vel`,
`/api/sport/request`, or `/lowcmd` without a separate supervised-motion
approval.
