# Ubuntu 22.04 + ROS 2 Humble 新笔记本运行指南

本文档用于把当前 Go2 EDU + Hesai XT16 的 DDDMR Navigation 工程迁移到新的
Ubuntu 22.04 笔记本，并在 ROS 2 Humble 环境下复现建图、导航 dry-run 和后续
监督运动测试前的检查流程。

本文下面的命令默认沿用旧机器路径；如果新笔记本用户名或目录不同，先在当前
terminal 设置这些变量，后续命令直接复用：

```bash
export DDDMR_WS="${DDDMR_WS:-/home/lin/new2/dddmr_navigation}"
export DDDMR_BAGS_DIR="${DDDMR_BAGS_DIR:-/home/lin/dddmr_bags}"
export GO2_SETUP="${GO2_SETUP:-/home/lin/go2_workspace/unitree_ros2/setup.sh}"
export GO2_CONTROLLER_IP="${GO2_CONTROLLER_IP:-192.168.123.161}"
export GO2_SSH_IP="${GO2_SSH_IP:-192.168.123.18}"
export GO2_DDS_IP="${GO2_DDS_IP:-192.168.123.18}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
```

当前旧机器源码仓库根目录示例：

```text
/home/lin/new2/dddmr_navigation
```

外层 `/home/lin/new2` 还有 bags、offline maps、review packets 等运行产物，不要把
外层目录当成干净的单一源码仓库。迁移时必须同时处理源码仓库和源码外的地图/bag 目录。

如果新笔记本已经把仓库放在别处，例如当前 shell 已经位于仓库根目录，可以用：

```bash
export DDDMR_WS="$(pwd)"
```

不要把 `DDDMR_BAGS_DIR` 设成 `/root/dddmr_bags`；这是 Docker 容器内的挂载路径。
宿主机路径由 `DDDMR_BAGS_DIR` 控制，容器内统一映射为 `/root/dddmr_bags`。

## 1. 默认运行原则

- 推荐路线是 Docker 构建和运行 DDDMR 主栈，宿主机只负责 Docker、ROS/DDS 环境、RViz 显示和少量 Go2 host-side 检查。
- Docker 在笔记本上运行，不在 Go2 本体上运行。
- 默认只做只读预检、建图、RViz 和 navigation dry-run。
- 不默认发布真实 `/cmd_vel`、`/api/sport/request`、`/lowcmd`。
- 不默认修改 Go2 网络、系统服务、启动项、固件、标定或 `/home/unitree/xt16_ws`。
- 真实运动测试必须单独确认：现场监督、空旷场地、物理接管/急停可用。

当前仓库的可靠入口是：

```bash
./scripts/dddmr_docker_go2_xt16.sh
```

不要优先用宿主机直接 `colcon build` 编完整 DDDMR 主栈。这个工程依赖 PCL 1.15.0、GTSAM 4.2a9、GCC 13 和特定 Humble 组合，Dockerfile 已经固定这套环境。

## 2. 新笔记本需要安装什么

目标系统：

```text
Ubuntu 22.04 LTS amd64
ROS 2 Humble
Docker Engine
```

宿主机基础工具：

```bash
sudo apt update
sudo apt install -y \
  git curl wget ca-certificates gnupg lsb-release \
  build-essential cmake pkg-config \
  python3 python3-pip python3-venv \
  python3-colcon-common-extensions \
  rsync unzip x11-xserver-utils \
  iproute2 iputils-ping net-tools htop \
  ripgrep
```

ROS 2 Humble：

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-rmw-cyclonedds-cpp ros-dev-tools
```

验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
ros2 doctor --report
```

Docker Engine 建议按 Docker 官方 apt 仓库安装。安装完成后至少验证：

```bash
sudo systemctl status docker
sudo docker run hello-world
```

如果希望当前用户直接运行 Docker：

```bash
sudo usermod -aG docker "$USER"
newgrp docker
docker run hello-world
```

如果使用独显或 Wayland 会话，先以普通 X11 桌面测试 RViz。Docker 内 RViz 需要访问 X11：

```bash
xhost +local:docker
```

## 3. Go2 网络和 ROS/DDS 前提

当前记录的 Go2 网络约定：

```text
Go2 controller / data-plane IP: 192.168.123.161
Go2 SSH / DDS route target:     192.168.123.18
Host wired IP:                  192.168.123.222/24
RMW:                            rmw_cyclonedds_cpp
```

新笔记本接 Go2 网线后先做只读检查：

```bash
ip -br addr
ip route
ping -c 2 -W 1 "${GO2_CONTROLLER_IP}"
ping -c 2 -W 1 "${GO2_SSH_IP}"
ip route get "${GO2_DDS_IP}"
```

期望路由形态：

```text
192.168.123.0/24 dev <go2-wired-nic> proto kernel scope link src 192.168.123.222
```

如果网卡名不是旧机器上的 `enp46s0` 或 `enp5s0`，不要硬改脚本；先用环境变量覆盖。
把下面的 `enp5s0` 换成 `ip -br addr` 里连接 Go2 的有线网卡名：

```bash
export GO2_NET_IFACE=enp5s0
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

本仓库的 Docker wrapper 会通过 `scripts/setup_go2_dds_env.sh` 自动生成 CycloneDDS
`CYCLONEDDS_URI`，默认按到 `GO2_DDS_IP=192.168.123.18` 的路由推断网卡。这里的
`GO2_DDS_IP` 主要用于 `ip route get` 选择正确有线网卡；如果现场 DDS topics 实际只能从
另一个 Go2 地址发现，先覆盖 `GO2_DDS_IP`，再重跑 preflight。

## 4. 需要从旧机器迁移的内容

最低需要迁移这几类：

```text
源码仓库:       ${DDDMR_WS}，默认 /home/lin/new2/dddmr_navigation
地图和 bags:    ${DDDMR_BAGS_DIR}，默认 /home/lin/dddmr_bags
Go2 ROS 环境:   ${GO2_SETUP}，默认 /home/lin/go2_workspace/unitree_ros2/setup.sh
可选外层产物:   /home/lin/new2/bags
可选离线地图:   /home/lin/new2/offline_maps
```

当前导航配置引用的地图在 Docker 内路径是：

```text
/root/dddmr_bags/go2_xt16_dddmr_mapping_20260704_081247_map_2026_07_04_00_21_37
```

对应宿主机路径是：

```text
${DDDMR_BAGS_DIR}/go2_xt16_dddmr_mapping_20260704_081247_map_2026_07_04_00_21_37
```

这个目录至少要包含：

```text
map.pcd
ground.pcd
edges.pcd
poses.pcd
```

迁移建议用 `rsync`，排除编译产物但保留源码、脚本、docs、地图和 bags。下面命令里的
`OLD_*` 是旧机器路径，`DDDMR_WS` 和 `DDDMR_BAGS_DIR` 是新笔记本目标路径：

```bash
NEW_LAPTOP=new-laptop-hostname-or-ip
OLD_DDDMR_WS=/home/lin/new2/dddmr_navigation
OLD_DDDMR_BAGS_DIR=/home/lin/dddmr_bags

rsync -aH --info=progress2 \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '.docker_go2_xt16_build/' \
  --exclude '.docker_go2_xt16_install/' \
  --exclude '.docker_go2_xt16_log/' \
  "${OLD_DDDMR_WS}/" \
  "${NEW_LAPTOP}:${DDDMR_WS}/"

rsync -aH --info=progress2 \
  "${OLD_DDDMR_BAGS_DIR}/" \
  "${NEW_LAPTOP}:${DDDMR_BAGS_DIR}/"
```

注意：当前工作区里很多 Go2/XT16 适配文件、docs、scripts 是未跟踪文件。只 `git clone`
上游仓库不够，必须确认这些本地文件也被带到新笔记本。

如果后续要跑 supervised live probe/navigation，还必须安装或迁移 `GO2_SETUP` 指向的
Unitree Go2 ROS 2 workspace；只做 Docker 建图、preflight 和 navigation dry-run 时可先跳过。

## 5. 新笔记本首次环境验证

进入项目：

```bash
cd "${DDDMR_WS}"
```

确认关键文件存在：

```bash
test -f scripts/dddmr_docker_go2_xt16.sh
test -f dddmr_docker/docker_file/Dockerfile_go2_xt16
test -f src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
test -f src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
test -d "${DDDMR_BAGS_DIR}/go2_xt16_dddmr_mapping_20260704_081247_map_2026_07_04_00_21_37"
```

如果准备做 supervised live 运动，再确认 Go2 host-side ROS 环境：

```bash
test -f "${GO2_SETUP}"
```

确认没有旧容器残留：

```bash
docker ps --format '{{.Names}} {{.Status}} {{.Image}}'
```

如果有旧的 `dddmr_go2_xt16`、`go2_xt16_nav`、`go2_xt16_mapping` 容器，先停掉再继续。

## 6. 构建 Docker 镜像

本工程有两层镜像：

```text
base image: dddmr:x64
Go2 image:  dddmr_go2_xt16:x64
```

首次构建：

```bash
cd "${DDDMR_WS}"
./scripts/dddmr_docker_go2_xt16.sh build-go2-image
```

如果 `dddmr:x64` 不存在，脚本会先用 `dddmr_docker/docker_file/Dockerfile_x64` 构建 base
image，再构建 `Dockerfile_go2_xt16`。这个过程会编译 GTSAM 和 PCL，耗时较长。

验证镜像：

```bash
docker image inspect dddmr:x64 >/dev/null
docker image inspect dddmr_go2_xt16:x64 >/dev/null
```

进入容器 shell：

```bash
./scripts/dddmr_docker_go2_xt16.sh shell
```

## 7. 只读 XT16 预检

接好 Go2 和 XT16 后，先跑只读预检：

```bash
cd "${DDDMR_WS}"
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

如果自动网卡判断不对：

```bash
export GO2_NET_IFACE=enp5s0
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

验收点：

```text
/lidar_points 有真实样本
PointCloud2 frame_id 是 hesai_lidar
字段包含 x, y, z, intensity, ring, timestamp
ring 范围是 0..15
点云频率接近 10 Hz
```

不要只看 topic publisher 是否存在。

## 8. 构建和运行建图

构建 LeGO-LOAM 及其依赖：

```bash
cd "${DDDMR_WS}"
./scripts/dddmr_docker_go2_xt16.sh build-lego
```

20 秒 no-motion 建图烟测：

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true \
  ./scripts/dddmr_docker_go2_xt16.sh mapping
```

如果 live graph 已经可靠发布 `base_link -> go2_imu -> hesai_lidar`，可以把
`PUBLISH_STATIC_TF=false`。默认首次迁移测试保留 `true` 更容易排除 TF 缺失。

当前 XT16 建图配置：

```text
src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_live.yaml
```

关键参数：

```yaml
num_vertical_scans: 16
num_horizontal_scans: 2000
vertical_angle_bottom: -15.0
vertical_angle_top: 15.0
scan_period: 0.1
patch_first_ring_to_baselink: true
use_omp_ground_voxel_filter: false
enable_loop_closure: false
```

## 9. 离线 bag 建图

当前脚本支持离线 rosbag2 建图：

```bash
CONFIG_FILE=/root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_go2_xt16_20260704_081247.yaml \
DDDMR_MAPPING_DIR=/root/dddmr_bags/go2_xt16_dddmr_mapping_20260704_081247_map_ \
  ./scripts/dddmr_docker_go2_xt16.sh mapping-bag
```

输出目录会写到 `/root/dddmr_bags`，也就是宿主机的 `${DDDMR_BAGS_DIR}`。

## 10. 构建和运行导航 dry-run

构建导航链路：

```bash
cd "${DDDMR_WS}"
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

启动 RViz dry-run：

```bash
RVIZ=true PUBLISH_STATIC_TF=true \
  ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run
```

非交互 smoke test：

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true \
  ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run
```

当前导航入口：

```text
src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch
src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
src/dddmr_beginner_guide/rviz/go2_xt16_navigation.rviz
```

dry-run 安全隔离：

```text
p2p_move_base /cmd_vel         -> /dddmr_go2/dry_run_cmd_vel
p2p_move_base /cmd_vel_stamped -> /dddmr_go2/dry_run_cmd_vel_stamped
```

默认还会走：

```text
/dddmr_go2/dry_run_cmd_vel -> go2_nav_cmd_gate -> /dddmr_go2/safe_cmd_vel
/dddmr_go2/safe_cmd_vel    -> go2_sport_cmd_vel_dry_run
```

`go2_sport_cmd_vel_dry_run` 只打印将要发布的 Sport API 请求，不创建真实
`/api/sport/request` 发布器。

RViz 中重点看：

```text
/map1/mapcloud
/map1/mapground
/sub_mapcloud
/sub_mapground
/weighted_ground
/global_path
/dddmr_go2/dry_run_cmd_vel
/dddmr_go2/safe_cmd_vel
```

## 11. 切换地图

导航配置中的地图路径：

```yaml
map1:
  ros__parameters:
    pose_graph_dir: "/root/dddmr_bags/go2_xt16_dddmr_mapping_20260704_081247_map_2026_07_04_00_21_37"
```

如果新笔记本使用不同地图，修改：

```text
src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
```

要求：

- `pose_graph_dir` 使用 Docker 内路径，例如 `/root/dddmr_bags/<map_dir>`。
- 宿主机上必须有对应 `${DDDMR_BAGS_DIR}/<map_dir>`。
- 目录内必须有 `map.pcd`、`ground.pcd`、`edges.pcd`、`poses.pcd`。
- 换图后重新跑 `build-navigation` 或至少重启 `navigation-dry-run`。

## 12. 监督 live 运动入口

只有 dry-run 通过并得到明确批准后才使用本节。

先跑只读 readiness gate：

```bash
cd "${DDDMR_WS}"
./scripts/check_go2_xt16_sport_live_readiness.sh
```

期望：

```text
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
```

短低速 Sport adapter live probe：

```bash
GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
  ./scripts/run_go2_sport_adapter_supervised_probe.sh --live
```

RViz live navigation 入口：

```bash
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_YYYYMMDD_HHMMSS_summary.env \
  ./scripts/run_go2_xt16_navigation_supervised_live.sh --live
```

这条路线会把 Docker 中 DDDMR 作为速度源，真实 `/api/sport/request` 由宿主机 Sport
adapter 发布。不要绕过 readiness gate 和 live probe。

## 13. 常见故障

### Docker 构建慢或失败

`dddmr:x64` 会从源码编译 PCL 1.15.0 和 GTSAM 4.2a9，首次构建可能很慢。网络不稳时建议先确认 Docker 官方 apt 源、GitHub 访问和磁盘空间。

### Docker 里看不到 Go2 topics

按顺序查：

```bash
ip -br addr
ip route
ping -c 2 -W 1 "${GO2_DDS_IP}"
GO2_NET_IFACE="${GO2_NET_IFACE:-enp5s0}" ./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

如果 ROS graph 只有 `/parameter_events` 和 `/rosout`，优先怀疑 DDS 网卡绑定或路由。

### RViz 不显示

先确认当前是可用的桌面会话：

```bash
echo "$DISPLAY"
xhost +local:docker
RVIZ=true ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run
```

Wayland 下显示问题较多，建议先用 X11 会话验证。

### 地图找不到

检查宿主机路径和 Docker 内路径是否对应：

```bash
ls -la "${DDDMR_BAGS_DIR}"
rg -n "pose_graph_dir" src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml
```

如果配置写的是 `/root/dddmr_bags/<map_dir>`，宿主机应存在 `${DDDMR_BAGS_DIR}/<map_dir>`。

### `lego_loam` 崩溃

优先检查是否混用了宿主机 PCL/GTSAM 和 Docker 构建产物。Go2 XT16 路线应通过：

```bash
./scripts/dddmr_docker_go2_xt16.sh build-lego
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh mapping
```

如果仍崩溃，按上游 XT16 issue 思路检查 scan 参数、ground FOV、TF 和最小 bag。

### 有路径但没有速度

检查：

```text
/global_path 是否存在
local planner odom topic 是否有效
TF 是否新鲜
map1/mapground 和 weighted_ground 是否连通
/dddmr_go2/dry_run_cmd_vel 是否有输出
```

### 有 dry-run 速度但不能进入 live

不要直接绕过 gate。先跑：

```bash
./scripts/check_go2_xt16_sport_live_readiness.sh
```

如果提示有 stale Docker/RViz/navigation/adapter 进程，先清理残留进程和容器，再重跑 readiness。

## 14. 新笔记本最短验证顺序

```bash
cd "${DDDMR_WS}"

./scripts/dddmr_docker_go2_xt16.sh build-go2-image
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
./scripts/dddmr_docker_go2_xt16.sh build-navigation

RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true \
  ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run

RVIZ=true PUBLISH_STATIC_TF=true \
  ./scripts/dddmr_docker_go2_xt16.sh navigation-dry-run
```

第一轮只验收：

```text
Docker 镜像能构建
XT16 /lidar_points 可读
当前 pose_graph_dir 地图能加载
RViz 中地图/TF/机器人位置能对齐
点击 3D goal 后有 /global_path
dry-run topic 有速度输出
没有真实 /cmd_vel 或 /api/sport/request 输出
```

## 15. 参考入口

- Docker wrapper: `scripts/dddmr_docker_go2_xt16.sh`
- Docker quickstart: `docs/go2_xt16_docker_quickstart.md`
- 技术路线: `docs/go2_xt16_technical_document_and_route.md`
- live 运动 runbook: `docs/go2_xt16_sport_live_runbook.md`
- Go2 XT16 导航 launch: `src/dddmr_beginner_guide/launch/go2_xt16_navigation.launch`
- Go2 XT16 导航 config: `src/dddmr_beginner_guide/config/go2_xt16_navigation.yaml`
- Go2 XT16 建图 config: `src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_live.yaml`
- ROS 2 Humble 官方安装页: `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`
- Docker Engine Ubuntu 官方安装页: `https://docs.docker.com/engine/install/ubuntu/`
