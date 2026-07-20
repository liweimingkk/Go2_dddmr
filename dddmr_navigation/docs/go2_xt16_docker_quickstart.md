# Go2 XT16 Docker Quickstart

This workspace keeps the upstream DDDMR Docker files and adds only a small Go2 XT16 wrapper layer.

## Boundary

- Docker runs on the host, not on the Go2.
- The first checks are perception-only and no-motion.
- Do not publish `/cmd_vel`, `/api/sport/request`, or any low-level motion command from this flow.
- Do not modify the Go2 network, services, startup files, or `/home/unitree/xt16_ws`.

## Official Docker

The upstream Docker assets are in:

```bash
dddmr_docker/
```

The upstream base image is:

```bash
dddmr:x64
```

The Go2 wrapper uses a thin derived image because the upstream x64 image does not include the CycloneDDS RMW package needed by the current Go2 ROS 2 setup:

```bash
dddmr_go2_xt16:x64
```

This host already has `dddmr:x64` in Docker. Build the small Go2 layer once:

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-go2-image
```

## XT16 Contract

The clean Go2 XT16 config is:

```text
src/dddmr_lego_loam/lego_loam_bor/config/loam_go2_xt16_live.yaml
```

It assumes:

- topic: `/lidar_points`
- frame: `hesai_lidar`
- fields: `x`, `y`, `z`, `intensity`, `ring`, `timestamp`
- width: `32000`
- point step: `26`
- rings: `0..15`
- points per ring: `2000`
- scan period: `0.1`

This is the active navigation mode (`10 Hz`). The optional `64000`-point mode
uses `4000` points per ring but has measured only about `5 Hz` on this system;
use matching projection parameters whenever the driver mode changes.

## Commands

From the clean workspace:

```bash
cd /home/lin/new2/dddmr_navigation
```

Start a shell in the official Docker image:

```bash
./scripts/dddmr_docker_go2_xt16.sh shell
```

Run the read-only live XT16 preflight:

```bash
./scripts/dddmr_docker_go2_xt16.sh preflight --samples 3 --timeout 10
```

Build LeGO-LOAM inside Docker:

```bash
./scripts/dddmr_docker_go2_xt16.sh build-lego
```

The build goes to ignored hidden directories:

```text
.docker_go2_xt16_build/
.docker_go2_xt16_install/
.docker_go2_xt16_log/
```

Run a 20 second no-motion mapping smoke test:

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=false ./scripts/dddmr_docker_go2_xt16.sh mapping
```

If the Go2 graph does not already publish `base_link -> go2_imu -> hesai_lidar`, enable the local static TF only for the smoke test:

```bash
RUN_SECONDS=20 RVIZ=false PUBLISH_STATIC_TF=true ./scripts/dddmr_docker_go2_xt16.sh mapping
```

## Network Overrides

Defaults:

```text
GO2_DDS_IP=192.168.123.18
ROS_DOMAIN_ID=0
```

If the host wired interface is not auto-detected:

```bash
GO2_NET_IFACE=enp5s0 ./scripts/dddmr_docker_go2_xt16.sh preflight
```

Use current read-only output as truth if the interface or ROS domain differs.
