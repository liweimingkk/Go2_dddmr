# Go2 XT16 Sport Live Runbook

This runbook is for the supervised handoff from DDDMR dry-run navigation to the
Go2 Sport API.

## Boundary

- Dry-run Docker navigation remains no-motion.
- Real Sport output runs from the host Go2 ROS environment, not from Docker.
- Do not run live steps without onsite supervision, clear floor space, and a
  ready physical stop path.
- Do not publish `/lowcmd`.
- Do not modify Go2 network, services, startup files, firmware, calibration, or
  `/home/unitree/xt16_ws`.

## Current Routing

The live runtime is intentionally split:

```text
Docker: DDDMR navigation/RViz
  p2p_move_base /cmd_vel
    -> /dddmr_go2/dry_run_cmd_vel

Host: Go2 Sport adapter
  /dddmr_go2/dry_run_cmd_vel
    -> clamp/watchdog/StopMove
    -> /api/sport/request
```

The real adapter only creates a `/api/sport/request` publisher when both gates
are enabled:

```text
enable_sport_output=true
allow_real_request_topic=true
```

Default launch values keep live Sport output disabled. Docker must never publish
the real Sport API; it is only the velocity source.

## Build

Build the navigation stack in Docker before live use:

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

Do not use the host as the full DDDMR navigation build target for live bringup;
the host only runs the Go2 ROS Sport adapter.

## Readiness

Run the read-only readiness gate:

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/check_go2_xt16_sport_live_readiness.sh
```

Expected final line:

```text
RESULT: GO2_XT16_SPORT_LIVE_READINESS_PASS
```

This check does not publish `/cmd_vel`, `/dddmr_go2/dry_run_cmd_vel`, or
`/api/sport/request`.

## WASD Terminal Teleop

The standalone keyboard teleop defaults to preview mode and does not create a
ROS publisher:

```bash
cd /home/lin/new2/dddmr_navigation
./scripts/go2_wasd_teleop.py
```

Controls:

```text
W / S       forward / backward
A / D       turn left / right
Space or X  StopMove immediately
Q or Esc    StopMove and quit
Ctrl-C      StopMove and quit
```

A direction key remains active for only `0.35 s` without keyboard repeat. The
watchdog then publishes `StopMove`; normal exit and handled terminal signals
also publish three final `StopMove` requests. Default live limits are
`0.10 m/s` linear and `0.25 rad/s` yaw.

Live mode requires the host Unitree ROS messages, CycloneDDS, the exact
confirmation phrase, onsite supervision, clear floor space, and a ready App or
physical stop path:

```bash
cd /home/lin/new2/dddmr_navigation
source /opt/ros/humble/setup.bash
source .unitree_msg_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
GO2_WASD_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_WASD \
  ./scripts/go2_wasd_teleop.py --live
```

The live script refuses to start until `/api/sport/request` has a subscriber.
It publishes only `1008 Move` and `1003 StopMove`. Do not run it alongside
autonomous navigation or another Sport velocity controller. On this Go2,
pure-yaw Sport commands have previously produced weak turning; do not add an
implicit forward arc to `A/D` without a separate supervised validation.

## Live Probe

Only after readiness passes and onsite supervision is active, run the short
low-speed probe:

```bash
cd /home/lin/new2/dddmr_navigation
GO2_SPORT_LIVE_CONFIRM=I_AM_SUPERVISING_GO2 \
  ./scripts/run_go2_sport_adapter_supervised_probe.sh --live
```

Default probe limits:

```text
x=0.05
y=0.0
yaw=0.0
duration=0.6s
```

Hard caps enforced by the script:

```text
abs(x) <= 0.10
abs(y) <= 0.05
abs(yaw) <= 0.35
duration <= 1.0s
```

The default yaw clamp remains lower:

```text
GO2_SPORT_MAX_YAW=0.25
```

The probe writes a summary:

```text
SUMMARY_LOG=/tmp/go2_sport_adapter_live_<timestamp>_summary.env
```

That summary must point to an echo log proving this probe's unique
`REQUEST_ID_BASE` observed both:

```text
api_id: 1008  # Move
api_id: 1003  # StopMove
```

## RViz Live Navigation

Start RViz live navigation only after the live probe summary exists. The
supervisor starts Docker `navigation-live-source`, waits for the host to see
`/dddmr_go2/dry_run_cmd_vel`, starts the host Sport adapter, and records
`/api/sport/request`:

```bash
cd /home/lin/new2/dddmr_navigation
GO2_NAV_LIVE_CONFIRM=I_AM_SUPERVISING_GO2_NAV \
GO2_SPORT_PROBE_SUMMARY=/tmp/go2_sport_adapter_live_<timestamp>_summary.env \
  ./scripts/run_go2_xt16_navigation_supervised_live.sh --live
```

The script refuses to start live navigation when:

- the navigation confirmation phrase is missing;
- the Sport live probe summary is missing;
- the summary is from `typed-preview` instead of `live`;
- the summary echo log does not prove both `1008 Move` and `1003 StopMove` for
  that probe's request id range;
- stale Docker, RViz, navigation, adapter, or `ros2 topic pub/echo` processes
  are present;
- Docker does not expose a `/dddmr_go2/dry_run_cmd_vel` publisher to the host.

When RViz opens, use a short, nearby, clear-space goal first. Treat rotational
commands without forward `x > 0` as navigation/planner behavior, not proof of
forward motion.

Current supervised live navigation default:

```text
GO2_SPORT_MAX_X=0.30
```

This is separate from the live Sport probe above, which remains a smaller
adapter handshake capped at `0.10`.

## Evidence To Keep

Record these paths after each supervised run:

```text
ADAPTER_LOG=...
DOCKER_LOG=...
REQUEST_ECHO_LOG=...
SUMMARY_LOG=...
```

For RViz live navigation, record the terminal output, exact clicked goal, and
whether the request ids after `REQUEST_ID_BASE` contain the expected Sport
requests.

## Stop And Cleanup

If any behavior is unexpected, stop the launch with `Ctrl-C`, then verify:

```bash
docker ps --format '{{.Names}} {{.Status}} {{.Image}}'
ps -eo pid,args | rg 'go2_sport_cmd_vel_adapter|go2_sport_cmd_vel_dry_run|go2_xt16_navigation.launch|rviz2|p2p_move_base_node|ros2 topic pub|ros2 topic echo' | rg -v 'rg |ps -eo' || true
```

Re-run the readiness gate before any next live attempt.
