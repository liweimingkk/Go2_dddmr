# DDDMR Web Viewer

`dddmr_web_viewer` provides a Three.js browser view for the stable Go2 XT16
DDDMR stack. It renders the complete map, traversable ground, robot TF, global
path and local pruned path without replacing the DDDMR point-cloud map or
planners with OctoMap.

The bridge is fail-closed:

- navigation execution defaults to disabled;
- selecting a goal first calls the read-only `/get_plan` action;
- the preview expires after 30 seconds and may be executed only once;
- a second, explicit browser confirmation is required before
  `/p2p_move_base` receives a goal;
- the stop button cancels only a goal started through this bridge;
- no browser joystick or direct velocity topic is provided.

## Topic adaptation

| Browser-side interface | DDDMR interface | Purpose |
| --- | --- | --- |
| `/dddmr_web/mapcloud` | `/map1/mapcloud` | Cached obstacle/map point cloud |
| `/dddmr_web/mapground` | `/map1/mapground` | Cached traversable ground cloud |
| `/dddmr_web/preview_goal` | `/get_plan` action | Read-only route preview |
| `/dddmr_web/preview_path` | `GetPlan` result | Confirmable preview path |
| `/dddmr_web/initial_pose` | `/initial_3d_pose` | Validated localization seed |
| `/dddmr_web/execute_preview` | `/p2p_move_base` action | One-shot confirmed execution |
| `/dddmr_web/cancel_navigation` | current web goal cancel | Stop web-started navigation |

The map relay caches the transient-local DDDMR point clouds and republishes
them when a browser requests status. This avoids depending on rosbridge's
subscription QoS to recover a map that was published before the browser
connected.

## Build

Use the repository's authoritative Docker path:

```bash
cd dddmr_navigation
./scripts/dddmr_docker_go2_xt16.sh build-navigation
```

The Go2 wrapper image includes `rosbridge_server`. A manual environment must
provide the matching ROS package, for example on Humble:

```bash
sudo apt install ros-humble-rosbridge-server
```

## No-motion launch

Start the normal navigation graph with its existing dry-run/live safety
settings, then launch the viewer from a second terminal:

```bash
./scripts/dddmr_docker_go2_xt16.sh web-viewer
```

The x64 wrapper keeps the repository's 16 MiB CycloneDDS receive-buffer
safety gate. If startup reports a smaller host buffer, follow the existing
`scripts/check_go2_dds_receive_buffers.sh` guidance before using live XT16
point clouds; do not weaken the gate for a robot session.

Open `http://127.0.0.1:8080`. The page connects to
`ws://127.0.0.1:9090` by default.

For a remote laptop, keep both servers bound to localhost and use an SSH
tunnel:

```bash
ssh -L 8080:127.0.0.1:8080 -L 9090:127.0.0.1:9090 robot-host
```

Do not expose rosbridge directly to an untrusted network: a rosbridge client
can otherwise access more of the ROS graph than the viewer UI exposes.
On Humble, the supplied launch additionally limits browser publishing to
`/dddmr_web/*` and limits subscriptions to the viewer, TF, and path topics.
Localhost binding or an equivalent trusted network boundary remains required;
topic filters are defense in depth, not authentication.

## Guarded execution

Only for an already-approved, supervised navigation session, enable the
bridge's final action step explicitly:

```bash
WEB_ALLOW_NAVIGATION_EXECUTION=true \
  ./scripts/dddmr_docker_go2_xt16.sh web-viewer
```

This flag does not bypass `p2p_goals_enabled`, the Go2 navigation command gate,
the Sport adapter's output gate, localization health checks, or onsite
supervision requirements. Selecting a point remains preview-only until the
operator confirms the displayed route.

## Upstream inspiration

The interaction model is inspired by
[`6-robot/jie_3d_nav`](https://github.com/6-robot/jie_3d_nav), while the web
application and ROS bridge in this package are implemented for DDDMR's native
`PointCloud2` and action interfaces. Third-party JavaScript notices are in
`THIRD_PARTY_NOTICES.md`.
