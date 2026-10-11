# uav_ugv_coordinator

Mission-level coordinator for energy-aware collaborative exploration with a
UAV–UGV team. This is the top-level package of
[uav-ugv-collaborative-exploration](../../../README.md).

## What it does

Runs an 8-state finite state machine at 20 Hz that sequences one exploration
tour and repeats it until the environment is explored:

```
INITIAL_TAKEOFF → INITIAL_SCAN → TRIGGER_PLANNER → WAIT_FOR_PATHS
      → EXECUTE_DUAL_PATHS → RENDEZVOUS_LAND → LOOP_RESET ─┐
             ▲                                             │
             └─────────────────────────────────────────────┘
                   (MISSION_COMPLETE on /exploration/complete)
```

The UAV and UGV execute their tours **simultaneously**. Each tour ends with the
UAV landing on a platform mounted on the UGV, where it recharges before the next
tour.

## Important: this replaces `dynamic_exploration_node`

The node constructs `AutoFlight::dynamicExploration` with a global
`NodeHandle`, which pulls in the whole UAV autonomy stack — flight base,
occupancy map, DEP planner and B-spline trajectory planner — into this process.

Do **not** run `autonomous_flight dynamic_exploration.launch` at the same time.

## Run

```bash
roslaunch uav_ugv_coordinator uav_ugv_coordinator.launch
```

This is step 3 of the four-terminal launch sequence; see the
[top-level README](../../../README.md#run-the-collaborative-exploration-demo).

## ROS interface

| Direction | Topic | Type |
|---|---|---|
| sub | `/CERLAB/quadcopter/pose` | `geometry_msgs/PoseStamped` |
| sub | `/uav/op_path` | `nav_msgs/Path` |
| sub | `/ugv/op_path` | `nav_msgs/Path` |
| sub | `/exploration/complete` | `std_msgs/Bool` |
| pub | `/uav_ugv_coordinator/waypoints` | `nav_msgs/Path` (latched) |
| pub | `/global_planner/request_op` | `std_msgs/Empty` (latched) |
| action | `/move_base` | `move_base_msgs/MoveBaseAction` |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `map_frame` | `map` | Frame for `move_base` goals |
| `scan_hold_sec` | `5.0` | Dwell at each yaw stop of the initial scan (s) |
| `ugv_nav_timeout_sec` | `300.0` | Per-waypoint `move_base` timeout (s) |
| `reach_tol_xyz` | `0.25` | UAV waypoint arrival tolerance (m) |

Full reference: [`docs/PARAMETERS.md`](../../../docs/PARAMETERS.md).

## Sources

| File | Built? | Purpose |
|---|---|---|
| `src/uav_ugv_coordinator.cpp` | yes | The method: simultaneous dual-path execution |
| `src/uav_ugv_coordinator_benchmarking.cpp` | no | Sequential baseline arm — see [`docs/BENCHMARKS.md`](../../../docs/BENCHMARKS.md) |

## Monitoring

Useful log lines during a run:

```
[Coordinator] Requesting New Plans
[Coordinator] Received BOTH paths
[Coordinator] Touchdown Confirmed
```

## License

MIT. See the [top-level LICENSE](../../../LICENSE).
