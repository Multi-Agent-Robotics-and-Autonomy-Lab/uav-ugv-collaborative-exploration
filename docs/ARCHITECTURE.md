# Software Architecture

Module-level design of the UAV–UGV collaborative exploration system.

> This document describes the code as it stands. Where an earlier internal
> design note disagrees with the implementation, the code is authoritative and
> the difference is called out.

---

## 1. Overview

Three modules do the work:

| Module | File | Role |
|---|---|---|
| **Coordinator** | `uav_ugv_coordinator/src/uav_ugv_coordinator.cpp` | Mission-level finite state machine |
| **Planner (DEP)** | `global_planner/include/global_planner/dep.cpp` | Layered PRM, information gain, coupled orienteering problems |
| **UAV flight stack** | `autonomous_flight/include/autonomous_flight/simulation/dynamicExploration.cpp` | Trajectory generation, execution, collision avoidance |

Supporting: `map_manager` (occupancy map + UGV fusion), `trajectory_planner`
(B-spline), `tracking_controller`, `uav_simulator` (Gazebo model, plugins,
worlds), `scout_*` (UGV model and navigation).

### One process, not three

The coordinator is **not** a standalone node that talks to a separate planner.
It constructs the entire UAV autonomy stack inside its own process:

```cpp
ros::NodeHandle nh_global;
de_ = std::make_shared<AutoFlight::dynamicExploration>(nh_global);
```

Constructing `dynamicExploration` pulls in `flightBase`, `mapManager::dynamicMap`,
`globalPlanner::DEP` and `trajPlanner::bsplineTraj`. Consequently the
`uav_ugv_coordinator` binary **replaces** `dynamic_exploration_node`, and
`uav_ugv_coordinator.launch` loads exactly the same six parameter files as
`dynamic_exploration.launch`. Running both would start two exploration stacks
and two tracking controllers.

This is also why `dynamicExploration` inherits *publicly* from `flightBase` and
re-exports `takeoff`, `moveToOrientation` and `stop` — the coordinator calls
them directly, and replanning is triggered by an in-process call
(`de_->triggerExplorationReplan()`) rather than over a topic.

---

## 2. Coordinator — mission state machine

Node `uav_ugv_coordinator`, main loop at **20 Hz**. One-shot states are guarded
by a static `last_executed_state` so they fire once despite the tick rate.

### States

| State | Behaviour |
|---|---|
| `INITIAL_TAKEOFF` | Take off, allow the UAV to stabilise. |
| `INITIAL_SCAN` | 360° yaw sweep in four increments at 0.5 rad/s, dwelling `scan_hold_sec` at each, to seed the map. Then initialise the planner and register callbacks. |
| `TRIGGER_PLANNER` | Clear path flags and request a new plan from DEP. |
| `WAIT_FOR_PATHS` | Poll until **both** `/uav/op_path` and `/ugv/op_path` have arrived. |
| `EXECUTE_DUAL_PATHS` | Run both robots **simultaneously**: publish the whole UAV path at once; dispatch UGV waypoints one at a time through `move_base`. |
| `RENDEZVOUS_LAND` | Cancel UGV motion and land the UAV on the UGV platform. |
| `LOOP_RESET` | Take off again and return to `TRIGGER_PLANNER`. |
| `MISSION_COMPLETE` | Terminal state, entered when the planner latches `/exploration/complete`. |

Touchdown is confirmed only when the UAV stays below **z = 0.75 m** for two
consecutive seconds, so a single noisy altitude sample cannot end a tour early.

### Rendezvous alignment

In `sendNextUgvWaypoint()`, each UGV goal is given a yaw pointing at the
*following* waypoint, for smooth navigation. The **final** UGV waypoint instead
inherits the **UAV's final orientation** — that is what aligns the landing
platform with the descending UAV.

### Abort on replan

If a new `/uav/op_path` arrives during `EXECUTE_DUAL_PATHS`, the UAV has
replanned around an obstacle on its own. The coordinator cancels the active
`move_base` goal and returns to `TRIGGER_PLANNER` so both tours are recomputed
together — the tours are coupled, so re-planning one in isolation would break
the rendezvous guarantee.

Both `RENDEZVOUS_LAND` and `LOOP_RESET` call `de_->abortTrajectory()` before
`land()` / `takeoff()`. Without it, a stale B-spline still queued in `trajExeCB`
overrides the hover hold and flies the UAV back to the origin.

### ROS interface

| Direction | Topic | Type |
|---|---|---|
| sub | `/CERLAB/quadcopter/pose` | `PoseStamped` |
| sub | `/uav/op_path` | `nav_msgs/Path` |
| sub | `/ugv/op_path` | `nav_msgs/Path` |
| sub | `/exploration/complete` | `std_msgs/Bool` |
| pub | `/uav_ugv_coordinator/waypoints` | `nav_msgs/Path` (latched) |
| pub | `/global_planner/request_op` | `std_msgs/Empty` (latched) |
| action | `/move_base` | `MoveBaseAction` |

`/global_planner/request_op` is vestigial — it is advertised, but replanning is
actually triggered in-process. It is retained because the parameter and topic
are part of the existing wiring.

---

## 3. Planner — layered PRM and coupled orienteering

`makePlan()` runs this pipeline:

1. **Frontier detection** — slice the occupancy map at several heights, then
   OpenCV blob detection to find free/unknown boundaries.
2. **UAV PRM** (`buildRoadMap()`) — frontier-biased, local and global sampling
   within `global_region_min/max`; k-NN connection (k = 15) with collision-free
   line-of-sight edges.
3. **Node pruning** — drop nodes invalidated since they were sampled.
4. **UGV PRM**, in two stages:
   - `buildUGVFromUAVProjection()` projects each *new* UAV node vertically to the
     ground plane, validating occupancy at both the base and sensor height. It
     deliberately does **not** clear the previous roadmap, so the ground graph
     accumulates across tours. The map `projOf_` (UAV node → ground twin) is the
     coupling between the two graphs.
   - `buildUGVRoadMap()` adds *independently sampled* ground nodes with no aerial
     counterpart, then k-NN-connects. This is what makes the roadmap genuinely
     layered rather than a flat projection: the ground layer has its own nodes
     and its own connectivity.
5. **Information gain** — raycast per node. The UAV variant uses the camera FOV
   over 32 yaw bins; `calculateUnknownUGV` raycasts from `z + ugvSensorHeight_`
   out to `ugvMaxRange_`.
6. **Target selection** — the coupling step, below.
7. **UAV OP solve** — GRASP-PR over the aerial graph, budget `timeBudgetSec_`.
8. **UGV OP solve** — GRASP-PR over the ground graph, ending at the ground
   projection of the UAV's final waypoint, which is what guarantees rendezvous.

### Target selection

Dijkstra runs over the UGV graph from the UGV's current node, then every UAV
node is scored:

```
score = info_gain · exp(−λ · t_ugv_path)
```

subject to four hard gates — a candidate is rejected if its information gain is
≤ 10, if it has no ground twin in `projOf_`, if the vertical corridor between
the aerial node and its twin is not collision-free, or if the twin is
unreachable in the ground graph.

So the UAV's goal is chosen by **aerial information gain discounted by how far
the UGV must drive to get underneath it** — the mechanism by which the two tours
stay coupled and the rendezvous stays feasible.

### GRASP-PR orienteering solver

Maximise collected information gain subject to a travel-time budget:

- **Constructive phase (C4)** — greedy insertion by score-to-time-increment
  ratio, randomised through a restricted candidate list (`ALPHA`).
- **Local search** — 2-opt reversal plus swap-insert.
- **Path relinking** — between pairs of elite solutions, in both directions.

`buildScoresAndTimes` builds the time matrix with **infinite entries for
non-edges**, and `edgeExists` / `validatePathUsesPRMEdges` enforce that no move
ever shortcuts off the roadmap. This is what the paper means by tours
constructed over collision-validated roadmap edges.

### Completion

`getExplorationStatistics()` gives explored voxels; coverage is
`explored / total_voxels`. The mission ends above 97% coverage or when total
remaining information gain falls below 150, latching `/exploration/complete`.
80% and 95% milestones are timestamped for the paper's metrics.

---

## 4. Shared mapping

Bidirectional, but all planning is aerial.

**UGV → UAV.** The UGV's Velodyne cloud is ground-filtered and voxelised, passed
through `map_fusion_node`, and injected into the UAV's occupancy map via
`injectPointCloudWithExclusions()`. The gmapping 2D grid is painted in as free
floor through `inject2DFreeSpace()`.

Critically, before injecting, `clearFootprints()` erases the UGV's own body from
the map using its pose history. Without this the rover carves a *moving phantom
obstacle* through the roadmap, invalidating nodes behind itself as it drives.

**UAV → UGV.** There is no planner on the UGV. DEP runs inside the UAV process,
reads the fused map, projects the aerial roadmap down, and publishes a finished
tour on `/ugv/op_path`, which the coordinator feeds to `move_base` one waypoint
at a time. The UGV contributes sensing and odometry only.

### New `map_manager` API

`occupancyMap` gained: `injectPointCloud`, `injectPointCloudWithExclusions`,
`inject2DFreeSpace`, `clearFootprints`, `getExplorationStatistics`,
`getExploredVolume`, `getExploredVoxelCount`.

---

## 5. UAV flight stack

`dynamicExploration` runs the callbacks that execute a tour:

| Callback | Rate |
|---|---|
| `exploreReplan()` | 10 Hz |
| `plannerCB()` | 50 Hz |
| `replanCheckCB()` | 100 Hz |
| `trajExeCB()` | 100 Hz |
| `visCB()` | 30 Hz |

`explorationReplan_` defaults to **false** here (upstream: true) because the
coordinator owns the replan trigger. `explorationCB` no longer assigns its own
waypoints — they arrive on `/uav_ugv_coordinator/waypoints`.

### Simulator robustness fixes

Several changes in `flightBase` and the Gazebo plugin exist because of failures
that are easy to misdiagnose:

- **Origin teleport.** The Gazebo quadcopter plugin computed
  `load_factor = g²/(g·g_body)`, which passes through zero near 90° tilt,
  producing infinite thrust and NaN that ODE folds into the model permanently.
  The load factor is now clamped with `isfinite` guards.
- **Poisoned PID state.** The PID gains were uninitialised and a single NaN in
  the integral accumulator was permanent. Inputs are now sanitised, internal
  state self-heals, and an anti-windup clamp was added.
- **Takeoff to (0,0,0).** A stale-odometry check now aborts takeoff rather than
  flying to the origin, and a `poseTgtInit_` guard stops the 200 Hz publisher
  commanding a zero target before takeoff.
- **Serialization race.** `targetMtx_` guards `poseTgt_`/`stateTgt_`, fixing a
  `StreamOverrunException`.

---

## 6. Message flow for one exploration cycle

```
1. Coordinator: INITIAL_SCAN seeds the map (360° sweep)
2. Coordinator ──► DEP                     triggerExplorationReplan()
3. DEP: extend layered PRM, update information gain
4. DEP: Dijkstra on ground graph, score + gate UAV candidates
5. DEP: solve UAV OP, then UGV OP ending under the UAV's last waypoint
6. DEP ──► /uav/op_path, /ugv/op_path
7. Coordinator ──► /uav_ugv_coordinator/waypoints   (UAV, whole path)
   Coordinator ──► /move_base                        (UGV, one waypoint at a time)
8. Both execute; UGV LiDAR flows back into the shared map via map_fusion_node
9. Coordinator: RENDEZVOUS_LAND — UAV lands on the UGV and recharges
10. LOOP_RESET ──► back to step 2
```

---

## 7. Differences from the earlier internal design note

An earlier `.docx` design note circulated with this project. Where it disagrees
with the code, the code is correct:

| Item | Design note | Code |
|---|---|---|
| `λ` (utility weight) | 0.15 | **0.001** (`dep.cpp:595`), matching the paper |
| UGV speed in OP | 0.3 m/s | **1.0 m/s** (`dep.cpp:848`) |
| `scan_hold_sec` | 1.0 | **5.0** (launch file) |
| `ugv_nav_timeout_sec` | 100.0 | **300.0** |
| `reach_tol_xyz` | 0.3 | **0.25** |
| PRM height floor | z ≥ 1.0 m | **z ≥ 0.7 m** (`global_region_min`) |
| Coordination model | sequential | **simultaneous** (`EXECUTE_DUAL_PATHS`) |
| State count | 7 | **8** (`MISSION_COMPLETE` added) |
| UGV roadmap | mirrors UAV edges + k-NN stitching | incremental projection, connectivity deferred to `buildUGVRoadMap()` |

The sequential model the note describes is implemented in
`uav_ugv_coordinator_benchmarking.cpp`, which is a benchmark arm — not the
method. The note also frames the UGV as a "mobile launch platform"; the paper's
framing, and the one the energy model implements, is a **mobile charging
station** with an explicit flight-time budget.
