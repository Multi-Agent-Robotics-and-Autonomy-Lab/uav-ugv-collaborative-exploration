# Parameter Reference

Every tunable in the collaborative exploration pipeline, where it lives, and
what it does. Values shown are the defaults as shipped.

Parameters fall into three tiers:

| Tier | Location | Rebuild needed |
|---|---|---|
| 1 | `roslaunch` arguments and `<param>` tags | No |
| 2 | YAML under `autonomous_flight/cfg/dynamic_exploration/` | No |
| 3 | Compile-time constants in `dep.cpp` / `dep.h` | **Yes** |

Paths below are relative to `src/CERLAB-UAV-Autonomy/`.

---

## Tier 1 — Launch arguments

### `uav_simulator/launch/try.launch`

| Argument | Default | Meaning |
|---|---|---|
| `wname` | `construction_site_big_v` | World from `uav_simulator/worlds/floorplan3/`. **See [Switching environments](#switching-environments).** |
| `planner` | `teb` | UGV local planner: `teb`, `dwa` or `base` |
| `paused` | `true` | Start Gazebo paused so both robots finish spawning; press ▶ to run |
| `gui` | `true` | Gazebo client window |
| `debug` | `false` | Run `gzserver` under gdb (diagnostics only) |

### `uav_ugv_coordinator/launch/uav_ugv_coordinator.launch`

| Parameter | Default | Meaning |
|---|---|---|
| `map_frame` | `map` | Frame for `move_base` goals sent to the UGV |
| `scan_hold_sec` | `5.0` | Dwell at each of the four yaw stops of the initial 360° scan (s) |
| `ugv_nav_timeout_sec` | `300.0` | Per-waypoint `move_base` timeout before the UGV skips ahead (s) |
| `reach_tol_xyz` | `0.25` | 3D tolerance for declaring a UAV waypoint reached (m) |
| `uav_path_topic` | `/uav/op_path` | UAV tour published by the planner |
| `ugv_path_topic` | `/ugv/op_path` | UGV tour published by the planner |
| `ext_waypoints_topic` | `/uav_ugv_coordinator/waypoints` | Waypoints handed to the UAV flight stack |
| `completion_topic` | `/exploration/complete` | Mission-complete latch from the planner |

Raise `ugv_nav_timeout_sec` if the UGV legitimately needs longer on a leg;
lower it if a stuck rover is stalling the mission.

### `map_manager/launch/map_fusion.launch`

| Parameter | Default | Meaning |
|---|---|---|
| `fusion_period` | `1.0` | How often UGV cloud is republished for injection (s) |
| `ugv_cloud_topic` | `/pass_y/output` | Filtered UGV point cloud input |

Lower `fusion_period` for faster map sharing at higher CPU cost.

---

## Tier 2 — YAML configuration

All loaded by `uav_ugv_coordinator.launch`; no rebuild required.

### `autonomous_flight/cfg/dynamic_exploration/exploration_param.yaml` → namespace `DEP`

This is the roadmap and sensing model — the file you will touch most.

| Line | Parameter | Default | Meaning |
|---|---|---|---|
| 3–4 | `local_region_min/max` | `[-5,-5,-2]` / `[5,5,2]` | Local sampling box around the UAV |
| 5–6 | `global_region_min/max` | `[-20,-20,0.7]` / `[20,20,2.8]` | **Global sampling bounds — must enclose the world.** The `z` floor of 0.7 keeps nodes above ground; the ceiling caps flight height. |
| 7 | `local_sample_thresh` | `200` | Consecutive local-sampling failures before switching strategy |
| 8 | `global_sample_thresh` | `100` | Same, for global sampling |
| 9 | `frontier_sample_thresh` | `100` | Same, for frontier-biased sampling |
| 10 | `dist_thresh` | `1.2` | Minimum spacing between PRM nodes (m) — paper's `dmin` |
| 11 | `safe_distance_xy` | `0.7` | Horizontal obstacle clearance for node validity (m) |
| 12 | `safe_distance_z` | `0.4` | Vertical clearance (m) |
| 13 | `safe_distance_check_unknown` | `true` | Treat unknown space as unsafe — conservative but safer |
| 17–18 | `horizontal_FOV`, `vertical_FOV` | `1.57`, `1.57` | UAV camera FOV (rad) = 90° × 90°, matching Table I |
| 19–20 | `dmin`, `dmax` | `0.3`, `2.0` | Camera depth range used for information-gain raycasts (m) |
| 22 | `nearest_neighbor_number` | `15` | k for k-NN roadmap connection |
| 24 | `max_connect_dist` | `3.0` | Maximum PRM edge length (m) — paper's `dmax` |
| 25 | `num_yaw_angles` | `32` | Yaw bins evaluated per node for information gain |
| 28–30 | `min_voxel_thresh`, `min/max_goal_candidates` | `0.5`, `5`, `10` | Goal candidate filtering |

Sparser roadmap (faster, coarser): raise `dist_thresh`, lower
`nearest_neighbor_number`. Denser (slower, better coverage): the reverse.

### `autonomous_flight/cfg/dynamic_exploration/flight_base.yaml` → namespace `autonomous_flight`

| Line | Parameter | Default | Meaning |
|---|---|---|---|
| 1 | `use_fake_detector` | `true` | Use the ground-truth obstacle detector; the learned YOLO detector is not on the critical path |
| 2 | `takeoff_height` | `1.4` | Hover altitude after takeoff (m) |
| 3 | `desired_velocity` | `3.0` | Commanded UAV cruise speed (m/s) |
| 4 | `desired_acceleration` | `1.0` | (m/s²) |
| 5 | `desired_angular_velocity` | `1.0` | Yaw rate (rad/s) |
| 8 | `replan_time_for_dynamic_obstacles` | `0.3` | Replan latency on dynamic obstacles (s) |
| 10 | `reach_goal_distance` | `0.1` | Waypoint arrival tolerance (m) |

> `desired_velocity` is the *commanded* speed. The paper's `v_uav = 0.25 m/s` is
> the *effective* average over a full tour, which is much lower because the UAV
> decelerates and rotates at every viewpoint.

### `autonomous_flight/cfg/dynamic_exploration/mapping_param.yaml` → namespace `/dynamic_map`

Occupancy map resolution (`0.1 m`, the paper's `r`), map extent, local update
range, and raycast limits. Enlarge the map extent together with
`global_region_min/max` when moving to a bigger world.

### UGV navigation — `src/scout_control/params/`

| File | Key values |
|---|---|
| `move_base_params.yaml` | `controller_frequency 20.0`, `planner_frequency 10.0`, `clearing_rotation_allowed false` |
| `teb_local_planner_params.yaml` | `max_vel_x 3.0`, `min_obstacle_dist 0.5`, `yaw_goal_tolerance 3.14` (yaw unconstrained), homotopy planning on |
| `dwa_local_planner_params.yaml` | `max_vel_x 3.0`, `sim_time 2.5`, `controller_frequency 2.0` |
| `global_costmap_params.yaml` | 40×40 m @ 0.1 m, static map, `inflation_radius 0.6` |
| `local_costmap_params.yaml` | 20×20 m @ 0.1 m, rolling window, `inflation_radius 0.6` |
| `costmap_common_params.yaml` | Polygonal footprint (~0.59×0.54 m), `footprint_padding 0.05` |

`yaw_goal_tolerance 3.14` is deliberate: the rendezvous waypoint's heading is
set by the coordinator, so the local planner must not fight it.

---

## Tier 3 — Compile-time constants

These are **not** ROS parameters. Editing them requires `catkin_make`.

### `global_planner/include/global_planner/dep.cpp`

| Line | Constant | Default | Meaning |
|---|---|---|---|
| **363** | **`total_voxels`** | `1240000` | **Denominator of the coverage percentage. Must match the world — see below.** |
| 351 | `exploration_percent` | `97.0` | Coverage at which the mission terminates (%) |
| 403 | `total_info_gain < 150` | `150` | Secondary stop: remaining information gain too low |
| 595 | `lambda` | `0.001` | Discount on UGV travel time in target scoring. Matches the paper's `λ`. Larger → UAV targets strongly biased toward where the UGV can easily reach |
| 596 | `ugv_avg_speed` | `1.0` | Assumed UGV speed when scoring targets (m/s) |
| 616 | `info_gain <= 10` | `10` | Candidate rejection floor |
| 771–774 | `opp.GRASP_ITER / ELITE_SET_SIZE / ALPHA` | `40`, `10`, `0.30` | UAV orienteering solver effort |
| 774 | `opp.T_MAX` | `timeBudgetSec_` | UAV tour budget |
| 857–859 | `ugv_opp.*` | `40`, `10`, `0.2` | UGV solver effort |
| 870 | `ugv_opp.T_MAX` | `max(20.0, min_time*1.2)` | UGV tour budget → the paper's ≈20 m effective budget |

Target scoring is `score = info_gain · exp(-λ · t_ugv_path)`, hard-gated on three
UGV feasibility tests: the UAV node must have a ground twin, the vertical
corridor between them must be collision-free, and the twin must be reachable in
the ground graph.

### `global_planner/include/global_planner/dep.h`

| Line | Constant | Default | Meaning |
|---|---|---|---|
| 127 | `uavCruiseSpeed_` | `1.0` | Speed used to build the OP time matrix (m/s) |
| 128 | `timeBudgetSec_` | `30.0` | **UAV energy budget.** With `uavCruiseSpeed_ = 1.0` this is the paper's 30 m effective UAV-OP distance budget |
| 204 | `z_proj_ugv_` | `0.30` | Ground projection plane (`dep.cpp` adds `+0.4` when projecting) |
| 206 | `ugvMaxRange_` | `5.0` | UGV LiDAR range for ground information gain (m) — Table I |
| 207 | `ugvVerticalFOV_` | `0.52` | Declared but unused; `calculateUnknownUGV` uses `verticalFOV_` |
| 208 | `ugvSensorHeight_` | `0.2` | Velodyne height above the ground node (m) |

> **Modelling the UAV's energy budget** is `timeBudgetSec_`. It is the single
> most important knob in the paper's formulation: it bounds each tour and
> therefore forces the rendezvous cadence.

---

## Switching environments

Changing `wname` alone is **not sufficient**. Three things must agree:

1. **`wname`** — the Gazebo world.
2. **`total_voxels`** (`dep.cpp:363`) — the voxel count of the explorable volume
   at 0.1 m resolution. Wrong here and coverage percentages (and therefore the
   97% termination test) are meaningless. The shipped alternatives:

   ```cpp
   int total_voxels = 1240000;                  // 20 x 20 x 3.1 m   (active)
   // int total_voxels = 1440000;               // 20 x 25 x 3.1 m
   // int total_voxels = 15.5*15*3.1*1000;      // 15 x 15 x 3.1 m  (room)
   ```

   Exactly one line must be uncommented. For a new world, compute
   `width × depth × height / r³` with `r = 0.1`.

3. **`global_region_min/max`** (`exploration_param.yaml:5–6`) — must enclose the
   world, or the PRM will sample outside it or fail to cover it.

Then rebuild, because `total_voxels` is compile-time:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

> Promoting `total_voxels` to a ROS parameter would remove this rebuild step and
> is a worthwhile follow-up.

---

## Correspondence with Table I of the paper

| Paper symbol | Value | Where in the code |
|---|---|---|
| `r` (voxel resolution) | 0.1 m | `mapping_param.yaml` |
| UAV horizontal / vertical FOV | 90° / 90° | `exploration_param.yaml:17–18` (1.57 rad) |
| UAV sensing range | 2.0 m | `exploration_param.yaml:20` (`dmax`) |
| UGV sensing range | 5.0 m | `dep.h:206` (`ugvMaxRange_`) |
| UGV horizontal FOV | 360° | Velodyne VLP-16 model |
| `τ_a` (max flight time) | 120 s | `dep.h:128` as a 30 s / 30 m OP budget (see note) |
| `λ` (utility weight) | 0.001 | `dep.cpp:595` |
| `d_min` (min node spacing) | 1.2 m | `exploration_param.yaml:10` (`dist_thresh`) |
| `d_max` (max node spacing) | 3.0 m | `exploration_param.yaml:24` (`max_connect_dist`) |

> **On `τ_a`:** the paper reports the physical flight-time limit (120 s) and
> derives an *effective UAV-OP distance budget of 30 m* from `τ_a · v_uav`. The
> code encodes that derived budget directly: `timeBudgetSec_ = 30.0` with
> `uavCruiseSpeed_ = 1.0 m/s` gives the same 30 m. Likewise the UGV's
> `max(20.0, …)` budget corresponds to the paper's ≈20 m.
