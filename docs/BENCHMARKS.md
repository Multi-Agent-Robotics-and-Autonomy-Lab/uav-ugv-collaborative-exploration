# Reproducing the Benchmarks

The paper compares the proposed method against a single-UAV ablation and
reports per-environment exploration metrics. This document maps those
experimental arms onto the code.

---

## Selecting a planner variant

All three planner variants implement `globalPlanner::DEP` against the same
`dep.h`, so they are drop-in replacements. The variant is selected at configure
time by `global_planner/CMakeLists.txt`:

```bash
# Proposed method (default)
catkin_make -DCMAKE_BUILD_TYPE=Release -DDEP_VARIANT=dep.cpp

# Single-UAV ablation
catkin_make -DCMAKE_BUILD_TYPE=Release -DDEP_VARIANT=dep_uav.cpp

# Baseline arm
catkin_make -DCMAKE_BUILD_TYPE=Release -DDEP_VARIANT=dep_benchmarking.cpp
```

The configure step prints which one was selected:

```
-- global_planner: building with DEP_VARIANT=dep.cpp
```

Because the variant is compiled into `libglobal_planner.so`, **switching
variants requires a rebuild.** Remember to rebuild back to `dep.cpp` before
running the main demo.

| `DEP_VARIANT` | Arm |
|---|---|
| `dep.cpp` | **Ours** — coupled air–ground roadmap, coupled OPs, UGV as mobile charging station |
| `dep_uav.cpp` | **Single-UAV ablation** — UGV removed; the UAV keeps the same effective distance budget but must return to a static charging station at the origin after each tour |
| `dep_benchmarking.cpp` | **Baseline arm** used for the benchmark comparisons |

---

## Coordinator variants

Two coordinator sources exist:

| File | Coordination model |
|---|---|
| `uav_ugv_coordinator.cpp` | **Simultaneous** — both robots execute their tours at the same time (`EXECUTE_DUAL_PATHS`). This is the proposed method. |
| `uav_ugv_coordinator_benchmarking.cpp` | **Sequential** — the UAV rides on the UGV, the UGV drives its leg, then the UAV takes off and flies its leg (`INITIAL_LAND → EXECUTE_UGV_PATH → UAV_TAKEOFF → EXECUTE_UAV_PATH`). |

Only `uav_ugv_coordinator.cpp` is wired into `CMakeLists.txt`. To run the
sequential arm, add a second executable in
`uav_ugv_coordinator/CMakeLists.txt`:

```cmake
add_executable(uav_ugv_coordinator_benchmarking src/uav_ugv_coordinator_benchmarking.cpp)
target_link_libraries(uav_ugv_coordinator_benchmarking ${catkin_LIBRARIES})
add_dependencies(uav_ugv_coordinator_benchmarking
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

then point the `type` field of the node in `uav_ugv_coordinator.launch` at it.

A matching UAV-only flight stack exists as
`autonomous_flight/include/autonomous_flight/simulation/dynamicExploration_uav_bench.{cpp,h}`,
likewise not in its `CMakeLists.txt`.

---

## Environments

The paper uses three environments of differing size and complexity. Worlds are
in `uav_simulator/worlds/floorplan3/`:

| World | Notes |
|---|---|
| `construction_site_big_v` | Default. |
| `construction_site_big` | Larger construction yard variant. |
| `heterogenous_arena` | Designed so neither robot alone suffices — a tunnel the UAV cannot fly through, elevated scaffolding the UGV cannot climb. |
| `heterogenous_large_arena` | Larger variant of the above. |
| `custom_arena` | Arena with a passage tunnel. |
| `simple_maze` | Simple maze layout. |

For each environment you **must** set `total_voxels` (`dep.cpp:363`) and the
`global_region_min/max` bounds to match, or the coverage metric is invalid. See
[PARAMETERS.md](PARAMETERS.md#switching-environments).

```cpp
int total_voxels = 1240000;                  // 20 x 20 x 3.1 m   (active)
// int total_voxels = 1440000;               // 20 x 25 x 3.1 m
// int total_voxels = 15.5*15*3.1*1000;      // 15 x 15 x 3.1 m  (room)
```

---

## Metrics the planner reports

`dep.cpp` instruments the run and logs to the terminal:

| Metric | Meaning |
|---|---|
| Coverage % | `explored_voxels / total_voxels` |
| `t_80%`, `t_95%` | Time to reach 80% and 95% exploration |
| Total computation time | Accumulated planning time |
| Number of tours | Tours until termination |

Termination is at **97% coverage** (`exploration_percent`, `dep.cpp:351`) or
when total remaining information gain drops below 150 (`dep.cpp:403`). The
paper additionally caps each mission at 30 minutes and averages over 5 runs per
environment.

Milestones are timestamped relative to `exploration_start_time_` and announced
when crossed, and `/exploration/complete` latches at the end:

```bash
rostopic echo /exploration/complete
```

---

## Suggested procedure

For each (arm, environment) pair:

1. Set `total_voxels` and `global_region_min/max` for the environment.
2. `catkin_make -DCMAKE_BUILD_TYPE=Release -DDEP_VARIANT=<variant>`
3. `source setup_env.sh`
4. `roslaunch uav_simulator try.launch wname:=<world>` — press ▶ in Gazebo.
5. `roslaunch map_manager map_fusion.launch`
6. `roslaunch uav_ugv_coordinator uav_ugv_coordinator.launch`
7. Record the terminal metrics; repeat for 5 seeds and report mean ± std.

Record a bag if you want to re-derive metrics offline:

```bash
rosbag record /exploration/complete /uav/op_path /ugv/op_path \
              /CERLAB/quadcopter/odom /ground_truth/state
```
