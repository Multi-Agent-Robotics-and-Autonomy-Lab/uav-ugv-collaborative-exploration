# Energy-Aware Collaborative Exploration for a UAV–UGV Team

Reference implementation and Gazebo simulation for the paper:

> **Energy-Aware Collaborative Exploration for a UAV–UGV Team**
> Cahit Ikbal Er, Saikiran Juttu, Yasin Yazıcıoğlu
> Northeastern University, Boston, MA
> [arXiv:2603.22507](https://arxiv.org/abs/2603.22507) · [paper PDF](docs/paper.pdf)

![Collaborative exploration demo](docs/media/preview.gif)

*The full-resolution demo video is attached to the
[latest release](https://github.com/juttu-s/uav-ugv-collaborative-exploration/releases/latest).*

---

## Overview

A UAV and a UGV explore an unknown environment together. The UAV's energy budget
is modelled as a **maximum flight time**, so it flies a sequence of
energy-bounded exploration tours. The UGV explores the ground simultaneously and
doubles as a **mobile charging station**: each tour ends in a rendezvous, where
the UAV lands on a platform mounted on the UGV and recharges before the next
tour.

The planner builds a **sparsely coupled air–ground roadmap** using a
density-aware layered PRM, and selects each pair of tours by solving **coupled
orienteering problems (OPs)** that maximise information gain subject to the
shared rendezvous time budget. All tours are constructed over
collision-validated roadmap edges.

Two properties make the pair genuinely complementary: the UGV can enter
passages the UAV cannot fly through, and the UAV can reach elevated structures
the UGV cannot climb. Neither robot alone achieves full coverage in the test
environments.

### How the two robots share a map

Mapping is bidirectional, but all planning is aerial — there is no second
planner on the UGV:

```
UGV Velodyne ─► PassThrough filter ─► map_fusion_node ─► occupancy map
                                                            ▲   │
gmapping /map ──────── inject2DFreeSpace ───────────────────┘   │
                                                                ▼
                                                   DEP: layered PRM
                                                   + coupled OP solve
                                                      │          │
                                              /uav/op_path  /ugv/op_path
                                                      │          │
                                                      ▼          ▼
                                          uav_ugv_coordinator FSM
                                                      │          │
                                           B-spline traj     move_base
```

The UGV contributes sensor data and odometry; the planner runs inside the UAV
process, projects the aerial roadmap to the ground, and publishes both tours.
Before injecting UGV scans, the coordinator erases the UGV's own footprint from
the map so the rover does not carve a moving phantom obstacle through the
roadmap.

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the module-level design.

---

## Requirements

- **Ubuntu 20.04** with **ROS Noetic**
- Gazebo 11 (ships with `ros-noetic-desktop-full`)
- A GPU is recommended; the paper's results were produced on an i9-14900K with
  an RTX 4090.

### Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-msgs \
  ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins \
  ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs \
  ros-noetic-vision-msgs \
  ros-noetic-velodyne-simulator ros-noetic-velodyne-description \
  ros-noetic-velodyne-gazebo-plugins \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins \
  ros-noetic-controller-manager ros-noetic-velocity-controllers \
  ros-noetic-joint-state-controller ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-state-publisher \
  ros-noetic-move-base ros-noetic-costmap-2d ros-noetic-costmap-converter \
  ros-noetic-teb-local-planner ros-noetic-dwa-local-planner \
  ros-noetic-base-local-planner \
  ros-noetic-gmapping ros-noetic-openslam-gmapping ros-noetic-fake-localization \
  ros-noetic-pointcloud-to-laserscan ros-noetic-pcl-ros ros-noetic-nodelet-core \
  ros-noetic-twist-mux ros-noetic-roslint ros-noetic-xacro ros-noetic-topic-tools \
  libeigen3-dev libpcl-dev libopencv-dev libprotobuf-dev protobuf-compiler \
  qtbase5-dev libboost-all-dev
```

`ros-noetic-velodyne-simulator` is not optional — the Scout Mini URDF will not
parse without `velodyne_description`.

---

## Build

```bash
git clone https://github.com/juttu-s/uav-ugv-collaborative-exploration.git
cd uav-ugv-collaborative-exploration
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Use **`catkin_make`**, not `catkin build` — this workspace ships a
`.catkin_tools/CATKIN_IGNORE` marker.

Then, **in every terminal you open**:

```bash
source setup_env.sh
```

`setup_env.sh` locates the workspace from its own path, sources ROS and the
devel space, and *prepends* this workspace to `GAZEBO_PLUGIN_PATH` and
`GAZEBO_MODEL_PATH`. That last part matters: if another catkin workspace is
sourced from your `~/.bashrc` and ends up first on the plugin path, Gazebo
silently loads *its* quadcopter plugin instead of this one, and the drone will
teleport to the origin no matter how often you rebuild. The script prints which
plugin directory won, so you can check.

It also deactivates any conda environment, because conda's python has no
`rospkg` and `xacro` dies with
`substitution args not supported: No module named 'rospkg'`.

---

## Run the collaborative exploration demo

Four terminals, in order. `source setup_env.sh` in each one first.

```bash
# Terminal 1 — Gazebo world, UAV, UGV, SLAM and the UGV navigation stack
roslaunch uav_simulator try.launch
#   Gazebo starts PAUSED: press the play button once both robots are visible.

# Terminal 2 — bridge UGV LiDAR into the UAV's occupancy map
roslaunch map_manager map_fusion.launch

# Terminal 3 — the mission state machine (this also starts the planner)
roslaunch uav_ugv_coordinator uav_ugv_coordinator.launch

# Terminal 4 — visualisation
roslaunch remote_control exploration_rviz.launch
```

The coordinator takes off, performs a 360° scan to seed the map, then enters the
plan → execute → rendezvous → recharge loop. Watch terminal 3 for
`[Coordinator] Requesting New Plans`, `[Coordinator] Received BOTH paths` and
`[Coordinator] Touchdown Confirmed`.

> **The coordinator replaces `dynamic_exploration_node`.** It constructs the
> `dynamicExploration` stack in its own process, so never run
> `autonomous_flight dynamic_exploration.launch` at the same time — you would
> start two exploration stacks and two tracking controllers.

> **Spawn race:** robots occasionally fail to spawn after a fresh build. If
> either is missing or mispositioned in Gazebo, `Ctrl+C` and relaunch. Two or
> three attempts is normal.

### Choosing an environment

```bash
roslaunch uav_simulator try.launch wname:=heterogenous_arena
```

Worlds live in `src/CERLAB-UAV-Autonomy/uav_simulator/worlds/floorplan3/`:
`construction_site_big_v` (default), `construction_site_big`, `custom_arena`,
`heterogenous_arena`, `heterogenous_large_arena`, `simple_maze`.

⚠️ Changing the world is **not** just a launch argument. You must also update
`total_voxels` in `dep.cpp` and the PRM sampling bounds, or coverage percentages
and sampling will be wrong. See
[`docs/PARAMETERS.md`](docs/PARAMETERS.md#switching-environments).

Other `try.launch` arguments: `planner:=teb|dwa|base`, `paused:=false`,
`gui:=false`, `debug:=true` (runs gzserver under gdb).

### Running a single robot

```bash
# UAV only
roslaunch uav_simulator start.launch
roslaunch remote_control exploration_rviz.launch
roslaunch autonomous_flight dynamic_exploration.launch

# UGV only (then use RViz "2D Nav Goal")
roslaunch scout_gazebo_sim scout_mini_playpen.launch wname:=example planner:=teb
```

---

## Changing parameters

Three tiers, in increasing order of inconvenience:

| Where | Examples | Needs rebuild? |
|---|---|---|
| Launch file args and `<param>` | `wname`, `planner`, `scan_hold_sec`, `reach_tol_xyz` | No |
| YAML under `autonomous_flight/cfg/dynamic_exploration/` | PRM bounds, node spacing, clearances, flight speeds | No |
| Constants in `dep.cpp` / `dep.h` | `total_voxels`, `lambda`, time budgets, UGV sensor model | **Yes** |

[`docs/PARAMETERS.md`](docs/PARAMETERS.md) documents every one of them, with the
file and line it lives on and its correspondence to Table I of the paper.

---

## Reproducing the benchmarks

The planner variants are drop-in replacements selected at configure time:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DDEP_VARIANT=dep_uav.cpp
```

| `DEP_VARIANT` | Arm |
|---|---|
| `dep.cpp` (default) | Proposed method — coupled air–ground PRM, coupled OPs |
| `dep_uav.cpp` | Single-UAV ablation, UGV removed |
| `dep_benchmarking.cpp` | Baseline arm used for the benchmark comparisons |

See [`docs/BENCHMARKS.md`](docs/BENCHMARKS.md).

---

## Repository layout

```
├── setup_env.sh                     environment bootstrap — source every terminal
├── docs/                            paper, architecture, parameters, benchmarks
└── src/
    ├── CERLAB-UAV-Autonomy/         UAV autonomy stack (see Acknowledgements)
    │   ├── uav_ugv_coordinator/     ★ mission FSM for the UAV–UGV team
    │   ├── global_planner/          ★ dep.cpp — layered PRM + coupled OP solver
    │   ├── map_manager/             ★ occupancy map + UGV map fusion
    │   ├── autonomous_flight/       ★ UAV flight stack
    │   ├── uav_simulator/           ★ Gazebo UAV model, plugins, worlds
    │   ├── trajectory_planner/      B-spline trajectory generation
    │   ├── tracking_controller/     UAV trajectory tracking
    │   ├── time_optimizer/          time-optimal reparameterisation
    │   ├── onboard_detector/        dynamic obstacle detection (unused here)
    │   └── remote_control/          ★ RViz configurations
    ├── scout_control/               ★ UGV move_base / costmap / TEB parameters
    ├── scout_description/           ★ Scout Mini URDF + landing platform
    └── scout_gazebo_sim/            Scout Mini Gazebo bringup
```

★ = contains work from this paper.

---

## Citation

```bibtex
@article{er2026energyaware,
  title   = {Energy-Aware Collaborative Exploration for a {UAV}--{UGV} Team},
  author  = {Er, Cahit Ikbal and Juttu, Saikiran and Yaz{\i}c{\i}o{\u{g}}lu, Yasin},
  journal = {arXiv preprint arXiv:2603.22507},
  year    = {2026}
}
```

---

## Acknowledgements

This work builds on **[CERLAB-UAV-Autonomy](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy)**
by Zhefan Xu and the CERLAB group at Carnegie Mellon University, which provides
the UAV simulator, occupancy mapping, trajectory planning and the incremental
PRM exploration planner that this work extends (reference [4] in the paper). The
UGV packages derive from the AgileX / Weston Robot `scout_ros` family, itself
derived from Clearpath Robotics' Husky description.

---

## License

MIT — see [`LICENSE`](LICENSE).

This repository vendors third-party code under its own licenses, including
**MOSEK**, a proprietary commercial solver that is *not* covered by the MIT
license. Read [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) before
redistributing.
