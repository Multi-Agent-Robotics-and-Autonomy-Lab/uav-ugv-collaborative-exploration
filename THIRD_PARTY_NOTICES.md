# Third-Party Notices

This repository is a self-contained ROS catkin workspace. To make the system
reproducible from a single `git clone`, it **vendors** (includes copies of)
software written by others. Those components remain under their own licenses,
which are reproduced in their respective subdirectories. The MIT license in
`LICENSE` covers only the work of this paper's authors.

---

## 1. CERLAB-UAV-Autonomy — the framework this work builds on

Nearly everything under `src/CERLAB-UAV-Autonomy/` originates with
**Zhefan Xu and the CERLAB group at Carnegie Mellon University**.

- Upstream: https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy
- License: MIT (`src/CERLAB-UAV-Autonomy/LICENSE`, Copyright (c) 2023 Zhefan Xu)
- Vendored from commit `e045ce55b4d06b4e04c1c2c7ef066a54368d36a9`

This framework provides the UAV model and Gazebo simulator, the occupancy
mapping stack, the B-spline trajectory planner, the tracking controller, and the
incremental PRM exploration planner that our method extends. It is reference
**[4]** in the paper:

> Z. Xu, C. Suzuki, X. Zhan, and K. Shimada, "Heuristic-based incremental
> probabilistic roadmap for efficient UAV exploration in dynamic environments."

Upstream is organised as nine git submodules. They were vendored flat (their
`.git` directories removed) so that this workspace clones and builds in one
step. The upstream commit each was taken from:

| Package | Upstream | Commit |
|---|---|---|
| `autonomous_flight` | https://github.com/Zhefan-Xu/autonomous_flight | `e18ca9d7` (branch `simulation`) |
| `global_planner` | https://github.com/Zhefan-Xu/global_planner | `2afc6e3c` |
| `map_manager` | https://github.com/Zhefan-Xu/map_manager | `d8112b85` |
| `onboard_detector` | https://github.com/Zhefan-Xu/onboard_detector | `4b4be99e` |
| `remote_control` | https://github.com/Zhefan-Xu/remote_control | `7e66ca4a` |
| `time_optimizer` | https://github.com/Zhefan-Xu/time_optimizer | `0854be30` |
| `tracking_controller` | https://github.com/Zhefan-Xu/tracking_controller | `7f8f5d77` |
| `trajectory_planner` | https://github.com/Zhefan-Xu/trajectory_planner | `a84b4914` |
| `uav_simulator` | https://github.com/Zhefan-Xu/uav_simulator | `80d7a4a1` |

Six of these carry our modifications. See `docs/ARCHITECTURE.md` for what was
changed and why. `onboard_detector`, `time_optimizer` and `tracking_controller`
are unmodified.

---

## 2. MOSEK — proprietary, NOT covered by the MIT license

`src/CERLAB-UAV-Autonomy/time_optimizer/include/time_optimizer/third_party/`
contains **MOSEK**, a commercial optimization solver, as pre-compiled binaries:

```
third_party/include/mosek.h
third_party/lib/x86/libmosek64.so.10.0   (~29 MB)
third_party/lib/ARM/libmosek64.so.10.1   (~15 MB)
third_party/lib/{x86,ARM}/libmosekxx*.so, libmosekjava*.so
```

- Copyright © **MOSEK ApS** — https://www.mosek.com
- These files are redistributed here as they appear in the upstream
  CERLAB-UAV-Autonomy repository, solely so that `time_optimizer` (a build
  dependency of `autonomous_flight` and `trajectory_planner`) compiles.
- **They are governed by MOSEK's own license terms, not by the MIT license in
  `LICENSE` and not by the MIT license of the surrounding `time_optimizer`
  package.** MOSEK requires a valid license file at runtime; academic licenses
  are available free of charge from MOSEK ApS.
- If you intend to redistribute this repository further, or to use it
  commercially, review MOSEK's terms first.

The `time_optimizer` module is **not on the critical path** for the exploration
experiments in this paper; it is present because the upstream build graph
requires it.

---

## 3. AgileX Scout Mini packages

`src/scout_description/`, `src/scout_gazebo_sim/` and `src/scout_control/`
describe and simulate the **AgileX Scout Mini** UGV. They derive from the
AgileX / Weston Robot `scout_ros` family, which in turn derives from Clearpath
Robotics' Husky description.

- `scout_description` — BSD. `package.xml` credits the Clearpath Robotics Husky
  authors: Ryan Gariepy, Prasenjit Mukherjee, Mike Purvis, Paul Bovbel.
- `scout_gazebo_sim` — BSD. Authored by Ruixiang Du (Weston Robot).
- `scout_control` — parameter sets only (move_base, costmaps, TEB/DWA). The
  upstream copy carried no license tag; it is marked BSD-3-Clause here to match
  its sibling packages. The planner and costmap values have been retuned for
  the experiments in this paper.

The BSD text these packages require was not shipped upstream; it is reproduced
in `src/scout_description/LICENSE`, `src/scout_gazebo_sim/LICENSE` and
`src/scout_control/LICENSE`.

Modifications made for this work: a landing platform was added to the Scout Mini
model (`urdf/mini.xacro`) so the UAV can touch down on the UGV, and a Velodyne
VLP-16 was mounted.

---

## 4. Other bundled components

- **OSQP** — vendored inside
  `src/CERLAB-UAV-Autonomy/trajectory_planner/include/trajectory_planner/third_party/osqp/`.
  Apache-2.0. https://osqp.org
- **velodyne_simulator** (`velodyne_description`, `velodyne_gazebo_plugins`) —
  *not* vendored; installed from apt. BSD. Required for the Scout Mini URDF to
  parse.
- **YOLO detector weights** —
  `src/CERLAB-UAV-Autonomy/onboard_detector/scripts/yolo_detector/weights/`
  ship with upstream and carry no stated provenance. They are not used by the
  exploration pipeline, which runs with `use_fake_detector: true`.
