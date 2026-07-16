# FRET Functional Requirements

Requirements trace to [releases.md](releases.md) and [scenarios.md](scenarios.md).

Format: `FR-<LAYER>-<NN>`

Layers: `SYS`, `SCN`, `PLN`, `CTL`, `GSP` (grasp), `SIM`, `HW`

---

## System-Level

**FR-SYS-01:** Multiple robot models selectable via `model:=` at launch.

**FR-SYS-02:** A scenario YAML fully specifies a reproducible run.

**FR-SYS-03:** Simulation and SITL backends:

| Backend | Role | From |
|---|---|---|
| MuJoCo | Physics, contacts, rendering, SITL | v1.0 |
| HITL | Hardware | post v1.4 |

**FR-SYS-04:** All runtime-significant values in ROS parameters or YAML.

---

## Scene Acquisition

**FR-SCN-01:** Subscribe to obstacle geometry as `sensor_msgs/PointCloud2` on `/obstacle_cloud`.

**FR-SCN-02:** Transform all obstacle data to `world` frame before ARCO.

**FR-SCN-03:** Maintain `KDTreeOccupancy` from obstacle point cloud.

**FR-SCN-04:** Occupancy updates shall not block planning or control loops.

---

## Planning

**FR-PLN-01:** Collision-free C-space path from `q_start` to `q_goal`.

**FR-PLN-02:** C-space planning with FK → KDTree clearance (or direct C-space for PPP).

**FR-PLN-03:** Action feedback: iteration count, cost, elapsed time.

**FR-PLN-04:** Planning timeout 30 s (60 s for v1.4 6-DOF) → `ABORTED / TIMEOUT`.

**FR-PLN-05:** Failure → `ABORTED` with error code; no auto-retry.

**FR-PLN-06:** Post-process: prune → optimize → B-spline (when ARCO available).

**FR-PLN-07:** Reject out-of-envelope start/goal before planning.

---

## Control

**FR-CTL-01:** Trajectory tracking at 50 Hz.

**FR-CTL-02:** Position tracking error limits:

| Release | Robot | Limit |
|---|---|---|
| v1.0 | PPP gantry | ≤ 10 mm EE |
| v1.1 | Dubins | ≤ 0.5 m pose (SE(2)) |
| v1.3 | RRP | ≤ 5 mm EE |
| v1.4 | 6-DOF | ≤ 5 mm EE |

**FR-CTL-03:** Velocity commands published to `/joint_commands`.

**FR-CTL-04:** FK → TF2 broadcast `base_link → tool0` at control rate.

**FR-CTL-05:** Controller independent of planner (async start on trajectory).

**FR-CTL-06:** Fault on sustained tracking error → HALTED + `/fault`.

---

## Grasp (v1.0+)

**FR-GSP-01:** v1.0 uses **magnetic weld** grasp only (no finger DOF).

**FR-GSP-02:** Welded cargo included in planner collision predicate during TRANSPORT.

**FR-GSP-03:** Weld releases at goal; cargo remains at goal pose.

**FR-GSP-04:** Grasp FSM states: IDLE, APPROACH, CAPTURE, TRANSPORT, RELEASE.

---

## Simulation (MuJoCo)

**FR-SIM-01:** MuJoCo is the sole simulator for SITL, physics, contacts, and rendering.

**FR-SIM-02:** Algorithm layers simulator-agnostic; I/O in `fret.ros` only.

**FR-SIM-03:** MuJoCo headless MP4 render for CI and release artifacts.

**FR-SIM-04:** MJCF assets under `src/fret/mjcf/` with joint names matching controller configs.

**FR-SIM-05:** `MuJoCoBridgeNode` publishes `/joint_states` at 50 Hz from simulated state.

**FR-SIM-06:** Kinematic mirror mode (v1.0–v1.1): integrate commands in Python, sync
`qpos` + `mj_forward` for visuals and collision geometry.

**FR-SIM-07:** Physics mode (v1.2+): controller commands drive actuators; advance with
`mj_step`; `/joint_states` from simulated `qpos`/`qvel` — no pose teleportation.

**FR-SIM-08:** Contact forces resolved by MuJoCo during physics mode; contact logging
available for CI regression.

**FR-SIM-09:** `physics_mode` selectable via ROS parameter or scenario YAML.

**FR-SIM-10:** PPP collision checking via `CSpaceCheckerMujoco` (MJCF geometry + contacts).

**FR-SIM-11:** Minimal unit-robot MJCF sandboxes (`ppp_unit`, `diffdrive_unit`)
shall respond to open-loop actuator commands under pure `mj_step` physics without
pose teleportation, warehouse/race orchestration, or post-step velocity surgery.
Optional tests under `tests/simulation/` validate basic motions; they need not
gate every PR.

Full integration specification: [mujoco.md](mujoco.md).
Unit / external model repertoire: [mujoco_models_benchmark.md](mujoco_models_benchmark.md).

---

## Hardware (post v1.4)

**FR-HW-01:** Relay `/joint_commands` to Arduino via Micro-ROS.

**FR-HW-02:** Publish encoder feedback on `/joint_states` ≥ 50 Hz.

**FR-HW-03:** Validate message integrity before actuation.

---

## Operational envelopes by release

### v1.0 — PPP

| Parameter | Value |
|---|---|
| Joints | 3 prismatic (x, y, z) |
| Workspace | [0,60] × [0,20] × [0,6] m (full); 12×4×3 m MJCF preview |
| Cargo | 0.5 m box, magnetic weld |
| Planner | ARCO RRT* (MuJoCo collision) |
| Sim mode | Kinematic mirror |

### v1.1 — Dubins

| Parameter | Value |
|---|---|
| State | (x, y, θ) SE(2) |
| Agents | 2 |
| Planner | ARCO SST per agent |
| Control | ARCO Pure Pursuit |
| Sim mode | Kinematic mirror |

### v1.2 — Physics SITL

| Parameter | Value |
|---|---|
| Scope | PPP + Dubins scenarios |
| Sim mode | MuJoCo `mj_step` + actuators |
| Control rate | 50 Hz |

### v1.3 — RRP

| Parameter | Value |
|---|---|
| Joints | 2 revolute + 1 prismatic |
| Planner | ARCO SST |
| Control | Jacobian pseudoinverse |
| Sim mode | Physics SITL |

### v1.4 — 6-DOF

| Parameter | Value |
|---|---|
| Joints | 6 revolute |
| Planner | ARCO SST |
| Planning timeout | 60 s |
| Sim mode | Physics SITL |

---

## Validation mapping

| Requirement | Release | Validated by |
|---|---|---|
| FR-SYS-01–04 | all | Launch smoke tests |
| FR-SCN-01–04 | v1.0+ | `tests/scene/` |
| FR-PLN-01–07 | v1.0+ | `tests/planning/`, SC-v10+ |
| FR-CTL-01–06 | v1.0+ | `tests/control/` |
| FR-GSP-01–04 | v1.0 | `tests/control/test_grasp_magnet.py` |
| FR-SIM-01–06 | v1.0–v1.1 | MuJoCo launch + MP4 artifact |
| FR-SIM-07–09 | v1.2 | Physics SITL smoke + integration tests |
| FR-SIM-10 | v1.0 | `tests/planning/test_cspace_checker_mujoco.py` |
| FR-SIM-11 | v1.2+ | `tests/simulation/test_*_robot_unit.py` |
| FR-HW-01–03 | post v1.4 | — |

### Regression (bootstrap SCARA)

MS-1–5 scenarios (SC-01 – SC-05) remain in pure-Python CI as regression tests until v1.3
supersedes them. They are **not** release acceptance criteria.
