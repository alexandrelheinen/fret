# FRET Functional Requirements

Requirements trace to [releases.md](releases.md) and [scenarios.md](scenarios.md).

Format: `FR-<LAYER>-<NN>`

Layers: `SYS`, `SCN`, `PLN`, `CTL`, `SIM`, `VIS`, `HW`

---

## System-Level

**FR-SYS-01:** Multiple robot models selectable via `model:=` at launch.

**FR-SYS-02:** A scenario YAML fully specifies a reproducible run.

**FR-SYS-03:** Simulation and deployment backends:

| Backend | Role | From |
|---|---|---|
| MuJoCo | Physics, contacts, cameras, rendering, SITL | v1.1 |
| HITL | Physical hardware | **v2.x** (not v1.3) |

**FR-SYS-04:** All runtime-significant values in ROS parameters or YAML.

**FR-SYS-05:** Computer vision applies only to **manipulator** models that
interact with graspable objects. Dubins / TB3 scenarios shall not depend on
ball vision.

---

## Scene Acquisition

**FR-SCN-01:** Subscribe to obstacle geometry as `sensor_msgs/PointCloud2` on `/obstacle_cloud`.

**FR-SCN-02:** Transform all obstacle data to `world` frame before ARCO.

**FR-SCN-03:** Maintain `KDTreeOccupancy` from obstacle point cloud.

**FR-SCN-04:** Occupancy updates shall not block planning or control loops.

---

## Planning

**FR-PLN-01:** Collision-free C-space path from `q_start` to `q_goal`.

**FR-PLN-02:** C-space planning with FK → KDTree clearance (or SE(2) occupancy for Dubins).

**FR-PLN-03:** Action feedback: iteration count, cost, elapsed time.

**FR-PLN-04:** Planning timeout 30 s (60 s for v1.2.4 6-DOF) → `ABORTED / TIMEOUT`.

**FR-PLN-05:** Failure → `ABORTED` with error code; no auto-retry.

**FR-PLN-06:** Post-process: prune → optimize → B-spline (when ARCO available).

**FR-PLN-07:** Reject out-of-envelope start/goal before planning.

---

## Control

**FR-CTL-01:** Trajectory tracking at 50 Hz.

**FR-CTL-02:** Position tracking error limits:

| Release | Robot | Limit |
|---|---|---|
| v1.1 | Dubins | ≤ 0.5 m pose (SE(2)) |
| v1.2.3 | OpenMANIPULATOR-X | ≤ 5 mm EE |
| v1.2.4 | 6-DOF | ≤ 5 mm EE |

**FR-CTL-03:** Velocity commands published to `/joint_commands`.

**FR-CTL-04:** FK → TF2 broadcast `base_link → tool0` at control rate.

**FR-CTL-05:** Controller independent of planner (async start on trajectory).

**FR-CTL-06:** Fault on sustained tracking error → HALTED + `/fault`.

---

## Simulation (MuJoCo)

**FR-SIM-01:** MuJoCo is the sole simulator for SITL, physics, contacts, and rendering.

**FR-SIM-02:** Algorithm layers simulator-agnostic; I/O in `fret.ros` only.

**FR-SIM-03:** MuJoCo headless MP4 render for CI and release artifacts.

**FR-SIM-04:** MJCF assets under `src/fret/mjcf/` with joint names matching controller configs.

**FR-SIM-05:** `MuJoCoBridgeNode` publishes `/joint_states` at 50 Hz from simulated state.

**FR-SIM-06:** Kinematic mirror mode (v1.1): integrate commands in Python, sync
`qpos` + `mj_forward` for visuals and collision geometry.

**FR-SIM-07:** Physics mode (v1.2+): controller commands drive actuators; advance with
`mj_step`; `/joint_states` from simulated `qpos`/`qvel` — no pose teleportation.

**FR-SIM-08:** Contact forces resolved by MuJoCo during physics mode; contact logging
available for CI regression.

**FR-SIM-09:** `physics_mode` selectable via ROS parameter or scenario YAML.

**FR-SIM-11:** Minimal unit-robot MJCF sandboxes (`diffdrive_unit`, TurtleBot3 /
unit-style bases) shall respond to open-loop actuator commands under pure
`mj_step` physics without pose teleportation, race orchestration, or post-step
velocity surgery. Optional tests under `tests/simulation/` validate basic
motions; they need not gate every PR.

**FR-SIM-12:** Opt-in time-series telemetry of simulation/control state shall
export a PlotJuggler-compatible CSV under `/tmp/fret_telemetry/`, using the
series naming grammar `agent.quantity_frame.component` defined in
[modules/telemetry.md](modules/telemetry.md). Contact JSONL (FR-SIM-08) remains
a separate path.

**FR-SIM-13:** (v1.4+) Manipulator CV scenarios shall expose calibrated MuJoCo
`<camera>` sensors for the vision pipeline (distinct from showcase overview
cameras).

Full integration specification: [mujoco.md](mujoco.md).
Telemetry module specification: [modules/telemetry.md](modules/telemetry.md).
Unit / external model repertoire: [mujoco_models_benchmark.md](mujoco_models_benchmark.md).

---

## Computer vision (v1.3–v1.5)

Program overview: [vision/README.md](vision/README.md).

**FR-VIS-01:** A pure-Python package `fret.vision` shall detect a graspable ball
in one or more camera frames without importing ROS.

**FR-VIS-02:** The pipeline shall accept `N ≥ 1` frames and emit at most one
primary `BallObservation` per call (plus diagnostics).

**FR-VIS-03:** v1.3 shall record an algorithm selection (candidates + metrics +
chosen primary) under `docs/vision/algorithm-selection.md`.

**FR-VIS-04:** Unit tests shall gate fixture centre accuracy for the chosen
primary algorithm (thresholds in the selection doc).

**FR-VIS-05:** (v1.4+) Release manipulator pick-place paths shall use vision for
**ball** pose; they shall not use hardcoded ball / `pick_xy` as the grasp
target source of truth.

**FR-VIS-06:** Place / dispenser / container pose shall be supplied as known
scenario parameters (not required from CV).

**FR-VIS-07:** (v1.5) The system shall classify detections as pickable or not
before starting grasp.

**FR-VIS-08:** (v1.5) Dynamic delivery: ball rest pose shall not be the scripted
pick target; vision must observe the settled ball.

---

## Hardware (v2.x)

**FR-HW-01:** Relay `/joint_commands` to Arduino via Micro-ROS.

**FR-HW-02:** Publish encoder feedback on `/joint_states` ≥ 50 Hz.

**FR-HW-03:** Validate message integrity before actuation.

---

## Operational envelopes by release

### v1.1 — Dubins

| Parameter | Value |
|---|---|
| State | (x, y, θ) SE(2) |
| Agents | 2 |
| Planner | ARCO SST per agent |
| Control | ARCO Pure Pursuit / path-following MPC |
| Sim mode | Kinematic mirror (physics from v1.2) |
| CV | None (case study only) |

### v1.2 — Physics SITL

| Parameter | Value |
|---|---|
| Scope | Dubins race scenario |
| Sim mode | MuJoCo `mj_step` + actuators |
| Control rate | 50 Hz |

### v1.2.3 — OpenMANIPULATOR-X

Tabletop A→B, pick-and-place FSM, desk clutter, Γ-wall maze — Menagerie OM-X.
Ball pose still hardcoded until v1.4.

### v1.2.4 — 6-DOF (OMY)

| Parameter | Value |
|---|---|
| Joints | 6 revolute |
| Planner | ARCO RRT* / SST |
| Planning timeout | 60 s |
| Sim mode | Physics SITL |
| CV | None until v1.4 |

### v1.3 — Vision pipeline

Algorithms + unit tests + selection; no manipulation closed loop required.

### v1.4 — CV integration

Manipulator cells with MuJoCo cameras; ball from vision; place parametric.

### v1.5 — Dynamic industrial cell

Rolling ball, pickability, improved container; single-ball cadence provisional.

---

## Validation mapping

| Requirement | Release | Validated by |
|---|---|---|
| FR-SYS-01–04 | all | Launch smoke tests |
| FR-SYS-05 | v1.3+ | Vision / scenario review |
| FR-SCN-01–04 | v1.1+ | `tests/scene/` |
| FR-PLN-01–07 | v1.1+ | `tests/planning/`, SC-v11+ |
| FR-CTL-01–06 | v1.1+ | `tests/control/` |
| FR-SIM-01–06 | v1.1 | MuJoCo launch + MP4 artifact |
| FR-SIM-07–09 | v1.2 | Physics SITL smoke + integration tests |
| FR-SIM-11 | v1.2+ | `tests/simulation/test_*_robot_unit.py` |
| FR-SIM-12 | v1.2.6+ | Telemetry tests / release upload |
| FR-SIM-13 | v1.4+ | SC-v16 MJCF + adapter tests |
| FR-VIS-01–04 | v1.3 | `tests/vision/`, selection doc |
| FR-VIS-05–06 | v1.4 | SC-v16 physics smoke |
| FR-VIS-07–08 | v1.5 | SC-v17 |
| FR-HW-01–03 | v2.x | HITL smoke (future) |
