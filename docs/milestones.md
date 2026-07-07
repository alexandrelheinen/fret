# FRET Milestones

> **Scope:** Milestones 1–5 established the algorithm stack on a SCARA bootstrap robot
> with Gazebo as the engineering simulator. Milestones 6–7 and the v1.0 release add
> a **MuJoCo visual backend** and a **showcase scenario** (robot + environment TBD).
>
> **v1.0 target:** ARCO planning + Gazebo engineering SITL + MuJoCo visual showcase.
> See [docs/v1.0.md](v1.0.md) and
> [docs/reports/simulation-platform-study-2026-q3.md](reports/simulation-platform-study-2026-q3.md).
>
> **MS-1–5 stop condition (met):** SCARA arm plans and executes collision-aware motion
> using ARCO occupancy and a workspace map (pillar scenario validated in pure-Python CI).

---

## Requirements by Milestone — Completion Status

This table maps every functional requirement from [docs/requirements.md](docs/requirements.md)
to the milestone that validates it and its current completion status.

### System-Level

| ID | Description (abbreviated) | Milestone | Status |
|---|---|---|---|
| FR-SYS-01 | Multiple robot models selectable at launch (`model:=`) | MS-1 | ✅ Done |
| FR-SYS-02 | Scenario YAML fully specifies a reproducible run | MS-1 | ✅ Done |
| FR-SYS-03 | SITL and HITL modes | MS-1 (SITL) / Phase 3 (HITL) | ✅ SITL done |
| FR-SYS-04 | All configurable values as ROS 2 parameters or YAML | MS-1 | ✅ Done |

### Scene Acquisition

| ID | Description (abbreviated) | Milestone | Status |
|---|---|---|---|
| FR-SCN-01 | Subscribe to world geometry on `/world_state` | MS-3 | ✅ Done |
| FR-SCN-02 | Transform all point cloud data to `world` frame | MS-3 | ✅ Done |
| FR-SCN-03 | Maintain `KDTreeOccupancy` from obstacle cloud | MS-2 | ✅ Done |
| FR-SCN-04 | Update occupancy without blocking planning/control | MS-3 | ✅ Done |

### Planning

| ID | Description (abbreviated) | Milestone | Status |
|---|---|---|---|
| FR-PLN-01 | Collision-free path from `q_start` to `q_goal` | MS-2 | ✅ Done |
| FR-PLN-02 | C-space planning; FK → KDTreeOccupancy check | MS-2 | ✅ Done |
| FR-PLN-03 | Action feedback: iteration count, cost, time | MS-2 | ✅ Done |
| FR-PLN-04 | Planning timeout ≤ 30 s → `ABORTED / TIMEOUT` | MS-2 | ✅ Done |
| FR-PLN-05 | `ABORTED` on failure; no auto-retry | MS-2 | ✅ Done |
| FR-PLN-06 | Post-processing: prune → optimize → B-spline | MS-2 | ✅ Done |
| FR-PLN-07 | Reject invalid configurations before planning | MS-2 | ✅ Done |

### Control

| ID | Description (abbreviated) | Milestone | Status |
|---|---|---|---|
| FR-CTL-01 | Controller at 50 Hz | MS-1 | ✅ Done |
| FR-CTL-02 | EE tracking error ≤ 5 mm in SITL | MS-1, MS-3 | ✅ Done |
| FR-CTL-03 | Jacobian pseudoinverse velocity commands | MS-1 | ✅ Done |
| FR-CTL-04 | FK → TF2 broadcast at 50 Hz | MS-1 | ✅ Done |
| FR-CTL-05 | Controller independent of planning node | MS-1 | ✅ Done |
| FR-CTL-06 | Fault detection → HALTED state after 20 mm / 0.5 s | MS-1 | ✅ Done |

### Hardware (Phase 3)

| ID | Description (abbreviated) | Milestone | Status |
|---|---|---|---|
| FR-HW-01 | Relay `/joint_commands` to Arduino via Micro-ROS | Phase 3 | 🔲 Not started |
| FR-HW-02 | Publish encoder feedback to `/joint_states` ≥ 50 Hz | Phase 3 | 🔲 Not started |
| FR-HW-03 | Validate message integrity | Phase 3 | 🔲 Not started |

---

## Component Status

| Component | Location | Status |
|---|---|---|
| SCARA URDF/XACRO model | `src/fret/urdf/scara.xacro` | ✅ Done |
| Kinematics engine (FK, IK, Jacobian) | `src/fret/control/kinematics.py` | ✅ Done — 19 unit tests |
| StateEstimator (joint states → RobotState + TF2) | `src/fret/control/state_estimator.py` | ✅ Done |
| ControllerNode (FSM, Jacobian tracking, Level 3 + 4) | `src/fret/control/controller_node.py` | ✅ Done — Level 3 + 4 |
| SceneAcquisition (point cloud → OccupancyUpdatePayload) | `src/fret/scene/acquisition.py` | ✅ Done |
| OccupancyAdapter (ARCO KDTree bridge) | `src/fret/scene/occupancy_adapter.py` | ✅ Done |
| WorkspaceOccupancyBuilder (20 cm voxel grid) | `src/fret/scene/workspace_occupancy.py` | ✅ Done — 20 tests |
| PerceptionBridgeNode (obstacle cloud → /obstacle_cloud) | `src/fret/ros/perception_bridge.py` | ✅ Done |
| PlannerNode (pure-Python planning core, Level 3) | `src/fret/planning/planner_node.py` | ✅ Done |
| PlannerNodeRos (ROS 2 Action server, Level 4) | `src/fret/planning/planner_node_ros.py` | ✅ Done |
| CSpaceChecker (FK + KDTreeOccupancy) | `src/fret/planning/cspace_checker.py` | ✅ Done |
| TrajectoryGenerator (pruner + optimizer + B-spline) | `src/fret/planning/trajectory_generator.py` | ✅ Done |
| ReplanningManager (replanning FSM) | `src/fret/planning/replanning_manager.py` | ✅ Done — 76 tests |
| TrajectoryConverter (trapezoidal profiles) | `src/fret/planning/trajectory_converter.py` | ✅ Done — 94 tests |
| Validation / quality gates | `src/fret/validation/` | ✅ Done — 71 tests |
| Launch files (view, sim, sitl, hardware) | `src/fret/launch/` | ✅ Done |
| Scenario YAMLs (SC-01 through SC-05) | `src/fret/config/scenarios/` | ✅ Done |
| Architecture, interface, requirements docs | `docs/` | ✅ Done |
| BridgeNode (hardware serial bridge) | `src/fret/hardware/bridge_node.py` | 🔲 Stub — Phase 3 |
| Pillar scenario SDF and YAML (MS-5) | `src/fret/worlds/pillar_scenario.sdf` | ✅ Done |

---

## Milestone 1 — SCARA straight-line tracking via ROS IK control

**Goal:** The SCARA end-effector moves in a straight horizontal line for 3 seconds
at low speed (constant z, EE error < 5 mm).  No ARCO involved — the trajectory is
analytically generated and the controller uses only ROS 2 topics and the FRET
Kinematics engine.  This milestone validates the control stack in isolation.

### Acceptance criteria — ✅ All passed

1. `ros2 launch fret sitl.py scenario:=straight_line model:=scara` starts without
   errors; Gazebo and RViz open.
2. The SCARA EE moves from `[0.0, 0.0, 0.10]` to `[0.785, 0.0, 0.10]` over 3 s
   (constant z = 0.10 m, i.e. `q3` unchanged).
3. `/joint_commands` is published at ≥ 45 Hz for the full duration.
4. Maximum EE position error ≤ 5 mm at every timestep (FR-CTL-02).
5. No `/fault` message is published.

### Implementation summary

- `ControllerNode` (Level 3 + 4) with 50 Hz timer, FSM, and Jacobian tracking.
- `ControllerRosNode` subclasses `rclpy.node.Node` with full ROS wiring.
- `StraightLineInjector` node reads `straight_line.yml` and publishes once.
- `sitl.py` routes `scenario:=straight_line` to the injector (no planner).
- **CI:** `pytest tests/simulation/test_scenario_straight_line.py` passes.

### Key files

| File | Role |
|---|---|
| `src/fret/control/controller_node.py` | ControllerNode (L3) + ControllerRosNode (L4) |
| `src/fret/ros/straight_line_injector.py` | Trajectory injector |
| `src/fret/config/scenarios/straight_line.yml` | Scenario config |
| `tests/control/test_controller_node*.py` | Unit + ROS tests |
| `tests/simulation/test_scenario_straight_line.py` | Simulation test |

---

## Milestone 2 — ARCO planning pipeline publishing trajectory to a topic

**Goal:** The ARCO planner finds a collision-free C-space path from the start to the
goal configuration in an **empty world**, converts it to a `JointTrajectory` message,
and publishes it on `/joint_trajectory`.  The controller is **not** connected yet — the
trajectory is published but not executed.  This isolates the planning pipeline for
validation before wiring it to the control stack.

> Key insight from the problem statement: *"all the planning is done at the start and
> only the IK control is used to further track the state sequence published by the
> planning module."*  Milestone 2 validates the planning half; Milestone 3 connects them.

### Acceptance criteria — ✅ All passed

1. `ros2 launch fret sitl.py scenario:=static_reach model:=scara` starts without errors.
2. The `PlannerNode` Action server accepts the goal from the scenario YAML and calls
   the collision predicate.
3. Planning returns `SUCCESS` within 30 s.
4. The resulting `JointTrajectory` is published on `/joint_trajectory`; it has ≥ 2
   waypoints.
5. The Action result carries status `SUCCEEDED`.
6. No `/fault` message; no unhandled exception.

### Implementation summary

- `PlannerNode` (L3) and `PlannerNodeRos` (L4 Action server) fully implemented.
- `CSpaceChecker` (FK + KDTreeOccupancy) and `TrajectoryGenerator` implemented.
- ARCO optional: linear-interpolation fallback when ARCO absent.
- **CI:** `pytest tests/planning/` and integration tests for static reach pass.

### Key files

| File | Role |
|---|---|
| `src/fret/planning/planner_node.py` | PlannerNode (L3) |
| `src/fret/planning/planner_node_ros.py` | PlannerNodeRos (L4 Action server) |
| `src/fret/planning/cspace_checker.py` | FK + occupancy collision checker |
| `src/fret/planning/trajectory_generator.py` | Post-processing chain |
| `tests/planning/test_planner_node*.py` | Unit + ROS tests |

---

## Milestone 3 — Full end-to-end: ARCO trajectory executed by the controller

**Goal:** Replace the hardcoded straight line from Milestone 1 with the ARCO-planned
trajectory from Milestone 2.  The world is empty (no obstacles).  The simulation runs
for 20 seconds.  The path is not hardcoded — it is computed by ARCO at startup and
then tracked by the Jacobian controller.

### Acceptance criteria — ✅ All passed

1. `ros2 launch fret sitl.py scenario:=static_reach model:=scara` starts without errors.
2. At t = 0 s: `PlannerNode` computes the trajectory and publishes it on `/joint_trajectory`.
3. `ControllerNode` transitions from `IDLE` to `TRACKING` at ≥ 45 Hz.
4. The SCARA EE reaches the goal within 20 s.
5. Maximum EE position error ≤ 5 mm at every timestep (FR-CTL-02).
6. No `/fault` message is published throughout the run.
7. Scenario SC-01 (Static Reach) passes end-to-end.

### Implementation summary

- `sitl.py` routes planner → controller via `/joint_trajectory` (no injector).
- `PerceptionBridgeNode` feeds `/obstacle_cloud` → `SceneAcquisition` → `CSpaceChecker`.
- **CI:** `tests/integration/test_scenario_static_reach_full.py` and controller
  tracking tests pass (max EE error **0.56 mm**, limit 5 mm).

### Key files

| File | Role |
|---|---|
| `src/fret/launch/sitl.py` | Full pipeline launcher |
| `src/fret/config/scenarios/static_reach.yml` | Scenario config |
| `tests/control/test_controller_node_tracking.py` | Tracking unit tests |
| `tests/integration/test_scenario_static_reach_full.py` | End-to-end test |
| `tests/integration/test_scenario_static_reach_full.py` | End-to-end integration test |

---

## Milestone 4 — 3-D workspace occupancy from Gazebo + matplotlib validation

**Goal:** Build a dense voxel-grid occupancy map of the full SCARA reachable workspace
(20 cm resolution, 147 voxels) and validate it with matplotlib and a programmatic query.

### SCARA workspace bounds

| Axis | Lower | Upper | # cells at 20 cm |
|---|---|---|---|
| X | −0.60 m | +0.60 m | 7 |
| Y | −0.60 m | +0.60 m | 7 |
| Z | 0.00 m | +0.40 m | 3 |

Total: **147 voxels**. Only voxels within `0.05 m ≤ r ≤ 0.60 m` are evaluated.

### Acceptance criteria — ✅ All passed

1. `pytest tests/scene/test_workspace_occupancy.py` passes.
2. Matplotlib figure shows occupied (red) vs. free (grey) voxels as a 3-D scatter.
3. `is_occupied(x, y, z)` returns correct values for known points.
4. Unit test coverage for `WorkspaceOccupancyBuilder` ≥ 90 %.

### Implementation summary

- `WorkspaceOccupancyBuilder` pure-Python class in `src/fret/scene/workspace_occupancy.py`.
- **CI:** `pytest tests/scene/test_workspace_occupancy.py` passes (19 occupied
  voxels, 59 free voxels in golden configuration).

### Key files

| File | Role |
|---|---|
| `src/fret/scene/workspace_occupancy.py` | WorkspaceOccupancyBuilder |
| `tests/scene/test_workspace_occupancy.py` | Unit tests |

---

## Milestone 5 — Two-pillar world and autonomous collision-aware motion at constant z

**Status: ✅ Done (pure-Python pipeline)**

**Goal:** Place two cylindrical pillar obstacles in the Gazebo world, run the full
planning pipeline, and have the SCARA arm plan and execute a collision-free
path at **constant z**, autonomously avoiding the pillars.

### World definition

| Pillar | Centre (world frame) | Radius | Height |
|---|---|---|---|
| `pillar_a` | `(0.25, 0.10, 0.20)` | 0.04 m | 0.40 m |
| `pillar_b` | `(−0.15, 0.30, 0.20)` | 0.04 m | 0.40 m |

### Motion parameters

| Parameter | Value |
|---|---|
| Start | `[0.0, 0.0, 0.10]` (home — EE at (0.60, 0, 0.138)) |
| Goal | `[-1.0, 0.5, 0.10]` (EE at (0.417, −0.405, 0.138)) |
| Constant z | `q3 = 0.10 m` throughout |
| Algorithm | `WorkspaceOccupancyBuilder` (20 cm grid) + fallback linear planner |
| Timeout | 30 s |

### Acceptance criteria

1. `ros2 launch fret sitl.py scenario:=pillar_avoidance model:=scara` starts;
   both pillars are visible in Gazebo and RViz.
2. `WorkspaceOccupancyBuilder` correctly marks voxels overlapping the pillars as occupied.
3. `PlannerNode` finds a collision-free path within 30 s; no waypoint within 0.05 m of either pillar.
4. `ControllerNode` tracks the trajectory; EE error ≤ 5 mm.
5. EE reaches the goal within the trajectory duration.
6. No `/fault` message is published.
7. ROS bag contains `/joint_states`, `/joint_commands`, `/joint_trajectory`, `/obstacle_cloud`.

### Deliverables (completed)

- ✅ `src/fret/worlds/pillar_scenario.sdf` — two-pillar Gazebo world
- ✅ `src/fret/config/scenarios/pillar_avoidance.yml` — scenario YAML
- ✅ `src/fret/config/perception.yaml` — cylinder obstacle entries added
- ✅ `PerceptionBridgeNode` already supports cylinder surface sampling
- ✅ `tests/scene/test_workspace_occupancy_pillars.py` — pillar occupancy unit tests
- ✅ `tests/integration/test_scenario_pillar_avoidance.py` — integration tests
- ✅ MS-5 covered by `tests/integration/test_scenario_pillar_avoidance.py` in CI

---

## Milestone sequence

```
MS-1  ✅  ControllerNode + straight-line tracking (3 s)
  │
  ▼
MS-2  ✅  Planning pipeline → /joint_trajectory (empty world)
  │
  ▼
MS-3  ✅  Full SITL — planned trajectory executed by Jacobian controller (20 s)
  │
  ▼
MS-4  ✅  Workspace occupancy map (20-cm grid, matplotlib validation)
  │
  ▼
MS-5  ✅  Pillar world + autonomous collision-aware motion at constant z
  │
  ▼
MS-6  🔲  MuJoCo visual backend (MJCF model, launch, render pipeline)
  │
  ▼
MS-7  🔲  v1.0 showcase scenario (robot + environment — design session TBD)
  │
  ▼
v1.0.0    Polished demo + article-ready assets
```

---

## Milestone 6 — MuJoCo visual backend

**Status: 🔲 Not started**

**Goal:** Add MuJoCo as a second simulator backend behind the same pure-Python FRET
pipeline. MuJoCo provides smooth physics and polished rendering for demos, portfolio
video, and the planned 2026 Q3 article on robotics simulation.

### Acceptance criteria

1. MJCF model exists for the bootstrap SCARA (migrates with the v1.0 showcase robot).
2. `ros2 launch fret mujoco.py` (or `sitl.py backend:=mujoco`) starts without errors.
3. The same scenario YAML that drives Gazebo also drives MuJoCo (joint I/O only differs).
4. Controller tracks a planned trajectory with EE error ≤ 5 mm in MuJoCo.
5. A headless render produces a PNG or short MP4 artifact suitable for README/article.
6. Unit or integration test validates the MuJoCo backend adapter (mocked or headless).

### Implementation tasks

| ID | Task | File(s) |
|---|---|---|
| T-601 | Evaluate `mujoco` Python bindings vs ROS bridge | `docs/v1.0.md` |
| T-602 | Create MJCF model | `src/fret/mjcf/scara.xml` (new) |
| T-603 | Implement joint I/O adapter | `src/fret/ros/mujoco_bridge.py` (new) |
| T-604 | Add launch file + `backend:=` selector | `src/fret/launch/mujoco.py`, `sitl.py` |
| T-605 | Headless render → CI artifact | `scripts/render_mujoco.py` (new) |
| T-606 | Gazebo vs MuJoCo benchmark on SC-01 | `docs/reports/` or `benchmark.yaml` |

### Key files (planned)

| File | Role |
|---|---|
| `src/fret/mjcf/scara.xml` | MuJoCo robot + scene model |
| `src/fret/ros/mujoco_bridge.py` | Simulator backend adapter |
| `src/fret/launch/mujoco.py` | MuJoCo launch entry point |

---

## Milestone 7 — v1.0 showcase scenario

**Status: 🔲 Not started — design session required**

**Goal:** Define and implement the **v1.0 showcase scenario** — the robot topology,
environment, and narrative that best present FRET + ARCO to an external audience.
The SCARA was the bootstrap; the showcase robot may differ.

### Open design questions

1. Robot: SCARA, 6-DOF arm, or delta topology?
2. Environment: pillars (proven), pick-and-place, or richer scene?
3. Narrative: academic benchmark vs. cinematic MuJoCo video?

### Acceptance criteria

1. SC-v1 scenario YAML defined with documented pass criteria.
2. End-to-end run passes on **Gazebo** (engineering validation).
3. End-to-end run passes on **MuJoCo** (visual showcase).
4. ARCO SST active (not linear fallback) in the showcase run.
5. Article-ready assets: screenshot, short video, benchmark table.
6. All v1.0 release gates in [docs/v1.0.md](v1.0.md) satisfied.
7. Tag `v1.0.0`.

### Implementation tasks

| ID | Task |
|---|---|
| T-701 | Design session: select robot + environment |
| T-702 | Define `config/scenarios/v1_showcase.yml` (SC-v1) |
| T-703 | Port or create robot model (URDF + MJCF) |
| T-704 | Run SC-v1 on Gazebo and MuJoCo |
| T-705 | Write article section + README demo assets |
| T-706 | Tag `v1.0.0` |

---

## Known Risks for Milestone 5

| Risk | Mitigation |
|---|---|
| ARCO not installed in CI | `try/except ImportError`; ARCO-dependent tests skip via `pytest.importorskip` |
| 20-cm grid too coarse for 4-cm pillar radii | Voxel circumradius ≈ 17 cm threshold; add 1-voxel safety margin |
| ARCO SST may time out | Ensure pillar centres ≥ 0.30 m from start/goal FK positions |
| Constant-z constraint vs. ARCO SST sampler | Set SST bounding box `q3_lo = q3_hi = 0.10` |

---

## References

- v1.0 goals and release gates: [`docs/v1.0.md`](v1.0.md)
- Platform study (2026 Q3): [`docs/reports/simulation-platform-study-2026-q3.md`](reports/simulation-platform-study-2026-q3.md)
- Scenario library and pass criteria: [`docs/scenarios.md`](scenarios.md)
- Interface contracts and QoS profiles: [`docs/interfaces.md`](interfaces.md)
- Architecture and data-flow diagrams: [`docs/architecture.md`](architecture.md)
- ARCO integration details and boundary: [`docs/arco.md`](arco.md)
- Coding standards: [`docs/guidelines.md`](guidelines.md)
- Development workflow (SDD, V-cycle): [`CONTRIBUTING.md`](../CONTRIBUTING.md)
- Simulation tutorial (A-Z): [`docs/simulation.md`](simulation.md)

