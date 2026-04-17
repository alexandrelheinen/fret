# FRET SITL Delivery Plan

> **Scope:** Four milestones that take the project from the existing framework to a
> complete SITL simulation where a SCARA arm executes an ARCO-planned trajectory
> in an empty-world Gazebo environment with a live ROS 2 point-cloud pipeline.
>
> **Stop condition:** end of Milestone 4 — the arm follows an ARCO trajectory in a
> 20-second Gazebo run with no hardcoded path.

---

## Current state — what the framework already provides

The following components are implemented and tested.  They form the baseline for
every milestone that follows.

| Component | Location | Status |
|---|---|---|
| SCARA URDF/XACRO model | `src/fret/urdf/scara.xacro` | ✅ Done |
| Kinematics engine (FK, IK, Jacobian) | `src/fret/control/kinematics.py` | ✅ Done — 19 unit tests |
| StateEstimator (joint states → RobotState + TF2) | `src/fret/control/state_estimator.py` | ✅ Done |
| ControllerNode logic core (FSM, Jacobian tracking) | `src/fret/control/controller_node.py` | ✅ Level 3 done; Level 4 ROS wiring pending |
| SceneAcquisition (point cloud → OccupancyUpdatePayload) | `src/fret/scene/acquisition.py` | ✅ Done |
| OccupancyAdapter (ARCO KDTree bridge) | `src/fret/scene/occupancy_adapter.py` | ✅ Done |
| PerceptionBridgeNode (obstacle cloud → /obstacle_cloud) | `src/fret/ros/perception_bridge.py` | ✅ Done |
| PlannerNode (Action server shell) | `src/fret/planning/planner_node.py` | 🔲 Stub only |
| CSpaceChecker (FK + KDTreeOccupancy) | `src/fret/planning/cspace_checker.py` | 🔲 Stub only |
| TrajectoryGenerator (pruner + optimizer + B-spline) | `src/fret/planning/trajectory_generator.py` | 🔲 Stub only |
| ReplanningManager | `src/fret/planning/replanning_manager.py` | ✅ Done — 76 tests |
| TrajectoryConverter (trapezoidal profiles) | `src/fret/planning/trajectory_converter.py` | ✅ Done — 94 tests |
| Validation / quality gates | `src/fret/validation/` | ✅ Done — 71 tests |
| Launch files (view, sim, sitl, hardware) | `src/fret/launch/` | ✅ Done (stubs launch nodes) |
| Scenario YAMLs (SC-01, SC-02, SC-03) | `src/fret/config/scenarios/` | ✅ Done (SC-04 missing) |
| Architecture, interface, requirements docs | `docs/` | ✅ Done |

---

## Milestone 1 — SCARA straight-line tracking via ROS IK control

**Goal:** The SCARA end-effector moves in a straight horizontal line for 3 seconds
at low speed (constant z, EE error < 5 mm).  No ARCO involved — the trajectory is
analytically generated and the controller uses only ROS 2 topics and the FRET
Kinematics engine.  This milestone validates the control stack in isolation.

### Acceptance criteria

1. `ros2 launch fret sitl.py scenario:=straight_line model:=scara` starts without
   errors; Gazebo and RViz open.
2. The SCARA EE moves from `[0.0, 0.0, 0.10]` to `[0.785, 0.0, 0.10]` over 3 s
   (constant z = 0.10 m, i.e. `q3` unchanged).
3. `/joint_commands` is published at ≥ 45 Hz for the full duration.
4. Maximum EE position error ≤ 5 mm at every timestep (FR-CTL-02).
5. No `/fault` message is published.
6. Integration test `tests/integration/test_scenario_straight_line.py` passes.

### Deliverables (V-cycle ordered)

**Level 1 — Spec (done above as acceptance criteria)**

**Level 2 — Architecture**

- Decide that the trajectory injector is a standalone Python node (not the planner)
  for this milestone; it reads `straight_line.yml` and publishes to `/joint_trajectory`.
- Confirm the controller reads `/joint_trajectory` and publishes `/joint_commands`
  at 50 Hz; no ARCO dependency path is exercised.

**Level 3 — Test files and stubs (write tests first)**

- `tests/integration/test_scenario_straight_line.py`:
  `launch_testing`-based test that asserts acceptance criteria 2–5.
- `tests/control/test_controller_node_ros.py`:
  Unit tests for Level 4 ROS wiring (mocked `rclpy.node.Node`).

**Level 4 — Implementation**

- Complete `ControllerNode` Level 4: subclass `rclpy.node.Node`; add timer at 50 Hz,
  subscription to `/joint_trajectory` (`trajectory_msgs/JointTrajectory`, Reliable),
  subscription to `/joint_states` via `StateEstimator`, and publisher for
  `/joint_commands` (`std_msgs/Float64MultiArray`, Best Effort).
- Register `controller_node` entry point in `CMakeLists.txt` and `pyproject.toml`.
- Add `src/fret/config/scenarios/straight_line.yml` (SC-04 schema):
  duration 3 s, 150 waypoints, linear joint interpolation from
  `[0.0, 0.0, 0.10]` to `[0.785, 0.0, 0.10]`.
- Add trajectory-injector node `src/fret/ros/straight_line_injector.py`:
  reads `straight_line.yml`, publishes `/joint_trajectory` once, then spins.
- Update `sitl.py` to include the injector when `scenario:=straight_line` and
  skip the planner node.

**Quality gates**

- `pytest tests/control/ tests/integration/test_scenario_straight_line.py -v` passes.
- `black --check src/ && isort --check-only src/` passes.
- `ros2 launch fret sitl.py scenario:=straight_line` smoke test passes.

---

## Milestone 2 — ARCO planning pipeline publishing trajectory to a topic

**Goal:** The ARCO planner finds a collision-free C-space path from the start to the
goal configuration in an **empty world**, converts it to a `JointTrajectory` message,
and publishes it on `/joint_trajectory`.  The controller is **not** connected yet — the
trajectory is published but not executed.  This isolates the planning pipeline for
validation before wiring it to the control stack.

> Key insight from the problem statement: *"all the planning is done at the start and
> only the IK control is used to further track the state sequence published by the
> planning module."*  Milestone 2 validates the planning half; Milestone 3 (not in
> scope here) would connect them.

### Acceptance criteria

1. `ros2 launch fret sitl.py scenario:=static_reach model:=scara` starts without errors.
2. The `PlannerNode` Action server accepts the goal from the scenario YAML and calls
   ARCO SST with `CSpaceChecker` as the collision predicate.
3. ARCO returns `SUCCESS` within 30 s.
4. The resulting `JointTrajectory` is published on `/joint_trajectory`; it has ≥ 2
   waypoints, is C²-smooth, and every waypoint's FK position clears all obstacles
   (empty world → trivially satisfied; prepares for SC-02).
5. The Action result carries status `SUCCEEDED`.
6. No `/fault` message; no unhandled exception.
7. Unit test coverage for `CSpaceChecker`, `TrajectoryGenerator`, and `PlannerNode`
   ≥ 90 % (all three currently stubbed).

### Deliverables (V-cycle ordered)

**Level 1 — Spec (done above as acceptance criteria)**

**Level 2 — Architecture**

- Confirm the planning pipeline topology:
  `SceneAcquisition → OccupancyAdapter → CSpaceChecker → SST planner → TrajectoryGenerator → /joint_trajectory`.
- Confirm ARCO is imported via `try/except ImportError`; `CSpaceChecker` and
  `TrajectoryGenerator` raise `NotImplementedError` when ARCO is absent.
- Confirm `PlannerNode` is a pure `rclpy.node.Node` Action server; no ROS calls
  inside the algorithm classes.

**Level 3 — Test files and stubs (write tests first)**

- `tests/planning/test_cspace_checker_impl.py`:
  parametric tests — known collision-free configs, known colliding configs, boundary.
- `tests/planning/test_trajectory_generator_impl.py`:
  smoothness test (C² check via finite differences), length reduction vs. raw path.
- `tests/planning/test_planner_node_ros.py`:
  mocked Action server — send goal, assert `SUCCEEDED` result and topic publication.
- `tests/integration/test_scenario_static_reach.py`:
  `launch_testing`-based, asserts acceptance criteria 1–6.

**Level 4 — Implementation**

- Implement `CSpaceChecker.__init__` and `is_collision_free`:
  iterate over link FK positions → query `KDTreeOccupancy.clearance()`.
- Implement `TrajectoryGenerator`:
  call ARCO `TrajectoryPruner` → `TrajectoryOptimizer` (with `Kinematics.ik` as hook)
  → `BSplineInterpolator`; pack output into `trajectory_msgs/JointTrajectory`.
- Implement `PlannerNode` Level 4:
  `rclpy.node.Node` base; register the `PlanRequest` Action server; on goal received:
  validate joint limits → call ARCO SST with `CSpaceChecker` → call
  `TrajectoryGenerator` → publish `/joint_trajectory` → return `SUCCEEDED`.
- Register `planner_node` entry point; update `scene_acquisition_node` entry point
  to ensure `SceneAcquisition` is running and connected to `PerceptionBridgeNode`.
- Register `scene_acquisition_node` executable entry point.

**Quality gates**

- `pytest tests/planning/ tests/integration/test_scenario_static_reach.py -v` passes.
- `black --check src/ && isort --check-only src/` passes.
- `ros2 launch fret sitl.py scenario:=static_reach` smoke test: `/joint_trajectory`
  appears in `ros2 topic list` within 35 s.

---

## Milestone 3 — Full end-to-end: ARCO trajectory executed by the controller

**Goal:** Replace the hardcoded straight line from Milestone 1 with the ARCO-planned
trajectory from Milestone 2.  The world is empty (no obstacles).  The simulation runs
for 20 seconds.  The path is not hardcoded — it is computed by ARCO at startup and
then tracked by the Jacobian controller.  This is the final stop condition.

### Acceptance criteria

1. `ros2 launch fret sitl.py scenario:=static_reach model:=scara` starts without errors.
2. At t = 0 s: `PlannerNode` computes the trajectory via ARCO and publishes it on
   `/joint_trajectory` (same as Milestone 2).
3. At trajectory receipt: `ControllerNode` transitions from `IDLE` to `TRACKING` and
   begins publishing `/joint_commands` at ≥ 45 Hz.
4. The SCARA EE reaches the goal configuration within the trajectory duration (≤ 20 s).
5. Maximum EE position error ≤ 5 mm at every timestep (FR-CTL-02).
6. No `/fault` message is published throughout the run.
7. Scenario SC-01 (Static Reach) passes end-to-end (see `docs/scenarios.md`).
8. The ROS bag recorded by `scenario.recording: true` contains `/joint_states`,
   `/joint_commands`, and `/joint_trajectory` with no gaps.

### Deliverables (V-cycle ordered)

**Level 1 — Spec (done above as acceptance criteria)**

**Level 2 — Architecture**

- Confirm the single-trajectory-first discipline: the planner publishes once at
  startup; the controller tracks the full sequence; no replanning in this milestone.
- Confirm `sitl.py` starts nodes in the correct order: Gazebo → scene acquisition →
  planner → controller.  Use `lifecycle_manager` or `event_handler` if node startup
  ordering requires it.
- Confirm the point-cloud pipeline is live during execution: `PerceptionBridgeNode`
  publishes `/obstacle_cloud` at 1 Hz; `SceneAcquisition` converts it to an
  `OccupancyUpdatePayload` which initialises `CSpaceChecker` for ARCO.

**Level 3 — Test files and stubs (write tests first)**

- `tests/integration/test_scenario_static_reach_full.py`:
  end-to-end `launch_testing` test asserting acceptance criteria 1–7 with a 25 s
  timeout.
- `tests/control/test_controller_node_tracking.py`:
  unit test — inject a pre-computed trajectory, assert joint-command sequence matches
  Jacobian kinematics prediction within tolerance.

**Level 4 — Implementation**

- Update `sitl.py` to remove the trajectory injector from Milestone 1 and route
  `/joint_trajectory` directly from the planner to the controller (they already share
  the topic name — this is a no-op at the launch level; it is the existing design).
- Update `static_reach.yml` to set `recording.enabled: true` and `duration: 20.0`.
- Validate point-cloud pipeline end-to-end with Gazebo: verify `/obstacle_cloud`
  is published after `PerceptionBridgeNode` subscribes to `/tf`; verify
  `SceneAcquisition` correctly builds the `KDTreeOccupancy` before the first
  planning request arrives.
- Fix any timing race: the planner must not send a goal before the scene is
  initialised.  Use a `/scene_ready` latched topic or a ROS 2 service call from
  `PlannerNode` to `SceneAcquisitionNode` as the synchronisation primitive.

**Quality gates**

- `pytest tests/ -v` — all tests pass (≥ 90 % coverage).
- `black --check src/ && isort --check-only src/` passes.
- `ros2 launch fret sitl.py scenario:=static_reach` 20-second smoke test: EE
  arrives within 5 mm of the goal; ROS bag recorded; no `/fault` published.
- All GitHub workflows (`formatting.yml`, `tests.yml`, `type_check.yml`) pass on push.

---

## Milestone sequence at a glance

```
Framework ✅
    │
    ▼
MS-1: Controller Level 4 wiring + straight-line SC-04 (3 s, constant z)
    │   ∘ ControllerNode ROS wiring
    │   ∘ straight_line.yml + injector node
    │   ∘ SC-04 integration test
    ▼
MS-2: ARCO planning pipeline → /joint_trajectory (publish only, not executed)
    │   ∘ CSpaceChecker + TrajectoryGenerator implemented
    │   ∘ PlannerNode Level 4 (Action server)
    │   ∘ SC-01 planning test (no control)
    ▼
MS-3: Full SITL — ARCO trajectory executed by Jacobian controller (20 s, empty world)
        ∘ sitl.py wires planner → controller via /joint_trajectory
        ∘ Point-cloud pipeline live
        ∘ SC-01 end-to-end integration test
        ∘ ROS bag recording
```

---

## Dependency and risk notes

| Risk | Milestone | Mitigation |
|---|---|---|
| ARCO not installed in CI | MS-2, MS-3 | `try/except ImportError` pattern; ARCO-dependent tests skip via `pytest.importorskip` |
| Node startup ordering races | MS-3 | Use `/scene_ready` latched topic or ROS 2 lifecycle nodes |
| Jacobian singularity near straight-line mid-point | MS-1, MS-3 | Damped pseudo-inverse (λ = 0.01, already in ControllerNode defaults) |
| ARCO SST non-determinism | MS-2, MS-3 | Fix random seed in scenario YAML; assert path-length bound, not exact path |
| Gazebo/RViz not available in headless CI | All | `sitl.py` supports `headless:=true` argument; smoke tests run with `DISPLAY=:99` |

---

## References

- Scenario library and pass criteria: [`docs/scenarios.md`](scenarios.md)
- Interface contracts and QoS profiles: [`docs/interfaces.md`](interfaces.md)
- Architecture and data-flow diagrams: [`docs/architecture.md`](architecture.md)
- ARCO integration details and boundary: [`docs/arco.md`](arco.md)
- Coding standards and V-cycle: [`docs/guidelines.md`](guidelines.md)
- Project roadmap (phases 1–7): [`docs/roadmap.md`](roadmap.md)
