# FRET Project Roadmap

## Project goals

* **High-Level Controller:** Raspberry Pi 5 running Linux (Ubuntu) with the high level task planning, motion planning, navigation, and control real-time processes.
* **Middleware:** ROS 2 (Jazzy) for logic, kinematics, and communication.
* **Low-Level Controller:** Arduino Mega for real-time actuation and signal conversion to physical layer for the initial prototyping. To be reviewed and replaced by a dedicated driver afterwards.
* **Communication:** Serial-based bridge Micro-ROS for data exchange between RPi 5 and Arduino.
* **Motion planning:** **ARCO** (author-owned Python library) for C-space sampling-based planning, occupancy, and trajectory post-processing.
* **Simulation — engineering backend:** **Gazebo Harmonic** for ROS-native SITL, CI validation, and the hardware path.
* **Simulation — visual showcase:** **MuJoCo** for polished demos, article figures, and portfolio video (v1.0 target).
* **Bootstrap robot:** A SCARA (RRP, 3-DOF) was used for Phases 1–2 and Milestones 1–5 because it was already validated on ARCO. The **v1.0 showcase robot and environment** will be selected in a dedicated design session — see [docs/v1.0.md](v1.0.md).
* **Physical target topology:** A delta-like robot (4 arms) is the intended physical prototype. The topology will be finalized at Phase 4 based on procurement constraints.

> The current system specification must be kept updated at the main project [README file](README.md) as decisions are made.
>
> **Platform study (2026 Q3):** [docs/reports/simulation-platform-study-2026-q3.md](reports/simulation-platform-study-2026-q3.md)

## Setup

The development was done on Ubuntu 24.04 under WSL on Windows 10.

## Technical Roadmap

---

### Phase 0 — Specification Completion ✅ *Complete*

All items in this phase are prerequisites for writing any implementation code.
They correspond to V-cycle Levels 1 and 2. See [CONTRIBUTING.md](../CONTRIBUTING.md)
for the full SDD and V-cycle workflow.

**Level 1 — Functional specifications and validation plan**

- [x] Decide on planning domain: C-space (joint configuration space). Recorded in
  [docs/architecture.md](architecture.md).
- [x] Write functional requirements (FR-xx register). See [docs/requirements.md](requirements.md).
- [x] Define operational envelope: SCARA joint limits, workspace volume, performance bounds
  (planning < 30 s, control 50 Hz, EE error < 5 mm). See [docs/requirements.md](requirements.md).
- [x] Define failure mode policy: `ABORTED` with structured error code, no auto-retry.
  See [docs/interfaces.md](interfaces.md).
- [x] Write scenario library for functional validation (SC-01 through SC-05).
  See [docs/scenarios.md](scenarios.md).

**Level 2 — Architecture and pipeline definition**

- [x] Define interface contracts for all cross-module data structures
  (`OccupancyUpdatePayload`, `RobotState`, `PlanningRequest`, `PlanningResult`).
  See [docs/interfaces.md](interfaces.md).
- [x] Define ROS 2 Action message format: `PlanRequest.action`.
  See [docs/interfaces.md](interfaces.md).
- [x] Assign QoS profiles to every topic. See [docs/interfaces.md](interfaces.md).
- [x] Define node state machines (`PlannerNode`, `ControllerNode`) with full transition
  tables. See [docs/interfaces.md](interfaces.md).
- [x] Document error propagation paths end-to-end. See [docs/interfaces.md](interfaces.md).
- [x] Pin ARCO dependency mechanism: `pip install -e ../arco/` (or git dep in `pyproject.toml`).
  See [docs/interfaces.md](interfaces.md).

**CI/CD gates**

- [x] `formatting.yml` — Black, isort, clang-format on every PR.
- [x] `tests.yml` — build, unit tests (pytest, ≥ 90% coverage), smoke tests.
- [x] `type_check.yml` — mypy strict mode on `src/`.
- [x] `integration.yml` — launch_testing inter-node scenario tests.

---

### Phase 1 — SITL Setup ✅ *Complete*

- [x] Configure the development environment on Linux (scripts verified: `install.sh`, `setup.sh`, `build.sh`).
- [x] Validate the existing SCARA URDF (`src/fret/urdf/scara.xacro`) in RViz via `ros2 launch fret view.py model:=scara`.
- [x] Launch the simulation environment (Gazebo + RViz) via `ros2 launch fret sim.py model:=scara`.
- [x] Implement module stubs and unit tests for `scene/`, `planning/`, `control/` (Level 3).
- [x] Fill stubs: implement `scene/acquisition.py` and `scene/occupancy_adapter.py` (Level 4).
- [x] Verify scenario SC-01 (Static Reach) passes end-to-end in SITL (pure-Python).

---

### Phase 2 — Trajectory Control ✅ *Complete*

- [x] Implement the kinematics engine (`control/kinematics.py`): FK, analytical IK for SCARA, Jacobian.
- [x] Implement the Jacobian-based velocity controller (`control/controller_node.py`).
- [x] Execute predefined trajectories (straight lines, arcs) in the simulator — scenario SC-04, SC-05.
- [x] Implement feedback correction to reject model disturbances and validate EE error < 5 mm — scenario SC-01.
- [x] Implement ARCO planning pipeline (MS-2) and full end-to-end SITL (MS-3, MS-4, MS-5).
- [x] Verify scenario SC-01 (Static Reach), SC-04 (Straight Line), SC-05 (Arc) pass.

---

### Phase 3 — HITL Setup

- [ ] Establish the serial communication link between RPi 5 and Arduino Mega (Micro-ROS).
- [ ] Implement `hardware/bridge_node.py`: relay commands, receive encoder feedback.
- [ ] Validate the control stack end-to-end with the Arduino (without active motor drivers).
- [ ] Test telemetry loopback: verify data integrity and measure round-trip latency.

> Status: TODO — deferred past v1.0.

---

### Phase 4 — Hardware Procurement

- [ ] Define and purchase remaining mechanical and electronic components. This step finalizes
  the actual physical robot topology (expected: delta-like, 4 arms).
- [ ] Select actuators (e.g., Nema 17) and drivers (e.g., TMC series).
- [ ] Extend the simulation stack to reproduce the physical prototype: new URDF, new MJCF,
  new scenario YAMLs.

> Status: TODO — deferred past v1.0.

---

### Phase 5 — v1.0 Release (ARCO + Gazebo + MuJoCo) 🔄 *In progress*

This phase consolidates the platform study decisions into a shippable v1.0 release.
See [docs/v1.0.md](v1.0.md) for acceptance criteria and task list.

**Phase 5a — Close Gazebo engineering gaps (MS-5 completion)**

- [x] Wire ARCO's planning pipeline into the SITL stack (SST planner, C-space checker).
- [x] Implement pillar-avoidance scenario (MS-5 pure-Python pipeline).
- [ ] Validate pillar avoidance in live Gazebo SITL (not just pure-Python).
- [ ] Add Gazebo headless smoke test to CI.
- [ ] Wire `ReplanningManager` into the live ROS pipeline.
- [ ] Add ARCO-enabled CI job with real SST (not linear fallback).

**Phase 5b — MuJoCo visual backend (MS-6)**

- [ ] Evaluate MuJoCo integration path (`mujoco` Python bindings vs ROS bridge).
- [ ] Create MJCF model for bootstrap robot (SCARA; migrate with v1.0 showcase).
- [ ] Implement simulator backend adapter in `fret.ros` (joint I/O abstraction).
- [ ] Add `launch/mujoco.py` and `backend:=` selector on `sitl.py`.
- [ ] Produce headless render artifacts (PNG/video) for CI and article.

**Phase 5c — v1.0 showcase scenario (MS-7)**

- [ ] Design session: select robot topology and environment for v1.0 demo.
- [ ] Define SC-v1 scenario YAML with pass criteria.
- [ ] Run end-to-end on both Gazebo and MuJoCo backends.
- [ ] Record article-ready assets (screenshot, short video, benchmark table).
- [ ] Tag `v1.0.0`.

> **Post-v1.0 research (not blocking):** OMPL side-by-side benchmark; Isaac Sim evaluation.

---

### Phase 6 — Physical Prototype Assembly

- [ ] Finalize the mechanical build.
- [ ] Calibrate low-level firmware constants against physical hardware.
- [ ] Execute first real-world trajectories.
- [ ] Decide whether to replace the Arduino with a dedicated embedded controller.

> Status: TODO

---

### Phase 7 — Vision Integration

- [ ] Close the loop with optical sensors (PiCam or Webcam).
- [ ] Implement object detection and real-time replanning based on visual feedback.
- [ ] Transition from known static targets to autonomous object manipulation.

> Status: To be defined
