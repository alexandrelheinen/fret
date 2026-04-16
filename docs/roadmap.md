# FRET Project Roadmap

## Project goals

* **High-Level Controller:** Raspberry Pi 5 running Linux (Ubuntu) with the high level task planning, motion planning, navigation, and control real-time processes.
* **Middleware:** ROS 2 (Jazzy) for logic, kinematics, and communication.
* **Low-Level Controller:** Arduino Mega for real-time actuation and signal conversion to physical layer for the initial prototyping. To be reviewed and replaced by a dedicated driver afterwards.
* **Communication:** Serial-based bridge Micro-ROS for data exchange between RPi 5 and Arduino.
* **SITL topology:** A SCARA robot (RRP: 2 revolute + 1 prismatic) is used for Phases 1 and 2. It is already modeled in `src/fret/urdf/scara.xacro` and exemplified by ARCO's planning primitives, making it the natural starting point.
* **Physical target topology:** A delta-like robot (4 arms) is the intended physical prototype. The topology will be finalized at Phase 4 based on procurement constraints. The simulation stack will be extended to match the physical reality at that point.

> The current system specification must be kept updated at the main project [README file](README.md) as decisions are made.

## Setup

The development was done on Ubuntu 24.04 under WSL on Windows 10.

## Technical Roadmap

---

### Phase 0 — Specification Completion *(current phase)*

All items in this phase are prerequisites for writing any implementation code.
They correspond to V-cycle Levels 1 and 2. See [docs/guidelines.md](guidelines.md)
for the full V-cycle definition.

**Level 1 — Functional specifications and validation plan**

- [x] Decide on planning domain: C-space (joint configuration space). Recorded in
  [docs/architecture.md](architecture.md).
- [x] Write functional requirements (FR-xx register). See [docs/requirements.md](requirements.md).
- [x] Define operational envelope: SCARA joint limits, workspace volume, performance bounds
  (planning < 30 s, control 50 Hz, EE error < 5 mm). See [docs/requirements.md](requirements.md).
- [x] Define failure mode policy: `ABORTED` with structured error code, no auto-retry.
  See [docs/interfaces.md](interfaces.md).
- [x] Write scenario library for functional validation (SC-01 through SC-04).
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
- [x] Pin ARCO dependency mechanism: `pip install -e ../arco/`.
  See [docs/interfaces.md](interfaces.md).

**CI/CD gates**

- [x] `formatting.yml` — Black, isort, clang-format on every PR.
- [x] `tests.yml` — build, unit tests (pytest, ≥ 90% coverage), smoke tests.
- [x] `type_check.yml` — mypy strict mode on `src/`.
- [x] `integration_tests.yml` — launch_testing inter-node scenario tests.
- [x] `release.yml` — full suite on version tags.

**Next step:** Move to Phase 1. Begin with Level 3 (module stubs + unit test files
defined simultaneously) per the V-cycle mandate.

---

### Phase 1 — SITL Setup

- [ ] Configure the development environment on Linux (scripts verified: `install.sh`, `setup.sh`, `build.sh`).
- [ ] Validate the existing SCARA URDF (`src/fret/urdf/scara.xacro`) in RViz via `ros2 launch fret view.py model:=scara`.
- [ ] Launch the simulation environment (Gazebo + RViz) via `ros2 launch fret sim.py model:=scara`.
- [ ] Implement module stubs and unit tests for `scene/`, `planning/`, `control/` (Level 3).
- [ ] Fill stubs: implement `scene/acquisition.py` and `scene/occupancy_adapter.py` (Level 4).
- [ ] Verify scenario SC-01 (Static Reach) passes end-to-end in SITL.

> Status: To be started (specification phase complete).

---

### Phase 2 — Trajectory Control

- [ ] Implement the kinematics engine (`control/kinematics.py`): FK, analytical IK for SCARA, Jacobian.
- [ ] Implement the Jacobian-based velocity controller (`control/controller_node.py`).
- [ ] Execute predefined trajectories (straight lines, arcs) in the simulator — scenario SC-04.
- [ ] Implement feedback correction to reject model disturbances and validate EE error < 5 mm — scenario SC-01.
- [ ] Verify scenario SC-02 (Obstacle Avoidance) and SC-03 (Planning Timeout) pass.

> Status: TODO

---

### Phase 3 — HITL Setup

- [ ] Establish the serial communication link between RPi 5 and Arduino Mega (Micro-ROS).
- [ ] Implement `hardware/bridge_node.py`: relay commands, receive encoder feedback.
- [ ] Validate the control stack end-to-end with the Arduino (without active motor drivers).
- [ ] Test telemetry loopback: verify data integrity and measure round-trip latency.

> Status: TODO

---

### Phase 4 — Hardware Procurement

- [ ] Define and purchase remaining mechanical and electronic components. This step finalizes
  the actual physical robot topology (expected: delta-like, 4 arms).
- [ ] Select actuators (e.g., Nema 17) and drivers (e.g., TMC series).
- [ ] Extend the simulation stack to reproduce the physical prototype: new URDF, new kinematics,
  new scenario YAMLs.

> Status: TODO

---

### Phase 5 — Motion Planner Integration

- [ ] Wire ARCO's full planning pipeline into the SITL stack (ARCO SST planner, C-space checker).
- [ ] Implement replanning triggers and scene-update loop.
- [ ] Deploy the full SITL pipeline with dynamic goal specification.
- [ ] Validate in both SITL and HITL setups.

> Status: TODO

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