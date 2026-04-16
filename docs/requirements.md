# FRET Functional Requirements

This document lists the functional requirements for FRET at **Level 1** of the V-cycle.
Requirements are traceable to scenarios in [docs/scenarios.md](scenarios.md) and validated
by the integration test workflows in `.github/workflows/`.

Format: `FR-<LAYER>-<NN>: The system shall...`

Layers: `SYS` (system-level), `SCN` (scene acquisition), `PLN` (planning), `CTL` (control), `HW` (hardware).

---

## System-Level Requirements

**FR-SYS-01:** The system shall support multiple robot models selectable at launch time
via a `model:=` argument. The active model determines the URDF loaded, the kinematics
engine configuration, and the joint-limit operational envelope.

**FR-SYS-02:** A scenario YAML file shall fully specify a reproducible run: robot model,
world file, start configuration, goal configuration, planner parameters, and controller
gains. Running the same scenario YAML on the same software version shall produce the
same observable result.

**FR-SYS-03:** The system shall operate in two modes: SITL (Gazebo-based) and HITL
(hardware-based). Phases 1 and 2 target SITL exclusively.

**FR-SYS-04:** All runtime-significant configurable values shall be declared as ROS 2
parameters or YAML configuration files. No magic number that affects observable behavior
shall be hardcoded in source files.

### Operational Envelope — Phase 1–2 (SCARA SITL)

The following constraints define the valid operating domain. Requirements that reference
"within the operational envelope" refer to this table.

| Parameter | Constraint | Notes |
|---|---|---|
| Robot model | SCARA, 3 DOF (RRP) | joint_1 and joint_2 revolute; joint_3 prismatic |
| joint_1 range | [−π, π] rad | Shoulder revolute |
| joint_2 range | [−π/2, π/2] rad | Elbow revolute |
| joint_3 range | [0.0, 0.2] m | Vertical prismatic |
| Max revolute joint velocity | 1.57 rad/s | Per joint |
| Max prismatic joint velocity | 0.1 m/s | |
| Workspace volume | Cylinder: radius ≤ 0.5 m, height ∈ [0, 0.3 m] | Centered at base_link origin |
| Obstacle model | Static convex point clouds in world frame | Dynamic obstacles out of scope |
| Simulation environment | Gazebo (Ignition) with ROS 2 bridge | Phases 1–2 only |

---

## Scene Acquisition Requirements

**FR-SCN-01:** The system shall subscribe to world geometry published by Gazebo as a
`sensor_msgs/PointCloud2` message on the `/world_state` topic.

**FR-SCN-02:** The system shall transform all received point cloud data to the `world`
coordinate frame before passing it to the planning layer. No data in any other frame
shall cross the FRET/ARCO boundary.

**FR-SCN-03:** The scene acquisition layer shall construct and maintain a
`KDTreeOccupancy` instance from the transformed obstacle point cloud.

**FR-SCN-04:** The occupancy model shall be updated each time a new `/world_state`
message arrives, without blocking the planning or control loops.

---

## Planning Requirements

**FR-PLN-01:** The system shall compute a collision-free joint-space path from a start
configuration `q_start` to a goal configuration `q_goal`, both expressed as arrays of
joint positions in radians (revolute) or meters (prismatic).

**FR-PLN-02:** Path planning shall operate in configuration space (C-space). Collision
checking shall evaluate `FK(q)` to obtain the world-frame position of each link, then
query `KDTreeOccupancy` for clearance. Task-space planning is explicitly out of scope.

**FR-PLN-03:** The planner shall expose its progress via ROS 2 Action feedback messages
containing: current iteration count, current path cost, and elapsed planning time.

**FR-PLN-04:** Planning shall complete within 30 seconds (soft deadline). Exceeding this
deadline shall cause the Action to return `ABORTED` with error code `TIMEOUT`.

**FR-PLN-05:** When planning fails for any reason (timeout, no path found, invalid
configuration), the system shall return `ABORTED` with a structured error code and an
empty path. No automatic retry shall occur.

**FR-PLN-06:** The raw planner output path shall be post-processed before execution in
the following order: (1) redundant waypoint removal, (2) time-optimal refinement, (3)
C² smooth trajectory interpolation.

**FR-PLN-07:** The planner shall reject any start or goal configuration that violates the
operational envelope (joint limits) before invoking ARCO, and shall return `ABORTED`
with error code `INVALID_CONFIGURATION`.

---

## Control Requirements

**FR-CTL-01:** The controller node shall track a `trajectory_msgs/JointTrajectory` at a
fixed loop rate of 50 Hz.

**FR-CTL-02:** The end-effector position tracking error in world space shall remain below
5 mm during trajectory execution in SITL under nominal conditions (no disturbances,
static environment, calibrated model).

**FR-CTL-03:** Joint velocity commands shall be computed from the Jacobian pseudoinverse
of the Cartesian tracking error and published to `/joint_commands` as
`std_msgs/Float64MultiArray`.

**FR-CTL-04:** Forward kinematics shall be computed from `/joint_states` and broadcast
as a TF2 transform: `base_link → tool0`, at the same rate as the control loop.

**FR-CTL-05:** The controller node shall operate independently of the planning node. It
shall not block waiting for planning to complete and shall begin execution as soon as a
valid trajectory is received on `/joint_trajectory`.

**FR-CTL-06:** If the end-effector tracking error exceeds 20 mm for more than 0.5
consecutive seconds, the controller node shall: zero all joint velocity commands,
transition to HALTED state, and publish a fault message on `/fault`.

---

## Hardware Requirements (Phase 3 and later)

**FR-HW-01:** The hardware bridge shall relay joint velocity commands from `/joint_commands`
to an Arduino Mega via a Micro-ROS serial link.

**FR-HW-02:** The hardware bridge shall receive joint encoder feedback from the Arduino
and publish it to `/joint_states` at a rate consistent with the control loop (≥ 50 Hz).

**FR-HW-03:** The hardware bridge shall validate message integrity (checksum or frame
delimiter) before forwarding any command to the low-level actuator driver.

---

## Validation Mapping

This table traces each requirement to its validation method and V-cycle level.

| Requirement | Validated by | V-level |
|---|---|---|
| FR-SYS-01 | Smoke tests: `view.py model:=scara`, `sim.py model:=scara` | L2 |
| FR-SYS-02 | Scenario SC-01 run produces identical observable result twice | L1 |
| FR-SYS-03 | SITL smoke tests; HITL deferred to Phase 3 | L1 |
| FR-SYS-04 | `mypy` type check + code review (no literals outside config) | L3 |
| FR-SCN-01 | `tests/scene/test_acquisition.py` | L3 |
| FR-SCN-02 | `tests/scene/test_acquisition.py` frame assertion | L3 |
| FR-SCN-03 | `tests/scene/test_occupancy_adapter.py` | L3 |
| FR-SCN-04 | Integration test: occupancy update latency ≤ 100 ms | L2 |
| FR-PLN-01 | Scenario SC-01, SC-02 | L1 |
| FR-PLN-02 | `tests/planning/test_cspace_checker.py`: confirm C-space path | L3 |
| FR-PLN-03 | `tests/planning/test_planner_node.py`: feedback message fields | L3 |
| FR-PLN-04 | Scenario SC-03: ABORTED within 12 s | L1 |
| FR-PLN-05 | Scenario SC-03: no trajectory published on failure | L1 |
| FR-PLN-06 | `tests/planning/test_trajectory_generator.py` | L3 |
| FR-PLN-07 | `tests/planning/test_planner_node.py`: invalid config rejection | L3 |
| FR-CTL-01 | Scenario SC-04: command rate ≥ 45 Hz | L1 |
| FR-CTL-02 | Scenario SC-01, SC-04: EE error ≤ 5 mm | L1 |
| FR-CTL-03 | `tests/control/test_controller_node.py` | L3 |
| FR-CTL-04 | `tests/control/test_kinematics.py` + TF2 broadcast check | L3 |
| FR-CTL-05 | Integration test: controller starts before planner action completes | L2 |
| FR-CTL-06 | `tests/control/test_controller_node.py`: fault injection | L3 |
| FR-HW-01–03 | Deferred to Phase 3 | — |
