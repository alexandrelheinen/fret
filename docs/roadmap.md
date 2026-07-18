# FRET Project Roadmap

> **Release targets:** [releases.md](releases.md) (v1.0 → v1.4)

---

## Project goals

* **Product:** A ROS 2 full-stack robotics framework connecting **ARCO** planning to
  simulation and hardware.
* **Middleware:** ROS 2 Jazzy.
* **Planning:** ARCO (SST, KDTree occupancy, trajectory pruner).
* **Simulation:** MuJoCo — physics, contacts, rendering, and SITL for all releases.
* **Assets:** ROBOTIS MuJoCo Menagerie + AWS RoboMaker warehouse (git submodules).
* **Hardware path:** Raspberry Pi 5 + Arduino Mega via Micro-ROS (post v1.4).

---

## Release sequence

```
v1.1  ✅  Dubins dual-robot race A→B through warehouse maze (Menagerie TB3)
  │
  ▼
v1.2  ✅  MuJoCo physics SITL — actuators, contacts, controller tuning
  │
  ▼
v1.3  🔲  OpenMANIPULATOR-X tabletop — positional A→B (Menagerie OM-X)
  │
  ▼
v1.4  🔲  6-DOF manipulator — final challenge (Menagerie OMY / equivalent)
```

Full acceptance criteria and tasks: [releases.md](releases.md).

Robot models come **only** from the Menagerie submodule (same pattern as TurtleBot3).
Legacy SCARA/RRP bootstrap work is retired — validated historically, not a product target.

---

## Phase 0 — Specification ✅ *Complete*

- [x] C-space planning domain decided
- [x] Interface contracts (`PlanningRequest`, `OccupancyUpdatePayload`, …)
- [x] ROS 2 Action format (`PlanRequest.action`)
- [x] QoS profiles and node FSMs
- [x] CI workflows (formatting, tests, type check, integration)

---

## Phase 1 — Mobile race showcase ✅ *Complete*

**Robot:** TurtleBot3 Burger × 3 (Menagerie). **Scenario:** SC-v11 warehouse race.

- [x] SE(2) planning adapter (ARCO DubinsVehicle)
- [x] Dual/triple-agent race orchestration
- [x] AWS warehouse maze + submodule meshes
- [x] Pure Pursuit tracking + obstacle repulsion
- [x] Release workflow + showcase MP4s
- [x] Tag `v1.1.0` … physics iterations through `v1.2.0`

---

## Phase 2 — v1.2 MuJoCo physics SITL ✅ *Complete*

**Cross-cutting release.** Applies to the v1.1 Dubins showcase.

- [x] `physics_mode` + wheel actuators + contact logging
- [x] Physics integration tests
- [x] Package version 1.2.0

Full specification: [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl).

---

## Phase 3 — v1.3 OpenMANIPULATOR-X tabletop 🔲

**Robot:** OpenMANIPULATOR-X (4-DOF + gripper) from
`third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x`.
**Scenarios:** SC-v13a (empty A→B), SC-v13b (AWS desk clutter detour).

- [ ] Wire Menagerie OM-X MJCF into FRET cell (empty tabletop)
- [ ] Command chain: plan → track → MuJoCo actuators (A→B, no obstacles)
- [ ] Add AWS desk / clutter props; verify detour planning
- [ ] Tune controller / lookahead / clearance as needed
- [ ] Showcase video (overview + EE follow)
- [ ] Tag `v1.3.0`

---

## Phase 4 — v1.4 6-DOF challenge 🔲

**Robot:** Menagerie 6-DOF arm (OpenMANIPULATOR-Y preferred). **Scenario:** SC-v14.

- [ ] Select Menagerie 6-DOF model (OMY)
- [ ] Numerical IK + Jacobian controller
- [ ] 6-D C-space checker with self-collision
- [ ] Cluttered cell MJCF (AWS props)
- [ ] Tag `v1.4.0`

---

## Phase 5 — Hardware HITL 🔲 *Post v1.4*

- [ ] Micro-ROS serial bridge (`hardware/bridge_node.py`)
- [ ] Encoder feedback loop
- [ ] Physical prototype integration

---

## Phase 6 — Vision 🔲 *Future*

- [ ] Camera-based perception
- [ ] Dynamic replanning from visual feedback
