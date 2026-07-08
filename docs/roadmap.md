# FRET Project Roadmap

> **Release targets:** [releases.md](releases.md) (v1.0 → v1.3)

---

## Project goals

* **Product:** A ROS 2 full-stack robotics framework connecting **ARCO** planning to
  simulators and hardware.
* **Middleware:** ROS 2 Jazzy.
* **Planning:** ARCO (SST, KDTree occupancy, trajectory pruner).
* **Visual simulation:** MuJoCo (primary showcase for all releases).
* **Engineering SITL:** Gazebo Harmonic (arm releases v1.2+).
* **Hardware path:** Raspberry Pi 5 + Arduino Mega via Micro-ROS (post v1.3).

---

## Release sequence

```
v0.9  ✅  Bootstrap SCARA pipeline (MS-1–5, pure-Python CI)
  │
  ▼
v1.0  ✅  PPP gantry — warehouse box pick-and-place (magnetic grasp, MuJoCo)
  │
  ▼
v1.1  🔲  Dubins dual-robot race A→B through column forest
  │
  ▼
v1.1+ 🔲  MuJoCo physics SITL — actuators, contacts, controller tuning
  │
  ▼
v1.2  🔲  RRP/SCARA — reproduce ARCO rrp + rr scenarios in FRET
  │
  ▼
v1.3  🔲  6-DOF manipulator — final challenge
```

Full acceptance criteria and tasks: [releases.md](releases.md).

---

## Phase 0 — Specification ✅ *Complete*

- [x] C-space planning domain decided
- [x] Interface contracts (`PlanningRequest`, `OccupancyUpdatePayload`, …)
- [x] ROS 2 Action format (`PlanRequest.action`)
- [x] QoS profiles and node FSMs
- [x] CI workflows (formatting, tests, type check, integration)

---

## Phase 1 — Bootstrap pipeline ✅ *Complete*

Validated on SCARA (RRP, 3-DOF):

- [x] Kinematics, Jacobian controller, state estimator
- [x] Scene acquisition → ARCO KDTree occupancy
- [x] PlannerNode (SST + fallback), CSpaceChecker
- [x] Trajectory post-processing and conversion
- [x] Pure-Python CI through pillar avoidance (MS-5)
- [x] Gazebo launch files (`sim.py`, `sitl.py`) — headless backend only

---

## Phase 2 — v1.0 PPP warehouse ✅ *Complete*

**Robot:** PPP gantry (3 prismatic). **Scenario:** SC-v10 magnetic box transport.

- [x] PPP kinematics and joint-space controller
- [x] Magnetic grasp FSM (weld / release)
- [x] PPP C-space collision checker (EE + cargo envelope, MuJoCo contacts)
- [x] MuJoCo MJCF: gantry + warehouse obstacles
- [x] MuJoCo backend adapter (`backend:=mujoco`)
- [x] Scenario `ppp_warehouse.yml` + launch
- [x] Headless MP4 render for README / article (RRT* + prune + tracking)
- [x] V10-1 ROS SITL smoke test
- [ ] Tag `v1.0.0`

---

## Phase 3 — v1.1 Dubins race ✅

**Robot:** Dubins mobile × 2. **Scenario:** SC-v11 column forest race.

- [x] SE(2) planning adapter (ARCO DubinsVehicle)
- [x] Dual-agent race orchestration
- [x] Column forest world (varied heights)
- [x] Pure Pursuit tracking integration
- [x] Release workflow + R2 upload (overview + follow POVs per scenario)
- [ ] Tag `v1.1.0`

---

## Phase 3.5 — MuJoCo physics & contact validation 🔲

**Prerequisite for v1.2.** Today MuJoCo is the **visual** backend: poses are
written into MJCF joint coordinates (`mj_forward`) while motion is integrated in
pure Python (PPP joint velocity, ARCO Dubins vehicle). Collision **checking**
uses MJCF geometry for PPP planning; Dubins uses analytic/KD-tree occupancy.
**Contact forces and dynamics are not applied** during showcase or SITL runs.

**Goal:** Drive robots through MuJoCo actuators (`mj_step`), resolve contacts
(columns, obstacles, inter-agent), and compare executed motion against the
existing controller outputs so gains can be tuned against real physics.

- [ ] PPP gantry: apply prismatic actuator forces/torques from `PPPControllerNode`
      commands; validate pick-and-place contacts with cargo weld in MJCF
- [ ] Dubins race: wheel/body actuation from Pure Pursuit outputs; column and
      floor contact response; optional inter-agent blocking
- [ ] Shared harness: sim-time vs wall-time metrics, contact logging, regression
      clips when physics diverges from kinematic mirror
- [ ] Document controller tuning workflow (kinematic baseline → physics SITL)
- [ ] Tag `v1.1.1` (optional patch) once physics SITL is CI-green for PPP + Dubins

Full acceptance criteria: [releases.md](releases.md) (section TBD before v1.2
kickoff).

---

## Phase 4 — v1.2 RRP / SCARA 🔲

**Robot:** RRP (existing SCARA) + RR planar. **Scenarios:** SC-v12a, SC-v12b.

- [ ] Port ARCO `rrp.yml` obstacle layout to FRET scenario
- [ ] Port ARCO `rr.yml` planar scenario
- [ ] MuJoCo MJCF for RRP
- [ ] Gazebo SITL validation
- [ ] ARCO vs FRET comparison video
- [ ] Tag `v1.2.0`

---

## Phase 5 — v1.3 6-DOF challenge 🔲

**Robot:** 6-DOF revolute manipulator. **Scenario:** SC-v13.

- [ ] Select robot model (UR5 / custom)
- [ ] Numerical IK + Jacobian controller
- [ ] 6-D C-space checker with self-collision
- [ ] Cluttered cell environment
- [ ] Tag `v1.3.0`

---

## Phase 6 — Hardware HITL 🔲 *Post v1.3*

- [ ] Micro-ROS serial bridge (`hardware/bridge_node.py`)
- [ ] Encoder feedback loop
- [ ] Physical prototype integration

---

## Phase 7 — Vision 🔲 *Future*

- [ ] Camera-based perception
- [ ] Dynamic replanning from visual feedback
