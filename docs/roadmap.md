# FRET Project Roadmap

> **Release targets:** [releases.md](releases.md) (v1.0 → v1.4)

---

## Project goals

* **Product:** A ROS 2 full-stack robotics framework connecting **ARCO** planning to
  simulation and hardware.
* **Middleware:** ROS 2 Jazzy.
* **Planning:** ARCO (SST, KDTree occupancy, trajectory pruner).
* **Simulation:** MuJoCo — physics, contacts, rendering, and SITL for all releases.
* **Hardware path:** Raspberry Pi 5 + Arduino Mega via Micro-ROS (post v1.4).

---

## Release sequence

```
v0.9  ✅  Bootstrap SCARA pipeline (MS-1–5, pure-Python CI)
  │
  ▼
v1.0  —   Superseded bootstrap tag (not a product showcase)
  │
  ▼
v1.1  ✅  Dubins dual-robot race A→B through structure forest
  │
  ▼
v1.2  ✅  MuJoCo physics SITL — actuators, contacts, controller tuning
  │
  ▼
v1.3  🔲  RRP/SCARA — reproduce ARCO rrp + rr scenarios in FRET
  │
  ▼
v1.4  🔲  6-DOF manipulator — final challenge
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
- [x] ROS SITL launch files (`sitl.py`, `mujoco.py`)
- [x] MuJoCo backend adapter (`mujoco_bridge.py`)
- [x] Headless MP4 render pipeline for README / article

---

## Phase 2 — v1.1 Dubins race ✅ *Complete*

**Robot:** Dubins mobile × 2. **Scenario:** SC-v11 structure forest race.
**First product showcase.**

- [x] SE(2) planning adapter (ARCO DubinsVehicle)
- [x] Dual-agent race orchestration
- [x] Rectangular structure forest + dead-end alcoves
- [x] Pure Pursuit tracking integration
- [x] Release workflow + R2 upload (overview + follow POVs)
- [x] Tag `v1.1.0`

---

## Phase 3 — v1.2 MuJoCo physics SITL ✅ *Complete*

**Cross-cutting release.** Applies to the v1.1 Dubins showcase.

MuJoCo **physics mode** (`physics_mode:=true`) drives robots through velocity
actuators (`mj_step`), resolves contacts (columns, obstacles), and
publishes `/joint_states` from simulated `qpos`/`qvel`. Kinematic mirroring
remains available for fast regression (`physics_mode:=false`).

| Step | Tag | Focus |
| --- | --- | --- |
| 1 | `v1.1.2` ✅ | MJCF collision policy + physics tracking baseline |
| 2 | `v1.1.3` ✅ | Floor-contact / contact-log handoff |
| 3 | `v1.1.4` ✅ | Dubins RTF + tracking gates |
| 4 | `v1.1.5` ✅ | Regression harness + CI hardening |
| 5 | `v1.2.0` ✅ | Physics-default release showcase |

- [x] `physics_mode` parameter + `step_physics()` on MuJoCo bridge (T12-01)
- [x] MJCF actuators + `mujoco_physics.yml` (T12-02)
- [x] Contact logging harness (T12-03); showcase `--physics-mode` flag (T12-05)
- [x] Dubins physics race + `both_reached_goal` (T12-02)
- [x] Physics integration tests at V12 gates (T12-04)
- [x] Controller tuning workflow + measured baselines ([mujoco.md](mujoco.md), T12-06)
- [x] Release CI switched to `--physics-mode`
- [x] Package version 1.2.0; docs updated for physics-default behaviour

Full specification: [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl) ·
[mujoco.md](mujoco.md) · [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md).

---

## Phase 4 — v1.3 RRP / SCARA 🔲

**Robot:** RRP (existing SCARA) + RR planar. **Scenarios:** SC-v13a, SC-v13b.

- [ ] Port ARCO `rrp.yml` obstacle layout to FRET scenario
- [ ] Port ARCO `rr.yml` planar scenario
- [ ] MuJoCo MJCF for RRP (physics SITL from day one)
- [ ] MuJoCo physics SITL validation
- [ ] ARCO vs FRET comparison video
- [ ] Tag `v1.3.0`

---

## Phase 5 — v1.4 6-DOF challenge 🔲

**Robot:** 6-DOF revolute manipulator. **Scenario:** SC-v14.

- [ ] Select robot model (UR5 / custom; MuJoCo Menagerie import)
- [ ] Numerical IK + Jacobian controller
- [ ] 6-D C-space checker with self-collision
- [ ] Cluttered cell MJCF environment
- [ ] Tag `v1.4.0`

---

## Phase 6 — Hardware HITL 🔲 *Post v1.4*

- [ ] Micro-ROS serial bridge (`hardware/bridge_node.py`)
- [ ] Encoder feedback loop
- [ ] Physical prototype integration

---

## Phase 7 — Vision 🔲 *Future*

- [ ] Camera-based perception
- [ ] Dynamic replanning from visual feedback
