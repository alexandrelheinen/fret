# FRET Project Roadmap

> **Authoritative release criteria:** [releases.md](releases.md)  
> **Vision program (v1.3–v1.5):** [vision/README.md](vision/README.md)

---

## Versioning eras

FRET is organized in three **eras**. Minor versions inside an era share the same
kind of work; major bumps change the delivery surface.

| Era | Versions | Focus | Hardware? |
| --- | --- | --- | --- |
| **Simulation & algorithms** | **v1.x** | Planners, controllers, MuJoCo SITL, CV in sim | No |
| **Hardware integration** | **v2.x** | Modular HITL: bridge → real cameras → real arm → full stack | Yes |
| **Product** | **v3.0+** | Definitive integrated system; then incremental features | Yes |

**Rules:**

1. **All of v1.x stays in simulation.** No Micro-ROS, Pi, or physical actuators
   until the v2.x line.
2. **TB3 / Dubins** is a **case study**: port ARCO kinematic SE(2) racing into
   MuJoCo physics. It does **not** consume computer vision — only manipulators
   interact with graspable scene objects.
3. **Manipulators (OM-X, OMY)** are the CV consumers: detect / classify balls and
   drive pick-and-place without hardcoded ball poses (from v1.4 onward).
4. **v3.0** is a north-star (“everything that works, integrated”). This roadmap
   only *mentions* it; no task breakdown for 3.0+ is maintained here.

```
v1.1–v1.2.x   ✅  Mobile + arm MuJoCo showcases (TB3, OM-X, OMY)
     │
     ▼
v1.3   ✅  Computer-vision pipeline (algorithms, unit tests, selection)
     │
     ▼
v1.4   🔲  CV ↔ manipulation integration (MuJoCo cameras, no hardcoded ball)
     │
     ▼
v1.5   🔲  Dynamic ball delivery + industrial place geometry + pickability
     │
     ▼
v2.0+  🔲  Hardware line (modular HITL)
     │
     ▼
v3.0   ○   Definitive product (goal only — no detailed plan yet)
```

---

## Project goals

* **Product (long-term):** ROS 2 full-stack framework: ARCO planning + control +
  vision → simulation, then hardware.
* **Middleware:** ROS 2 Jazzy.
* **Planning / control:** ARCO (RRT*, SST, occupancy, path-following and
  joint-space MPC).
* **Simulation:** MuJoCo — physics, contacts, cameras, rendering, SITL.
* **Assets:** ROBOTIS MuJoCo Menagerie + AWS RoboMaker warehouse (submodules).
* **v1.x vision:** Track a graspable ball; place target / dispenser pose stays a
  **known scenario parameter** (fixed industrial fixture).
* **v2.x hardware:** Raspberry Pi 5 + Arduino Mega (Micro-ROS) and real cameras,
  introduced **after** v1.5 is validated — modular milestones, not a big-bang cutover.

---

## Phase map (v1.x complete → open)

### Phase 0 — Specification ✅

- [x] C-space / SE(2) domains, interface contracts, QoS, CI

### Phase 1 — Mobile race showcase ✅ *(v1.1)*

**Robot:** TurtleBot3 Burger (Menagerie). **Role:** ARCO → MuJoCo case study.

- [x] SE(2) planning, dual-agent race, AWS maze, path-following MPC, release MP4s

### Phase 2 — MuJoCo physics SITL ✅ *(v1.2)*

- [x] `physics_mode`, wheel actuators, contact logging, physics-default showcase

### Phase 3 — OpenMANIPULATOR-X tabletop ✅ *(v1.2.3)*

- [x] SC-v13a–d: reach, pick-place FSM, desk clutter, Γ-wall maze

### Phase 4 — 6-DOF OpenMANIPULATOR-Y ✅ *(v1.2.4 + patches)*

- [x] SC-v14a–c: reach, pedestal pick-place, clutter detour; showcase RRT*/SST
- [x] Telemetry beside R2 videos *(v1.2.6)*; Dubins encode polish *(v1.2.7)*

Optional sim polish (non-blocking for the CV line): denser AWS desk props;
explicit 6-D self-collision C-space checker.

### Phase 5 — Computer vision pipeline ✅ *(v1.3)*

**Scope:** algorithms + unit tests + **algorithm selection** only. No MuJoCo
camera MJCF required to *select*; fixtures may use synthetic images.

- [x] Spec `FR-VIS-*` + module package `fret.vision`
- [x] Evaluate candidates against [vision/algorithm-selection.md](vision/algorithm-selection.md)
- [x] Lock primary detector / tracker for ball centre in image + world lift
  (HSV blob + table-plane lift; OpenCV)
- [x] Unit tests on synthetic fixtures (`tests/vision/`; CI control shard)
- [x] Web gallery script for qualitative photos (`scripts/vision_web_ball_gallery.py`)
- [x] Tag `v1.3.0`

### Phase 6 — CV ↔ manipulation integration 🔲 *(v1.4)*

- [x] MuJoCo cameras in OM-X / OMY cells (rear structural gate, dual corner cams)
- [x] MuJoCo image adapter → `fret.vision` + dual-view detect/lift benchmarks (no FSM feed)
- [x] Place cone moved in front of arm; pick on the side (±90° yaw margin)
- [x] Wire `BallObservation` → pick-and-place (replace hardcoded ball / `pick_xy` as grasp GT)
- [x] Keep **place / dispenser** as known YAML parameters
- [x] Equivalent behaviour to today’s physics smoke without pose cheats
- [ ] Tag `v1.4.0`

### Phase 7 — Dynamic scene + industrial place 🔲 *(v1.5)*

- [ ] Ball rolls on floor to a **random** rest pose; CV detects it
- [ ] **Pickability** classifier (in workspace / graspable vs reject)
- [ ] Asset research + improved container / dispenser geometry
- [ ] Decide single-ball vs multi-ball delivery for the release scenario
- [ ] Tag `v1.5.0`

### Phase 8 — Hardware line 🔲 *(v2.x)*

Modular integration (order fixed; each may be its own minor tag):

1. Low-level target bridge (Micro-ROS / serial)
2. Real image pipeline (same `fret.vision` contracts)
3. Real manipulator actuation + encoder feedback
4. Full closed-loop pick-and-place on hardware

See [releases.md § v2.x](releases.md#v2x--hardware-integration-line).

### Phase 9 — Definitive product ○ *(v3.0)*

Mention only: a release where simulation-proven algorithms and hardware modules
are validated together as the default product. Detailed 3.0+ planning is
**out of scope** for this document.

---

## Robot roles (stable)

| Model | Era role | Uses CV? |
| --- | --- | --- |
| `dubins` (TB3) | Case study: ARCO kinematics → MuJoCo physics race | **No** |
| `open_manipulator_x` | 4-DOF tabletop manipulation + CV from v1.4 | **Yes** |
| `omy` / `six_dof` | 6-DOF manipulation + CV from v1.4 | **Yes** |

Full acceptance criteria: [releases.md](releases.md).  
Architecture for vision: [vision/architecture.md](vision/architecture.md).
