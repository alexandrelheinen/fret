# FRET Project Roadmap

> **Authoritative release criteria:** [releases.md](releases.md)  
> **Vision program:** [vision/README.md](vision/README.md)

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

1. **All of v1.x stays in simulation.** No hardware transport or physical
   actuators until the v2.x line; the v2.x hardware target is undefined
   pending redefinition (`docs/requirements.md` FR-HW-01..03).
2. **TB3 / Dubins** is a **case study**: port ARCO kinematic SE(2) racing into
   MuJoCo physics. It does **not** consume computer vision — only manipulators
   interact with graspable scene objects.
3. **Manipulators (OM-X, OMY)** are the CV consumers: detect / classify balls and
   drive pick-and-place without hardcoded ball poses.
4. **v3.0** is a north-star (“everything that works, integrated”). This roadmap
   only *mentions* it; no task breakdown for 3.0+ is maintained here.

```
v1.1–v1.4    ✅  Mobile + arm showcases, physics SITL, CV ↔ manipulation
     │
     ▼
v1.5.0   ✅  Robot-model-agnostic core (last minor of major 1)
     │
     ▼
v2.0+    🔲  Hardware line (modular HITL); dynamic-ball scope backlogged
     │
     ▼
v3.0     ○   Definitive product (goal only — no detailed plan yet)
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
* **v2.x hardware:** target undefined as of v1.5.0 (decided when hardware
  planning for the relevant phase opens, see `docs/hardware/`), introduced
  in modular milestones, not a big-bang cutover.

---

## Shipped (summary)

| Phase | Tag line | What landed |
| --- | --- | --- |
| Spec + CI | — | C-space / SE(2) domains, interface contracts, QoS |
| Mobile race | v1.1 | Dual-agent Dubins / TB3 race, AWS maze, path-following |
| Physics SITL | v1.2 | `physics_mode`, wheel actuators, contact logging |
| OM-X tabletop | v1.2.3 | Reach, pick-place FSM, desk clutter, Γ-wall maze |
| OMY 6-DOF | v1.2.4+ | Reach, floor pick-place, clutter; telemetry on R2 |
| CV pipeline | v1.3 | `fret.vision`, HSV + table-plane, fixture gates |
| CV ↔ manip | v1.4 | MuJoCo gate cameras → `BallObservation` → pick-place |
| Robot-model-agnostic core | v1.5.0 | Robot registry (`fret.robots`); hardware-neutral docs; hygiene (last minor of major 1) |

Full acceptance criteria for shipped lines: [releases.md](releases.md).

---

## Open work

### Dynamic scene + industrial place 🔲 *(simulation backlog)*

Deferred from the former "v1.5" plan when `v1.5.0` became the
robot-model-agnostic core release (`docs/releases.md` § v1.5.0). Not
scheduled to a release yet.

- [ ] Ball rolls on floor to a **random** rest pose; CV detects it
- [ ] **Pickability** classifier (in workspace / graspable vs reject)
- [ ] Asset research + improved container / dispenser geometry
- [ ] Decide single-ball vs multi-ball delivery for the release scenario

### Hardware line 🔲 *(v2.x)*

Modular integration (order fixed; each may be its own minor tag); the
transport and physical target are undefined until hardware planning for
the relevant phase opens (`docs/hardware/`):

1. Low-level target bridge (transport TBD)
2. Real image pipeline (same `fret.vision` contracts)
3. Real manipulator actuation + feedback
4. Full closed-loop pick-and-place on hardware

See [releases.md § v2.x](releases.md#v2x--hardware-integration-line).

### Definitive product ○ *(v3.0)*

Mention only: a release where simulation-proven algorithms and hardware modules
are validated together as the default product. Detailed 3.0+ planning is
**out of scope** for this document.

---

## Robot roles (stable)

| Model | Era role | Uses CV? |
| --- | --- | --- |
| `dubins` (TB3) | Case study: ARCO kinematics → MuJoCo physics race | **No** |
| `open_manipulator_x` | 4-DOF tabletop manipulation + CV | **Yes** |
| `omy` / `six_dof` | 6-DOF manipulation + CV | **Yes** |

Full acceptance criteria: [releases.md](releases.md).  
Architecture for vision: [vision/architecture.md](vision/architecture.md).
