# ARCO Integration in FRET

> **Release mapping:** [releases.md](releases.md) · **Robot models:** [robots/README.md](robots/README.md)

---

## What is ARCO

ARCO (Algorithms for Robotic Control and Optimization) is the author-owned Python
planning library. FRET consumes ARCO for occupancy, sampling-based planning, and
(selected releases) guidance controllers.

ARCO repository: `https://github.com/alexandrelheinen/arco`

---

## ARCO usage by FRET release

| Release | ARCO components | ARCO scenario reference |
|---|---|---|
| **v1.1** Dubins | `SSTPlanner`, `DubinsVehicle`, `DubinsPrimitive`, Pure Pursuit | `map/vehicle.yml` |
| **v1.2** | *(physics upgrade — no new ARCO scenario)* | — |
| **v1.3** manipulator | `KDTreeOccupancy`, `SSTPlanner` | `map/rrp.yml`, `map/rr.yml` |
| **v1.4** 6-DOF | `SSTPlanner`, `KDTreeOccupancy` | *(new)* |

Bootstrap arm (MS-1–5) also uses `KDTreeOccupancy`, `SSTPlanner`, and
`TrajectoryPruner` for regression CI.

---

## Ownership boundary

| Responsibility | Owner |
|---|---|
| Occupancy (`KDTreeOccupancy`) | ARCO |
| Planner (SST, RRT*) | ARCO |
| Dubins vehicle + Pure Pursuit | ARCO (v1.1) |
| Scene acquisition, TF | FRET |
| Per-robot kinematics | FRET |
| ROS 2 I/O, MuJoCo simulation | FRET |

ARCO never reads ROS topics. FRET never calls ARCO internals beyond the public API.

---

## Data flow

```
FRET SceneAcquisition → OccupancyUpdatePayload → ARCO KDTreeOccupancy
FRET CSpaceChecker    ← wraps occupancy + FK
FRET PlanningRequest  → ARCO SSTPlanner.plan()
FRET PlanningResult   ← path + diagnostics
FRET TrajectoryGenerator → ARCO TrajectoryPruner
FRET ControllerNode   → execution
```

---

## Import strategy

```python
try:
    from arco.planning import SSTPlanner
except ImportError:
    SSTPlanner = None
```

CI may use linear-interpolation fallback when ARCO is absent. Release tags (v1.1+)
require ARCO installed.

---

## ARCO middleware

`arco.middleware` (Bus / PipelineRunner) is **not used**. FRET uses ROS 2 topics
and actions for all inter-node communication.

---

## v1.1 Dubins alignment

Reuse ARCO race infrastructure:

- `arco/simulator/scenes/vehicle.py` — dual-planner race scene
- `DubinsVehicle` + Pure Pursuit for execution
- Extend environment with 3-D columns (MuJoCo visual)

---

## v1.3 manipulator alignment

Reproduce ARCO CI scenarios `rrp` and `rr` inside FRET `sitl.py` pipeline on
MuJoCo physics SITL. FRET bootstrap arm kinematics map to ARCO manipulator topology.

---

## References

- ARCO visualization docs: `arco/docs/VISUALIZATION.md`
- ARCO vehicle scene: `arco/src/arco/simulator/scenes/vehicle.py`
- ARCO manipulator scene: `arco/src/arco/simulator/scenes/rrp.py`
