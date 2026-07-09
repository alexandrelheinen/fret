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
| **v1.0** PPP | `KDTreeOccupancy`, `SSTPlanner`, `TrajectoryPruner` | `map/ppp.yml` |
| **v1.1** Dubins | `SSTPlanner`, `DubinsVehicle`, `DubinsPrimitive`, Pure Pursuit | `map/vehicle.yml` |
| **v1.2** | *(physics upgrade — no new ARCO scenario)* | — |
| **v1.3** RRP | `KDTreeOccupancy`, `SSTPlanner` | `map/rrp.yml`, `map/rr.yml` |
| **v1.4** 6-DOF | `SSTPlanner`, `KDTreeOccupancy` | *(new)* |

---

## Ownership boundary

| Responsibility | Owner |
|---|---|
| Occupancy (`KDTreeOccupancy`) | ARCO |
| Planner (SST, RRT*) | ARCO |
| Dubins vehicle + Pure Pursuit | ARCO (v1.1) |
| Scene acquisition, TF | FRET |
| Per-robot kinematics | FRET |
| Magnetic grasp FSM | FRET (v1.0) |
| ROS 2 I/O, MuJoCo simulation | FRET |

ARCO never reads ROS topics. FRET never calls ARCO internals beyond the public API.

---

## Data flow

```
FRET SceneAcquisition → OccupancyUpdatePayload → ARCO KDTreeOccupancy
FRET CSpaceChecker    ← wraps occupancy + FK / grasp envelope
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

CI may use linear-interpolation fallback when ARCO is absent. Release tags (v1.0+)
require ARCO installed.

---

## ARCO middleware

`arco.middleware` (Bus / PipelineRunner) is **not used**. FRET uses ROS 2 topics
and actions for all inter-node communication.

---

## v1.0 PPP alignment

Port obstacle layout from ARCO `map/ppp.yml`:

- Workspace bounds: 60 × 20 × 6 m
- Width-crossing barriers and scatter boxes
- Start `(1, 1, 0)` → Goal `(59, 19, 0)` (adapt for FRET scenario YAML)

FRET adds magnetic grasp semantics not present in ARCO (cargo weld/release).

---

## v1.1 Dubins alignment

Reuse ARCO race infrastructure:

- `arco/simulator/scenes/vehicle.py` — dual-planner race scene
- `DubinsVehicle` + Pure Pursuit for execution
- Extend environment with 3-D columns (MuJoCo visual)

---

## v1.3 RRP alignment

Reproduce ARCO CI scenarios `rrp` and `rr` inside FRET `sitl.py` pipeline on
MuJoCo physics SITL. FRET bootstrap SCARA kinematics map to ARCO RRP topology.

---

## References

- ARCO visualization docs: `arco/docs/VISUALIZATION.md`
- ARCO PPP scene: `arco/src/arco/simulator/scenes/ppp.py`
- ARCO vehicle scene: `arco/src/arco/simulator/scenes/vehicle.py`
- ARCO RRP scene: `arco/src/arco/simulator/scenes/rrp.py`
