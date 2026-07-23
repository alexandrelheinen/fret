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
| **v1.1** Dubins | `SSTPlanner`, `DubinsVehicle`, path-following MPC (`DubinsPathFollowingMPC`) | `map/vehicle.yml` / `map/city.yml` |
| **v1.2** | *(physics upgrade — no new ARCO scenario)* | — |
| **v1.2.3** manipulator | `KDTreeOccupancy`, `SSTPlanner`, `JointSpaceMPC` | `map/rrp.yml`, `map/ppp.yml` |
| **v1.2.4** 6-DOF | `SSTPlanner`, `KDTreeOccupancy` | *(new)* |

Bootstrap arm (MS-1–5) also uses `KDTreeOccupancy`, `SSTPlanner`, and
`TrajectoryPruner` for regression CI.

---

## Ownership boundary

| Responsibility | Owner |
|---|---|
| Occupancy (`KDTreeOccupancy`) | ARCO |
| Planner (SST, RRT*) | ARCO |
| Dubins vehicle + path-following MPC | ARCO (≥ v0.3.2; progress-first contouring via arco#147 on main) |
| Joint-space MPC (OMX pick-and-place) | ARCO (≥ v0.3.2) |
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
- `DubinsVehicle` + `DubinsPathFollowingMPC` for RRT*/SST execution
  (grey foil retains Pure Pursuit)
- Extend environment with 3-D columns (MuJoCo visual)

### Progress-first contouring (arco#147)

Global planners ignore vehicle dynamics, so forcing the online MPC to
zero-fit their polylines stalls / orbits at sharp kinks. FRET’s Dubins
race trackers therefore consume ARCO’s progress-first fields:

| YAML key (`dubins.yml` → `mpc`) | ARCO config field | Role |
|---|---|---|
| `weight_lag` | `weight_lag` | Penalty on lagging `s0 + v_cruise·t` |
| `contour_deadzone` | `contour_deadzone` | Free lateral band (m); contour cost on `(|e_lat| − deadzone)_+²` |

Wired in `fret.scenario.dubins_race_runner._mpc_config` via
`PathFollowingMPCConfig.with_weight_overrides(...)`. Race agents keep
`occupancy=None` (planner owns avoidance).

**Lab scale, not city scale.** ARCO city demos use ~8 m deadzone / 14 m/s
cruise; FRET’s warehouse aisles are ≈1.6 m with TurtleBot half-size
≈0.09 m and cruise ≈0.36 m/s, so `dubins.yml` uses
`contour_deadzone: 0.10` m and softer contour/heading weights. Setting
`contour_deadzone: 0` restores classic contouring. Always-on ARCO
behavior even at zero lag/deadzone: no reverse progress
(`ṡ = v · max(cos e_ψ, 0)`) and geometric projection never rewinds `s`.

---

## v1.2.3 manipulator alignment

Reproduce ARCO CI scenarios `rrp` and `rr` inside FRET `sitl.py` pipeline on
MuJoCo physics SITL. FRET bootstrap arm kinematics map to ARCO manipulator topology.

---

## References

- ARCO visualization docs: `arco/docs/VISUALIZATION.md`
- ARCO vehicle scene: `arco/src/arco/simulator/scenes/vehicle.py`
- ARCO manipulator scene: `arco/src/arco/simulator/scenes/rrp.py`
