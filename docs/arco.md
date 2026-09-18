# ARCO Integration in FRET

> **Release mapping:** [releases.md](releases.md) · **Robot models:** [robots/README.md](robots/README.md)

---

## What is ARCO

ARCO (Algorithms for Robotic Control and Optimization) is the author-owned
planning library. From v0.5.0 its planners, maps, guidance and controllers are
Rust in a compiled extension module, and the Python package re-exports them
under the names they always had. FRET consumes ARCO for occupancy,
sampling-based planning, and (selected releases) guidance controllers.

ARCO repository: `https://github.com/alexandrelheinen/arco`

---

## ARCO usage by FRET release

| Release | ARCO components | ARCO scenario reference |
|---|---|---|
| **v1.1** Dubins | `SSTPlanner`, `DubinsVehicle`, path-following MPC (`DubinsPathFollowingMPC`) | `map/vehicle.yml` / `map/city.yml` |
| **v1.2** | *(physics upgrade — no new ARCO scenario)* | — |
| **v1.2.3** manipulator | `KDTreeOccupancy`, `SSTPlanner`, `JointSpaceMPC` | `map/rrp.yml`, `map/ppp.yml` |
| **v1.2.4** 6-DOF | `SSTPlanner`, `KDTreeOccupancy`, `JointSpaceMPC` | OMY clutter / pick-place |
| **v1.3+** Vision | *(FRET `fret.vision` — not ARCO)* | Ball track for manipulators |
| **v1.4.4** | Same components on the ARCO v0.5.0 compiled core | unchanged |

Bootstrap arm (MS-1–5) also uses `KDTreeOccupancy`, `SSTPlanner`, and
`TrajectoryPruner` for regression CI.

---

## Ownership boundary

| Responsibility | Owner |
|---|---|
| Occupancy (`KDTreeOccupancy`) | ARCO |
| Planner (SST, RRT*) | ARCO |
| Dubins vehicle + path-following MPCC | ARCO (≥ v0.3.7 classical MPCC) |
| Joint-space MPC (OMX pick-and-place) | ARCO (≥ v0.3.7; step dt must equal config.dt) |
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

`pyproject.toml` pins the v0.5.0 release wheels by URL, one per platform,
because the `arco` name on PyPI belongs to an unrelated project. The wheels
carry the abi3 extension, so installing needs no Rust toolchain; a platform
without one of the pinned wheels falls back to the source distribution and
does need `rustup`. The `mpc` extra no longer exists: CasADi and IPOPT left
with it, and ARCO now depends on SciPy.

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

### Classical MPCC (ARCO ≥ v0.3.7)

Global planners ignore vehicle dynamics, so a stiff zero-fit of their
polylines stalls / orbits at sharp kinks. ARCO v0.3.7 reworks
`DubinsPathFollowingMPC` as classical contouring MPCC (Lam / Liniger):
free virtual progress speed \(v_s\), **linear** progress reward, and a
**structural** lag error that couples path parameter \(s\) to the
vehicle. FRET’s Dubins race trackers consume:

| YAML key (`dubins.yml` → `mpc`) | ARCO config field | Role |
|---|---|---|
| `weight_lag` | `weight_lag` | Lag-error weight (**must be > 0**; construction rejects ≤ 0) |
| `weight_progress` | `weight_progress` | Linear reward per meter of arc-length advancement |
| `contour_deadzone` | `contour_deadzone` | Optional free lateral band (m); **keep 0** (flats chatter) |
| `weight_heading` | `weight_heading` | Keep small (~1/10 of contour); tracking emerges from contour + lag |

Wired in `fret.scenario.dubins_race_runner._mpc_config` via
`PathFollowingMPCConfig.with_weight_overrides(...)`. Race agents keep
`occupancy=None` (planner owns avoidance). `weight_slack` was removed
upstream and is no longer forwarded.

**Lab scale, not city scale.** ARCO city demos use ~14 m/s cruise and a
5 s horizon; FRET’s warehouse aisles are ≈1.6 m with TurtleBot half-size
≈0.09 m and cruise ≈0.36 m/s, so `dubins.yml` keeps a 1.2 s horizon
(`24 × 0.05 s`) matching `update_rate: 20`. Model `dt` must equal the
control period. Projection never rewinds `s`; curve-limited \(v_s\) caps
handle corner braking.

---

## What the compiled core (v0.5.0) changed for FRET

The imports and the argument names held, so most of FRET reads the same.
Five things did not, and each one is a place FRET had to move.

**Planner and controller state is read-only.** `DubinsVehicle.x/y/heading`
and `JointSpaceMPC.q/vel` are compiled properties with no setters, so
assignment raises `AttributeError`. FRET moves a vehicle through
`reset(x, y, heading)` and an arm through `reset(q)`. Because `reset` also
zeroes the speed and turn-rate filters, `fret.scenario.dubins_race_runner`
wraps it in `_place_vehicle`, which restores both through their setters:
the physics sync rewrites the pose on every tick, and dropping the filter
state there would hand the tracker a standing vehicle it never commanded.

**A Python map has to subclass `arco.mapping.occupancy.Occupancy`.** The
compiled planners and `TrajectoryPruner` call `segment_free` on the map
they are given, where the Python versions sampled the segment themselves.
`_CSpaceOccupancy` in `fret.planning.planner_node` now inherits that base
and implements `nearest_obstacle` alongside `is_occupied`. A duck-typed map
that only answers `is_occupied` fails inside the pruner, and the arm
scenarios silently fell back to their IK corridor when it did.

**Planner seeding is an argument, not a patched numpy.** RRT* and SST run
their own generator in Rust, so patching `numpy.random.default_rng` no
longer reaches them. `deterministic_planner_rng` still pins numpy for
FRET's own sampling and now also publishes the active seed through
`active_planner_seed()`, which every planner construction forwards as
`seed=`. Without it a showcase render is unseeded again.

**The two predictive controllers solve convex programs.** They reach a
different and also valid solution for the same input, `MPCStepResult.cost`
reports the surrogate objective and is comparable only across steps of one
controller, and `solver_status` uses a new vocabulary (`solved`,
`solved_inexact`, `invalid_state`, `infeasible`, `unbounded`,
`budget_exhausted`, `numerical`) in place of the IPOPT strings. Nothing in
FRET matches on those strings today; anything added later should read
`last_solver_success` instead.

**`obstacle_barrier_power` is inert.** The obstacle barrier is a half-space
with a penalized slack, so the exponent no longer shapes it. The key stays
in `config/controllers/dubins.yml` because the config record still accepts
it, with a comment saying it does nothing.

Every control step also validates the interval it is given against the
configured model step, which FRET already satisfied, and the rejection
message changed. ARCO's own `docs/rust/DEVIATIONS.md` names the symbol and
the reason behind each of these (A-17, A-20, A-30, A-31, A-32, A-37).

---

## v1.2.3 manipulator alignment

Reproduce ARCO CI scenarios `rrp` and `rr` inside FRET `sitl.py` pipeline on
MuJoCo physics SITL. FRET bootstrap arm kinematics map to ARCO manipulator topology.

---

## References

- ARCO visualization docs: `arco/docs/VISUALIZATION.md`
- ARCO vehicle scene: `arco/src/arco/simulator/scenes/vehicle.py`
- ARCO manipulator scene: `arco/src/arco/simulator/scenes/rrp.py`
