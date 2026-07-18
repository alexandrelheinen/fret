# ARCO MPC Path-Following Controller — Agent Specification

> **Status:** Handoff spec for the ARCO repository (not implementable from FRET).
> **Consumer:** FRET v1.1+ Dubins race (`build_vehicle_sim`, physics SITL).
> **ARCO standards:** [CONTRIBUTING.md](https://github.com/alexandrelheinen/arco/blob/main/CONTRIBUTING.md), [docs/guidelines.md](https://github.com/alexandrelheinen/arco/blob/main/docs/guidelines.md), [AGENTS.md](https://github.com/alexandrelheinen/arco/blob/main/AGENTS.md) — full V-cycle required.

---

## 1. Problem statement

FRET's Dubins race executes planned paths with **Pure Pursuit + additive APF repulsion** (`TrackingLoop._repulsion_turn_rate`). This architecture fails structurally in tight corridors:

- Path tracking and obstacle avoidance are **decoupled** and fight each other.
- Repulsion uses **nearest-point distance**, not directional relevance along the velocity cone.
- **Speed is not co-optimized** with steering; the vehicle holds cruise speed until turn-rate saturation.
- The same APF pattern exists in `JointSpaceTracker` and will not generalize to v1.3 SCARA / v1.4 6-DOF.

**Goal:** Replace online reactive tracking (PP + APF) with a **receding-horizon path-following MPC** that jointly optimizes lateral error, progress, speed, and obstacle clearance under Dubins/unicycle dynamics.

**Non-goals (this issue):**

- Replacing planning-time `TrajectoryOptimizer` (keep it).
- Full motion replanning (RRT*/SST stay upstream).
- ROS / MuJoCo bindings (FRET owns those).
- acados backend (optional follow-up after CasADi formulation is stable).

---

## 2. Motivation from FRET validation

On the `dubins_race` physics showcase (seed 5, warehouse pinch `str_002_col` / `str_008_col`):

| Tracker | Symptom |
|---------|---------|
| PP + APF @ `repulsion_gain=1.5` | Turn-rate saturation; clearance ~0.03 m; collision |
| PP + APF @ `repulsion_gain=3.0` | Better margin (~0.14 m) but still reactive; multi-seed collision rate 20% at `clearance_margin=0.30` |
| Pre-emptive occupancy block (removed) | Masked algorithm bugs; "glue effect" when combined with speed modulation |

MPC acceptance must show the optimizer **slows before pinch points** rather than relying on last-moment repulsion.

---

## 3. Architecture (ARCO)

### 3.1 Package layout

Follow guidelines §1 — use a sub-package, not suffixed filenames:

```
src/arco/control/mpc/
    __init__.py              # re-exports public API only
    base.py                  # MPCTracker ABC
    path_following.py        # DubinsPathFollowingMPC
    reference_path.py        # ReferencePath (arc-length queries)
    costs.py                 # barrier / contouring cost helpers (private-ish)
    result.py                # MPCStepResult dataclass
```

Add config:

```
src/arco/config/mpc.yml
```

### 3.2 Class hierarchy

The existing scalar `MPCController(Controller)` stub (`control(self, state: float, reference: float) -> float`) is **not suitable** for multi-state MPC. Do **not** flesh out that API.

| Class | Role |
|-------|------|
| `MPCTracker` (ABC) | `step(pose, reference, dt) -> MPCStepResult` |
| `DubinsPathFollowingMPC` | SE(2) unicycle NMPC implementation |
| `MPCTrackingLoop` | Drop-in parallel to `TrackingLoop`; wraps `DubinsVehicle` + `MPCTracker` |
| `ReferencePath` | Arc-length parameterization over planner waypoints |
| `MPCStepResult` | `speed_cmd`, `turn_rate_cmd`, diagnostics, `solver_success` |

Keep `PurePursuitController` and `TrackingLoop` unchanged for backward compatibility. Add factory sibling to `build_vehicle_sim`:

```python
# arco/simulator/sim/tracking.py
def build_vehicle_mpc_sim(
    waypoints: list[tuple[float, float]],
    cfg: VehicleConfig,
    mpc_cfg: PathFollowingMPCConfig,
    occupancy: Occupancy | None = None,
) -> tuple[DubinsVehicle, MPCTrackingLoop]: ...
```

Extend `VehicleConfig` with optional `tracker: Literal["pure_pursuit", "mpc"] = "pure_pursuit"` **or** keep tracker selection in YAML only (preferred: separate `mpc.yml` loaded by factory).

### 3.3 Dependency

Add **optional** extra in `pyproject.toml`:

```toml
[project.optional-dependencies]
mpc = ["casadi>=3.6"]
```

Core `arco` install must remain CasADi-free. `DubinsPathFollowingMPC` imports CasADi lazily; missing dependency raises `ImportError` with install hint: `pip install arco[mpc]`.

FRET will add `arco[mpc]` to its sim extra once ARCO releases.

---

## 4. Mathematical formulation — Dubins path-following NMPC

### 4.1 Dynamics (discrete Euler)

State at step `k`: `x_k = [p_x, p_y, θ, v, ω]^T` (world pose + realized speed/turn rate for warm-start consistency).

Controls: `u_k = [a_k, ω̇_k]^T` (linear acceleration, turn-rate derivative).

```
p_x,k+1 = p_x,k + v_k cos(θ_k) dt
p_y,k+1 = p_y,k + v_k sin(θ_k) dt
θ_k+1   = θ_k + ω_k dt
v_k+1   = clip(v_k + a_k dt, min_speed, max_speed)
ω_k+1   = clip(ω_k + ω̇_k dt, -max_turn_rate, +max_turn_rate)
```

Use the same saturation semantics as `DubinsVehicle.step` so MPC predictions match integration.

### 4.2 Reference path

Input: ordered waypoints `[(x_i, y_i)]` from pruned + optimized planner path.

`ReferencePath` provides, for arc length `s`:

- `position(s) -> (x_ref, y_ref)`
- `tangent(s) -> (cos θ_ref, sin θ_ref)`
- `curvature(s)` (finite differences acceptable)
- `project(pose) -> (s, lateral_error, heading_error)`

Progress state `s_k` is a **slowly evolving** scalar advanced by `v_k cos(heading_error_k)` (contouring MPC convention).

### 4.3 Cost (per step k over horizon N)

```
J = Σ_k [
      w_contour  * e_lat,k²
    + w_heading  * e_head,k²
    + w_progress * (v_ref - v_k)²        # v_ref = cruise_speed on straights
    + w_control  * (a_k² + ω̇_k²)
    + w_slack    * slack_k²              # soft constraint slack
]
  + w_terminal * (e_lat,N² + e_head,N²)
  + Σ_k obstacle_barrier(p_x,k, p_y,k, θ_k, v_k)
```

**Obstacle barrier** (reuse `TrajectoryOptimizer` philosophy):

```
d_k = dist_to_obstacle(p_x,k, p_y,k) - clearance
barrier_k = w_obs * max(0, -d_k / clearance) ** barrier_power
```

**Directional weighting (required):** multiply `barrier_k` by a forward cone factor:

```
cone_k = max(0, cos(θ_k - bearing_to_obstacle))   # 1 ahead, 0 abeam/behind
barrier_k *= (0.2 + 0.8 * cone_k)
```

Use `occupancy.nearest_obstacle(np.array([p_x, p_y]))` — same interface as `TrackingLoop` and `TrajectoryOptimizer`.

### 4.4 Constraints

Hard (or tight soft with slack):

- `min_speed ≤ v_k ≤ max_speed`
- `|ω_k| ≤ max_turn_rate`
- `|a_k| ≤ max_acceleration`
- `|ω̇_k| ≤ max_turn_rate_dot`
- Optional: `v_k ≤ v_curve,k` where `v_curve,k = min(max_speed, max_turn_rate / max(|κ_ref|, ε))`

### 4.5 Solver

- **CasADi** + **IPOPT** (default) for prototype.
- Horizon `N = 15–25`, `dt = 0.05` (match FRET `simulation_dt`).
- Warm-start from previous solution shifted by one step.
- Solver time budget: **< 40 ms** per step on CI runner (soft requirement; log `solve_time_s`).
- On solver failure: fall back to **last feasible command** or safe stop `(a=−max_acceleration, ω̇=0)`; set `solver_success=False` in result. Do **not** fall back to PP+APF silently.

---

## 5. Public API (typed, Google docstrings)

### 5.1 `MPCStepResult`

```python
@dataclass
class MPCStepResult:
    speed_cmd: float
    turn_rate_cmd: float
    cross_track_error: float
    heading_error: float
    progress: float
    predicted_clearance_min: float
    solver_success: bool
    solver_status: str
    solve_time_s: float
    cost: float
```

### 5.2 `PathFollowingMPCConfig`

```python
@dataclass
class PathFollowingMPCConfig:
    horizon_step_count: int = 20
    dt: float = 0.05
    cruise_speed: float = 0.36
    weight_contour: float = 10.0
    weight_heading: float = 5.0
    weight_progress: float = 1.0
    weight_control: float = 0.1
    weight_obstacle: float = 50.0
    obstacle_barrier_power: float = 4.0
    weight_terminal: float = 20.0
    max_solver_iter_count: int = 50
```

Load defaults from `config/mpc.yml` via `PathFollowingMPCConfig.create_from_config()`.

### 5.3 `DubinsPathFollowingMPC`

```python
class DubinsPathFollowingMPC(MPCTracker):
    def __init__(
        self,
        *,
        vehicle_limits: DubinsVehicleLimits,  # max_speed, min_speed, etc.
        config: PathFollowingMPCConfig,
        occupancy: Occupancy | None = None,
    ) -> None: ...

    def set_reference(self, waypoints: Sequence[tuple[float, float]]) -> None: ...

    def step(
        self,
        pose: tuple[float, float, float],
        *,
        speed: float,
        turn_rate: float,
        dt: float,
    ) -> MPCStepResult: ...
```

### 5.4 `MPCTrackingLoop`

Mirror `TrackingLoop.step` return schema for drop-in metrics:

```python
def step(self, path: list[tuple[float, float]], dt: float = 0.1) -> dict[str, Any]:
    # Returns keys compatible with TrackingLoop:
    # cross_track_error, heading_error, pose, speed, turn_rate,
    # plus mpc_* diagnostics
```

**Remove** APF repulsion from MPC loop — obstacle avoidance is inside the optimizer.

---

## 6. Configuration (`config/mpc.yml`)

```yaml
horizon:
  step_count: 20
  dt: 0.05

weights:
  contour: 10.0
  heading: 5.0
  progress: 1.0
  control: 0.1
  obstacle: 50.0
  terminal: 20.0

obstacle_barrier:
  power: 4.0

solver:
  max_iter_count: 50
```

---

## 7. Tests (mirrored layout)

Create `tests/control/mpc/`:

| Test | Acceptance |
|------|------------|
| `test_reference_path_project_straight_line` | Zero lateral error on centerline |
| `test_mpc_tracks_straight_path_no_obstacles` | After 5 s sim, `|cross_track_error| < 0.05 m`, `solver_success` rate > 95% |
| `test_mpc_slows_before_box_obstacle` | Single box ahead on straight path; speed at `t=2s` < 0.5 × cruise when starting at cruise |
| `test_mpc_avoids_lateral_obstacle` | Wall offset 0.5 m from path; min clearance ≥ 0.8 × occupancy.clearance |
| `test_mpc_respects_max_turn_rate` | All commanded ω within limit ±1% |
| `test_mpc_solver_failure_safe_stop` | Corrupt initial state → `solver_success=False`, deceleration command |
| `test_mpc_tracking_loop_metrics_schema` | Same keys as `test_step_returns_all_metric_keys` |
| `test_import_without_casadi` | `import arco.control.mpc` works; instantiating solver raises clear `ImportError` |

Use synthetic `RectOccupancy` test fixture (axis-aligned box), not FRET imports.

### 7.1 Regression scenario (tools/)

Add `tools/mpc_pinch_demo.py` (headless matplotlib, no pygame):

- Reproduce pinch geometry analogous to FRET `str_002_col` / `str_008_col` gap.
- Plot speed, lateral error, min clearance vs time for PP+APF vs MPC.
- Save figure to `tools/output/mpc_pinch_demo.png` for PR evidence.

---

## 8. Documentation updates (ARCO)

| File | Change |
|------|--------|
| `docs/GUIDANCE.md` | Replace MPC stub description with real API |
| `docs/ROADMAP.md` | Mark MPC tracker shipped; note joint-space MPC as follow-up |
| `README.md` | `pip install arco[mpc]` usage snippet |

---

## 9. FRET integration contract (downstream, separate PR)

After ARCO release `>= 0.4.0`:

1. `pyproject.toml`: `arco[mpc] @ git+...@tag`
2. `dubins.yml` or new `dubins_mpc.yml`: MPC weights
3. `dubins_race_runner.py`: `build_vehicle_mpc_sim` when `tracker: mpc`
4. Remove `_agent_physics_velocity_command` PP+APF hack; command MPC output directly
5. Keep FRET `_CollisionMonitor` as post-hoc safety latch (unchanged)
6. Multi-seed acceptance: collision rate **0/20** on seeds `0..19` at `clearance_margin=0.30`, `cruise_speed=0.36`

FRET will **not** import CasADi directly.

---

## 10. Phase 2 — Joint-space MPC (follow-up issue)

After Dubins MPC ships, generalize:

```
src/arco/control/mpc/joint_space.py  # JointSpaceMPC(MPCTracker)
```

- State: `q`, `q_dot`; control: `q_ddot` or velocity increments
- Obstacle cost via `occupancy.nearest_obstacle(q)` in C-space
- Replace `JointSpaceTracker` APF in v1.3 SCARA pipeline

Same `MPCTracker` ABC, same `MPCStepResult` pattern extended to N-DOF.

---

## 11. V-cycle checklist (mandatory)

Per ARCO `AGENTS.md`:

1. **GitHub issue** with acceptance criteria from §7 + §9.6
2. **Architecture + tests first** (CasADi solver can `@pytest.mark.xfail` until step 3)
3. **Implementation** filling solver
4. **`bash scripts/pre_push.sh`** — all 5 gates pass
5. **`tools/mpc_pinch_demo.py`** visual evidence in PR

---

## 12. Suggested PR title and scope

**Title:** `feat(control): Dubins path-following MPC tracker (CasADi)`

**Scope:**

- New `arco.control.mpc` sub-package
- `DubinsPathFollowingMPC`, `MPCTrackingLoop`, `ReferencePath`
- `config/mpc.yml`, `build_vehicle_mpc_sim`
- Tests + pinch demo tool
- Deprecate scalar `MPCController` stub with `DeprecationWarning` pointing to `DubinsPathFollowingMPC`

**Out of scope:** acados, joint-space MPC, FRET wiring.

---

## 13. Open decisions for maintainer

| Decision | Recommendation |
|----------|----------------|
| CasADi vs scipy shooting | CasADi (explicit NMPC, extensible to arms) |
| Replace or parallel `TrackingLoop` | Parallel `MPCTrackingLoop`; PP remains default |
| Hard vs soft obstacle constraints | Soft barrier (match `TrajectoryOptimizer`) |
| Progress state in MPC | Contouring `s` as slow state, not full replan |

---

## 14. References

- FRET collision analysis: branch `cursor/remove-dubins-speed-modulation-f5a1`
- ARCO `TrajectoryOptimizer` collision barrier: `planning/continuous/optimizer.py`
- ARCO `TrackingLoop` APF (to be superseded online): `control/tracking.py`
- Contouring MPC: Lam et al., "Real-time trajectory and velocity optimization for autonomous vehicles"
