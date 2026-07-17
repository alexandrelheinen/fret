# FRET Release Specification (v1.0 → v1.4)

> **Authoritative product roadmap.** All requirements, scenarios, and milestones trace
> to this document.
>
> **Related:** [roadmap.md](roadmap.md) ·
> [requirements.md](requirements.md) ·
> [mujoco.md](mujoco.md) · [scenarios.md](scenarios.md) ·
> [README § Architecture](../README.md#architecture)

---

## Product vision

FRET is a **ROS 2 full-stack robotics framework** that connects the **ARCO** motion-
planning library to simulation and (eventually) hardware. Each minor release adds one
robot class, one showcase scenario, and one article-ready visual demo.

| Release | Robot | Scenario | Simulator |
|---|---|---|---|
| **v1.0** | *(superseded)* | Bootstrap iteration — not a product showcase | — |
| **v1.1** | Dubins mobile × 2 | Dual-robot race A→B through column forest | MuJoCo |
| **v1.2** | Dubins (physics upgrade) | Actuator-driven SITL with contact dynamics | MuJoCo |
| **v1.3** | RRP / SCARA (3-DOF) | Reproduce ARCO `rrp` / `rr` scenarios in FRET | MuJoCo |
| **v1.4** | 6-DOF manipulator | Final challenge — full C-space planning + execution | MuJoCo |

**Platform stack (all releases):**

- **Planning:** ARCO (RRT*, SST, KDTree occupancy, trajectory pruner)
- **Middleware:** ROS 2 Jazzy
- **Simulation:** MuJoCo (physics, contacts, rendering, SITL)

---

## Engineering foundation (v0.x — complete)

Before the first product showcase, the project validated the pipeline on a
**3-DOF SCARA bootstrap robot** (MS-1–5): Jacobian control, C-space planning,
occupancy bridge, pillar avoidance in pure-Python CI. That code remains as shared
infrastructure; it is **not** a release target. v1.3 reuses and extends it for RRP.

---

## v1.0 — superseded

Tag `v1.0.0` was a bootstrap iteration. **Dubins race (v1.1) is the first product
showcase.** No v1.0 acceptance criteria or tasks remain in this specification.

---

## v1.1 — Dubins dual-robot race

### Goal

Two **Dubins vehicles** race from **A → B** through a world of **rectangular
structures** (posts, walls, and dead-end alcoves), reproducing the ARCO `vehicle`
race format (RRT* vs SST).

### Environment

| Element | Description |
|---|---|
| Floor | 80 m × 80 m warehouse plane |
| Structures | Rectangular posts/walls; U-shaped dead-end alcoves with single entrances |
| Start A | Lower-left region (6, 6) |
| Goal B | Upper-right region (74, 74) |
| Visual | Varied structure heights for depth in MuJoCo |

### Robots

| Property | Value |
|---|---|
| Model name | `dubins` |
| DOF | 3 (x, y, θ) — SE(2) |
| Kinematics | ARCO `DubinsVehicle` |
| Control | ARCO Pure Pursuit |
| Planning | ARCO SST (per robot) |
| ARCO reference | `map/vehicle.yml`, `scenes/vehicle.py` |

### Scenario ID

**SC-v11** — `config/scenarios/dubins_race.yml`

### Acceptance criteria

| # | Criterion |
|---|---|
| V11-1 | Two robots plan independently; both reach B without collision with columns |
| V11-2 | Race video shows simultaneous motion (split-screen or shared world) |
| V11-3 | Dubins curvature constraint respected (min turning radius) |
| V11-4 | MP4 artifact uploaded on release tag |

### Implementation tasks

| ID | Task |
|---|---|
| T11-01 | SE(2) kinematics + Dubins validity adapter |
| T11-02 | Column forest world (MJCF) |
| T11-03 | Dual-agent launch + race orchestration |
| T11-04 | Integrate ARCO Pure Pursuit tracking loop |
| T11-05 | Race metrics (time-to-goal, path length) |

---

## v1.2 — MuJoCo physics SITL ✅

### Goal

Upgrade FRET from **kinematic mirroring** to **full MuJoCo physics** for the
shipped Dubins showcase (v1.1). Controller commands drive actuators; the
simulation integrates dynamics and resolves contacts. Robots follow physical
laws — no open-loop pose teleportation.

This release does not add a new robot or showcase scenario. It hardens the
simulation foundation required for v1.3+ arm releases.

### Previous limitation (v1.1, superseded)

| Aspect | Behaviour before v1.2 |
|---|---|
| Motion integration | Pure Python (Dubins vehicle) |
| MuJoCo role | Visual mirror: `qpos` write + `mj_forward` |
| Contacts | Not applied as forces during execution |
| `/joint_states` | Derived from integrated commands, not `mj_step` |

### Shipped behaviour (v1.2)

| Aspect | Target behaviour |
|---|---|
| Motion integration | `mj_step` at control rate (50 Hz) |
| Actuators | `<actuator>` elements per joint / agent |
| Contacts | Columns, floor, optional inter-agent blocking |
| `/joint_states` | Read from simulated `qpos` / `qvel` |
| Tuning | Documented workflow: kinematic baseline → physics gains |

### Scope by robot

#### Dubins race

- Map Pure Pursuit outputs to agent body actuators (velocity + steering)
- Column and floor contact response
- Optional inter-agent contact / blocking
- Both agents reach goal B without penetration through structures

#### Shared infrastructure

- `physics_mode` ROS parameter on `MuJoCoBridgeNode`
- Contact force logging and sim-time metrics
- Regression clips when physics path diverges from kinematic baseline
- CI smoke tests with `physics_mode:=true` for Dubins

Full integration spec: [mujoco.md](mujoco.md) ·
[v1.2 implementation spec](mujoco_physics_v1.2.md).

### Scenario IDs

Physics validation runs against the shipped release scenario:

| ID | Scenario | Model |
|---|---|---|
| SC-v11 | `dubins_race.yml` | `dubins` |

### Acceptance criteria

| # | Criterion |
|---|---|
| V12-1 | `physics_mode:=true` SITL launches for Dubins without error |
| V12-2 | Dubins agents reach B with column contact response; no ghosting through walls |
| V12-3 | `/joint_states` timestamps match sim clock; no open-loop pose injection |
| V12-4 | Contact log artifact produced in CI |
| V12-5 | Controller tuning guide in [mujoco.md](mujoco.md) |
| V12-6 | Physics regression tests in `tests/integration/` |

### Implementation tasks

| ID | Task |
|---|---|
| T12-01 | `step_physics()` in `MuJoCoBridgeCore` — `ctrl` → `mj_step` |
| T12-02 | Dubins MJCF actuators + steering model |
| T12-03 | Contact logging harness + metrics |
| T12-04 | Physics integration tests (Dubins) |
| T12-05 | Update showcase scripts to support physics mode (optional flag) |
| T12-06 | Document tuning workflow in [mujoco.md](mujoco.md) and [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md) |

---

## v1.3 — RRP / SCARA (ARCO reproduction)

### Goal

Reproduce ARCO **`rrp`** and **`rr`** scenarios inside the FRET ROS 2 pipeline — the
same pillar/slab obstacles, joint-space planning, and race-style execution that ARCO
CI already validates. All execution runs on **MuJoCo physics SITL** (v1.2 foundation).

### Robots

| Model | DOF | ARCO map |
|---|---|---|
| `rrp` | 3 (θ₁, θ₂, z) | `map/rrp.yml` |
| `rr` | 2 (θ₁, θ₂) | `map/rr.yml` |

Leverages existing FRET SCARA bootstrap (`src/fret/control/kinematics.py`, MS-1–5 code).

### Scenario IDs

| ID | File | Description |
|---|---|---|
| SC-v13a | `rrp_pillars.yml` | 3-D pillars + slabs (ARCO rrp) |
| SC-v13b | `rr_planar.yml` | 2-D planar arm (ARCO rr) |

### Acceptance criteria

| # | Criterion |
|---|---|
| V13-1 | RRP scenario matches ARCO `rrp.yml` obstacle layout and pass/fail semantics |
| V13-2 | EE tracking error ≤ 5 mm (RRP) on MuJoCo physics SITL |
| V13-3 | RR planar scenario passes in pure-Python CI |
| V13-4 | Side-by-side video: ARCO arcosim vs FRET SITL (same scenario) |

### Implementation tasks

| ID | Task |
|---|---|
| T13-01 | MJCF model for RRP arm + pillar world |
| T13-02 | Extend `mujoco_bridge` for revolute + prismatic actuators |
| T13-03 | Port ARCO `rrp.yml` → `rrp_pillars.yml` |
| T13-04 | Port ARCO `rr.yml` → `rr_planar.yml` |
| T13-05 | MuJoCo physics SITL smoke + comparison video |

---

## v1.4 — 6-DOF manipulator (final challenge)

### Goal

A **6-DOF revolute manipulator** (e.g. UR5 or custom arm) performs C-space planning
and trajectory execution in a cluttered environment — the capstone release.

### Scope (high level)

| Item | Detail |
|---|---|
| Model | 6 revolute joints; MJCF (import from Menagerie or URDF) |
| IK | Numerical IK (Jacobian pseudoinverse or analytic where available) |
| Planning | ARCO SST in 6-D C-space |
| Collision | Per-link FK + KDTree clearance; self-collision in MJCF |
| Environment | Configurable obstacle field (tabletop or cell) |
| Simulation | MuJoCo physics SITL from day one |

### Scenario ID

**SC-v14** — `config/scenarios/six_dof_challenge.yml` *(to be created)*

### Acceptance criteria

| # | Criterion |
|---|---|
| V14-1 | Collision-free 6-D path planned within 60 s |
| V14-2 | EE reaches goal with ≤ 5 mm error under physics SITL |
| V14-3 | Self-collision checking enabled |
| V14-4 | Demo video + benchmark table (planning time, path length) |

*Detailed task breakdown will be written when v1.3 is complete.*

---

## Version tagging

| Tag | Content | Prerequisite |
|---|---|---|
| `v0.9.0` | Bootstrap SCARA pipeline (MS-1–5) | ✅ Done |
| `v1.0.0` | Superseded bootstrap tag (not a product showcase) | — |
| `v1.1.0` | Dubins dual race — **first product showcase** | T11-* |
| `v1.1.x` | Physics-bridge iterations (v1.1 → v1.2); no new robots | See [§ v1.1.x retrospective](#v11x--v12-retrospective) below |
| `v1.2.0` | MuJoCo physics SITL (Dubins) — **current** | T12-* ✅ |
| `v1.3.0` | RRP + RR ARCO reproduction | T13-* |
| `v1.4.0` | 6-DOF challenge | T14-* |

### v1.1.x → v1.2.0 retrospective

Intermediate tags between v1.1.0 and v1.2.0 landed physics SITL incrementally
without new showcase scenarios. Kinematic release MP4s were default until
v1.2.0; physics clips used `--physics-mode` during development.

| Tag | Focus | Status |
|---|---|---|
| `v1.1.1` | Physics bridge checkpoint (#88) | Superseded |
| `v1.1.2` | MJCF collision policy + physics tracking baseline | ✅ |
| `v1.1.3` | Floor-contact / contact-log handoff | ✅ |
| `v1.1.4` | Dubins RTF + tracking gates | ✅ |
| `v1.1.5` | Regression harness + CI hardening | ✅ |
| `v1.2.0` | Product release — physics-default showcase | ✅ |

**v1.1.x rules (historical):** no new robots or scenarios; each tag CI-green;
kinematic behaviour must not regress.

**Resolved physics gaps at v1.2.0:**

- **Dubins (SC-v11):** both agents reach goal under physics; race duration
  ~1.73× kinematic on CI.
- **Release pipeline:** `release.yml` uses `--physics-mode`.

**Deferred to v1.3+** (see [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md)):

- Race showcase controller RTF / finish-time tuning on true TB3 wheel actuators
- New robots (RRP, 6-DOF) and hardware HITL

Every release-tag MP4 (Dubins, all camera POVs) **must** play back at
**real-time simulation speed**. This is a hard product requirement, independent
of kinematic vs physics render mode.

Pipeline (`scripts/render_mujoco.py`, invoked by `scripts/video.sh` and
`.github/workflows/release.yml`):

1. Record frames at fixed `fps`; capture `sim_time_s` (simulated motion
   duration) and `render_duration_s` (`frame_count / fps`).
2. Compute `real_time_factor = render_duration_s / sim_time_s`.
3. Post-process with **ffmpeg** (`setpts=PTS/rtf`) so on-screen motion matches
   `sim_time_s`. Skip only when `rtf ≈ 1`.

**Rules:**

- Release CI must **not** pass `--no-realtime-postprocess`.
- `--timing-json` is required on release renders (persists RTF per clip in
  `meta.json`).
- Development/debug renders may use `--no-realtime-postprocess`, but uploaded
  R2 showcase assets must always be real-time adjusted.

Until v1.2.0, release CI used `--kinematic-mode`; from v1.2.0 onward it uses
`--physics-mode`. The real-time post-process step applies in both cases.

See [mujoco.md § Showcase rendering](mujoco.md#showcase-rendering-real-time-playback).

---

## Deprecated / removed

The following documents and goals are **superseded** by this release spec:

- MS-6 / MS-7 milestone framing (MuJoCo + TBD showcase)
- SCARA-as-v1.0-showcase
- SC-01 – SC-05 as primary release scenarios (retained as regression tests only)
- Platform study 2026 Q3 dual-demo proposal

Regression scenarios (SC-01 – SC-05) remain in CI for the bootstrap SCARA pipeline
until v1.3 replaces them as the canonical arm tests.
