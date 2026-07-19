# FRET Release Specification (v1.0 → v1.3)

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
| **v1.2.3** | OpenMANIPULATOR-X (4-DOF) | Tabletop A→B → pick-place → Γ-wall maze | MuJoCo |
| **v1.2.4** | 6-DOF manipulator | Cluttered-cell C-space planning + execution | MuJoCo |
| **v1.3** | Hardware HITL | Micro-ROS bridge + physical prototype | Pi 5 + Arduino |

**Platform stack (all releases):**

- **Planning:** ARCO (RRT*, SST, KDTree occupancy, trajectory pruner)
- **Middleware:** ROS 2 Jazzy
- **Simulation:** MuJoCo (physics, contacts, rendering, SITL)

---

## Engineering foundation

Early bootstrap work validated the planning/control pipeline. Product robots are
drawn from the **ROBOTIS MuJoCo Menagerie** submodule (TurtleBot3, OpenMANIPULATOR-X,
…). Legacy SCARA/RRP assets are retired.

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
| Floor | 10 m × 10 m lab plane (AWS ground texture) |
| Structures | Narrow aisles + alcoves sized for real TurtleBot3 Burger |
| Start A | (1.2, 1.2) |
| Goal B | (8.8, 8.8) |
| Visual | AWS clutter meshes on analytic boxes; real-scale TB3 |

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
| V11-2 | Race video shows simultaneous motion (split-screen **follow** + shared-world **overview**) |
| V11-3 | Dubins curvature constraint respected (min turning radius) |
| V11-4 | MP4 artifacts uploaded on release tag (overview + follow) |

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
simulation foundation required for v1.2.x arm releases.

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

## v1.2.3 — OpenMANIPULATOR-X tabletop ✅

### Goal

A **Menagerie OpenMANIPULATOR-X** (4-DOF + gripper) performs tabletop manipulation:
empty-cell A→B (command chain), then **pick-and-place** (FSM + grasp physics on a
Ø 25 mm ball → tip-down place cone), then **desk clutter** forcing a detour.
Warehouse meshes stay for clutter only — they are far too large for the OM-X
gripper (~3 cm aperture).

### Robot

| Model | DOF | Source |
|---|---|---|
| `open_manipulator_x` | 4 + gripper | `third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x` |

### Scenario IDs

| ID | File | Description |
|---|---|---|
| SC-v13a | `omx_reach.yml` | Empty tabletop, EE pose A → B |
| SC-v13b | `omx_pick_place.yml` | Stretch pick green→red (FSM + MuJoCo physics) |
| SC-v13c | `omx_desk_clutter.yml` | Mid-cell wall forces planned retract detour |
| SC-v13d | `omx_wall_maze.yml` | Γ (inverted-L) wall forces retract → climb → place |

### Acceptance criteria

| # | Criterion |
|---|---|
| V123-1 | Empty-cell A→B reaches goal under MuJoCo physics SITL (EE error ≤ 5 mm) |
| V123-2 | Pick-and-place FSM moves a free ball from green pedestal into the red place cone without drop mid-transfer |
| V123-3 | Cluttered cell plans a collision-free detour (not a straight joint-space line) |
| V123-4 | Showcase videos: Γ-maze isometric **overview** for RRT* and SST (same MPC tracking; no follow) |
| V123-5 | Robot from Menagerie; pick object is a plain MuJoCo ball; place is a cone funnel |
| V123-6 | Γ-wall maze path retracts and climbs over the cap before placing |

### Implementation tasks

| ID | Task |
|---|---|
| T123-01 | MJCF cell wrapping Menagerie OM-X (empty tabletop) |
| T123-02 | Kinematics + controller + bridge wiring for OM-X joints |
| T123-03 | SC-v13a scenario + unit/physics smoke |
| T123-04 | PickPlace FSM + ball/cone MJCF + SC-v13b physics smoke |
| T123-05 | Mid-cell wall + planner/controller SC-v13c detour (AWS later) |
| T123-06 | Showcase render pipeline + metrics |
| T123-07 | Γ-wall maze SC-v13d (stem+cap occupancy + climb filter) |

---

## v1.2.4 — 6-DOF manipulator challenge

### Goal

A **6-DOF revolute manipulator** (Menagerie OpenMANIPULATOR-Y preferred) performs C-space planning
and trajectory execution in a cluttered environment — the simulation capstone before hardware.

### Scope (high level)

| Item | Detail |
|---|---|
| Model | 6 revolute joints; MJCF from Menagerie submodule (OMY) |
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
| V124-1 | Collision-free 6-D path planned within 60 s |
| V124-2 | EE reaches goal with ≤ 5 mm error under physics SITL |
| V124-3 | Self-collision checking enabled |
| V124-4 | Demo video + benchmark table (planning time, path length) |

*Detailed task breakdown (T124-*) will be written when v1.2.3 is tagged.*

---

## v1.3 — Hardware HITL

### Goal

Close the loop from FRET's ROS 2 stack to a **physical prototype**: Raspberry Pi 5 high-level
control, Arduino Mega low-level actuation, Micro-ROS serial bridge, and encoder feedback.
Ships after simulation milestones (v1.2.4) are debugged and stable.

### Scope (high level)

| Item | Detail |
|---|---|
| Bridge | `hardware/bridge_node.py` — `/joint_commands` → serial → motor drivers |
| Feedback | Encoder data → `/joint_states` at ≥ 50 Hz |
| Transport | Micro-ROS preferred; custom ASCII fallback |
| Targets | Pi 5 + Arduino Mega + Nema 17 class steppers |

### Acceptance criteria

| # | Criterion |
|---|---|
| V13-1 | Bridge relays validated joint commands without corruption |
| V13-2 | Encoder feedback published at ≥ 50 Hz |
| V13-3 | End-to-end HITL smoke on at least one manipulator joint |

*Detailed task breakdown (T13-*) will be written when v1.2.4 planning starts.*

---

## Version tagging

| Tag | Content | Prerequisite |
|---|---|---|
| `v1.0.0` | Superseded bootstrap tag (not a product showcase) | — |
| `v1.1.0` | Dubins dual race — **first product showcase** | T11-* |
| `v1.1.x` | Physics-bridge iterations (v1.1 → v1.2); no new robots | See [§ v1.1.x retrospective](#v11x--v12-retrospective) below |
| `v1.2.0` | MuJoCo physics SITL (Dubins) | T12-* ✅ |
| `v1.2.3` | OpenMANIPULATOR-X tabletop showcase (mobile + OM-X videos) — **current** | T123-* ✅ |
| `v1.2.4` | 6-DOF challenge | T124-* |
| `v1.3.0` | Hardware HITL | T13-* |

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

**Deferred to v1.2.4+** (see [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md)):

- Race showcase controller RTF / finish-time tuning on true TB3 wheel actuators
- 6-DOF manipulator (v1.2.4)

**Deferred to v1.3+:**

- Hardware HITL (Micro-ROS bridge)

### Release showcase videos (v1.2.3+)

Canonical matrix: [`src/fret/config/release/showcase.yml`](../src/fret/config/release/showcase.yml).

| Robot class | Scenarios (today) | Required cameras |
|---|---|---|
| **mobile** | `dubins_race` (TB3); future mobile robots | `overview` (isometric) **and** `follow` (split-screen chase) |
| **static** | SC-v13d Γ-maze only: `omx_wall_maze_rrt` + `omx_wall_maze_sst` | `overview` only |

OM-X release clips share the same Γ-wall geometry and JointPathMPC / joint
control tracking; they differ only in the transfer planner (**RRT\*** vs
**SST**). Simpler OM-X demos (`omx_reach`, pick-place, desk clutter) stay in
the repo for development but are not release artifacts.

**Rules:**

- **Overview** must frame the robots and ideally start + goal (tune zoom /
  azimuth per scenario MJCF / runtime camera constants).
- **Follow** is mandatory for every **mobile** robot showcase, including future
  mobile platforms. Static / tabletop arms must **not** export follow on release.
- Dubins race uses `--physics-mode`; OM-X clips step MuJoCo actuators.
- Tags containing `-dev` upload under `releases/<tag>/` only and do **not**
  update `latest/`.

Orchestration: `scripts/release/render_showcase.py` +
`.github/workflows/release.yml`.

Every release-tag MP4 **must** play back at **real-time simulation speed**.
This is a hard product requirement, independent of kinematic vs physics render
mode.

Pipeline (`scripts/render_mujoco.py`, invoked by `scripts/video.sh` and
`.github/workflows/release.yml`):

1. Record frames at fixed `fps`; capture `sim_time_s` (simulated motion
   duration) and `render_duration_s` (`frame_count / fps`).
2. Compute `real_time_factor = render_duration_s / sim_time_s`.
3. Post-process with **ffmpeg** (`setpts=PTS/rtf`) so on-screen motion matches
   `sim_time_s`. Skip only when `rtf ≈ 1`.

**RTF rules:**

- Release CI must **not** pass `--no-realtime-postprocess`.
- `--timing-json` is required on release renders (persists RTF per clip in
  `meta.json`).
- Development/debug renders may use `--no-realtime-postprocess`, but uploaded
  R2 showcase assets must always be real-time adjusted.

Until v1.2.0, release CI used `--kinematic-mode`; from v1.2.0 onward Dubins
uses `--physics-mode`. The real-time post-process step applies in both cases.

See [mujoco.md § Showcase rendering](mujoco.md#showcase-rendering-real-time-playback).

---

## Deprecated / removed

The following documents and goals are **superseded** by this release spec:

- MS-6 / MS-7 milestone framing (MuJoCo + TBD showcase)
- SC-01 – SC-05 bootstrap scenarios (retired with SCARA/RRP purge)
- Platform study 2026 Q3 dual-demo proposal
