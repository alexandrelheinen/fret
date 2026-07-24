# FRET Release Specification

> **Authoritative product roadmap.** All requirements, scenarios, and milestones
> trace to this document. High-level eras: [roadmap.md](roadmap.md).
>
> **Related:** [requirements.md](requirements.md) ·
> [scenarios.md](scenarios.md) · [vision/README.md](vision/README.md) ·
> [mujoco.md](mujoco.md) · [README § Architecture](../README.md#architecture)

---

## Product vision

FRET is a **ROS 2 full-stack robotics framework** that connects **ARCO** planning
and control to MuJoCo simulation, then (from **v2.x**) to hardware. Releases are
grouped by era:

| Era | Versions | What ships |
| --- | --- | --- |
| **Simulation & algorithms** | **v1.x** | Planners, controllers, MuJoCo SITL, computer vision *in simulation* |
| **Hardware integration** | **v2.x** | Modular HITL (bridge → real cameras → real arm → full loop) |
| **Product** | **v3.0+** | Definitive integrated system (north-star only in this doc) |

### Release matrix (simulation line)

| Release | Robot / focus | Scenario theme | Backend |
|---|---|---|---|
| **v1.0** | *(superseded)* | Bootstrap — not a product showcase | — |
| **v1.1** | Dubins / TB3 × 2 | Dual-robot race A→B (ARCO → MuJoCo case study) | MuJoCo |
| **v1.2** | Dubins physics | Actuator-driven SITL with contacts | MuJoCo |
| **v1.2.3** | OpenMANIPULATOR-X | Tabletop reach → pick-place → Γ-wall maze | MuJoCo |
| **v1.2.4** | OpenMANIPULATOR-Y | 6-DOF reach → pick-place → clutter | MuJoCo |
| **v1.3** | Vision algorithms | Ball track pipeline + **algorithm selection** | Fixtures (+ optional MuJoCo frames) |
| **v1.4** | OM-X / OMY + CV | MuJoCo cameras → replace hardcoded ball pose | MuJoCo + CV |
| **v1.5** | Dynamic manipulation | Rolling ball, pickability, industrial place | MuJoCo + CV |

**TB3 / Dubins** never consumes CV — it exists to prove ARCO SE(2) racing under
MuJoCo. **Only manipulators** use vision (they interact with graspable objects).

**Platform stack (v1.x):**

- **Planning / control:** ARCO (RRT*, SST, occupancy, path-following + joint MPC)
- **Middleware:** ROS 2 Jazzy
- **Simulation:** MuJoCo (physics, contacts, cameras, rendering, SITL)
- **Vision (from v1.3):** `fret.vision` — pure-Python pipeline; ROS I/O thin

---

## Engineering foundation

Early bootstrap work validated the planning/control pipeline. Product robots are
drawn from the **ROBOTIS MuJoCo Menagerie** submodule (TurtleBot3,
OpenMANIPULATOR-X/Y, …). Legacy SCARA/RRP assets are retired.

**Decisions already locked for the CV line** (detail in
[vision/](vision/README.md)):

| Topic | Decision |
| --- | --- |
| CV consumers | Manipulators only (`open_manipulator_x`, `omy`) |
| Ball pose | From vision (v1.4+); no hardcoded ball / `pick_xy` in release paths |
| Place / dispenser | **Known scenario parameter** (fixed fixture) — not required from CV |
| v1.3 algorithm | **HSV blob + table-plane lift** (OpenCV); contracts remain swappable |
| v1.5 ball cadence (provisional) | **One ball at a time** for the release scenario; multi-ball deferred |

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

A **6-DOF revolute manipulator** (Menagerie OpenMANIPULATOR-Y) performs C-space
planning and trajectory execution in a cluttered environment — the last
**pre-vision** manipulator showcase on the v1.2.x line. *(Shipped.)*

### Scope (high level)

| Item | Detail |
|---|---|
| Model | 6 revolute joints; MJCF from Menagerie submodule (OMY) |
| IK | Numerical IK (Jacobian pseudoinverse or analytic where available) |
| Planning | ARCO SST in 6-D C-space |
| Collision | Per-link FK + KDTree clearance; self-collision in MJCF |
| Environment | Configurable obstacle field (tabletop or cell) |
| Simulation | MuJoCo physics SITL from day one |

### Scenario IDs

| ID | File | Description |
|---|---|---|
| SC-v14a | `omy_reach.yml` | Empty tabletop joint-space A→B |
| SC-v14b | `omy_pick_place.yml` | Floor ball pick → cone place (FSM + MuJoCo physics) |
| SC-v14c | `omy_clutter.yml` | Mid-cell wall forces planned transfer detour |

### Acceptance criteria

| # | Criterion |
|---|---|
| V124-1 | Empty-cell A→B reaches goal joint configuration under MuJoCo physics |
| V124-2 | Ground pick-and-place FSM places ball in cone (physics smoke in CI) |
| V124-3 | Cluttered cell plans a detour (straight joint line collides; path length ≥ 2) |
| V124-4 | Robot from Menagerie OMY; floor ball + tip-down place cone at Menagerie scale |

### Implementation tasks

| ID | Task |
|---|---|
| T124-01 | MJCF cells wrapping Menagerie OMY (tabletop, pick-place, clutter) |
| T124-02 | Kinematics + controller + bridge wiring for OMY joints |
| T124-03 | SC-v14a scenario + unit/physics smoke |
| T124-04 | Ground pick-place FSM + ball/cone MJCF + SC-v14b physics smoke |
| T124-05 | Mid-cell wall + planner/controller SC-v14c detour |

---

## v1.3 — Computer vision pipeline ✅

### Goal

Ship a **pure-Python computer-vision pipeline** that can track a graspable ball
in images from **one or more cameras**, with unit tests, written specs, and an
**explicit algorithm selection** for that goal. No manipulation closed loop and
no requirement for production MJCF camera mounts in this tag — those are v1.4.

Architecture: [vision/architecture.md](vision/architecture.md).  
Selection ADR: [vision/algorithm-selection.md](vision/algorithm-selection.md).

### Scope

| Item | Detail |
|---|---|
| Package | `fret.vision` (algorithm layer; no ROS imports in core) |
| Goal | Detect / track ball centre; lift to metric pose when calibration + geometry allow |
| Cameras | API supports N≥1; MVP uses 1 overhead view |
| Primary algo | OpenCV HSV blob + table-plane ray lift |
| Consumers | Spec’d for manipulators; **TB3 out of scope** |
| Place pose | Out of scope (known parameter later) |
| Tests | Synthetic fixtures under `tests/vision/` (+ optional web gallery script) |

### Scenario IDs

| ID | Description |
|---|---|
| SC-v15 | Vision unit / fixture scenarios (algorithm gates; not a robot showcase) |

### Acceptance criteria

| # | Criterion | Status |
|---|---|---|
| V13-1 | `FR-VIS-*` requirements and `docs/modules/vision.md` API stubs published | ✅ |
| V13-2 | Algorithm selection document records candidates, metrics, and **chosen primary** | ✅ |
| V13-3 | Unit tests: ball centre error ≤ threshold on fixtures (see selection doc) | ✅ |
| V13-4 | Pipeline runs without ROS; optional thin ROS wrapper may exist but is not required | ✅ |
| V13-5 | Multi-camera interface is defined even if the primary algo starts monocular | ✅ |

### Implementation tasks

| ID | Task | Status |
|---|---|---|
| T13-01 | Scaffold `fret.vision` + typed contracts (`BallObservation`, `CameraFrame`, …) | ✅ |
| T13-02 | Candidate implementations behind a common detector/tracker interface | ✅ HSV + plane |
| T13-03 | Fixture corpus + unit gates; record selection decision | ✅ |
| T13-04 | Module + interface docs; link from roadmap / scenarios | ✅ |

---

## v1.4 — CV ↔ manipulation integration

### Goal

Connect the v1.3 vision pipeline to **existing OM-X / OMY pick-and-place** so
release behaviours match today’s physics smoke **without hardcoded ball
positions**. Simulate cameras in MuJoCo with a **realistic mount**. The
**place / dispenser** pose remains a known scenario parameter.

### Scope

| Item | Detail |
|---|---|
| Robots | `open_manipulator_x`, `omy` |
| Cameras | MJCF `<camera>` on a portal / gantry (or better mount from selection needs) |
| Ball pose | From `BallObservation` → FSM / IK / planner entry |
| Place pose | YAML known parameter (fixed industrial fixture) |
| Equivalence | Same DONE / cone-place success as pre-CV scenarios on seeded runs |

### Scenario IDs

| ID | File (planned) | Description |
|---|---|---|
| SC-v16a | `omx_pick_place_cv.yml` (name TBD) | OM-X pick-place driven by CV ball pose |
| SC-v16b | `omy_pick_place_cv.yml` (name TBD) | OMY analogue |
| SC-v16c | clutter variants | Optional: clutter + CV ball (place still known) |

### Acceptance criteria

| # | Criterion |
|---|---|
| V14-1 | Scenario MJCF includes calibrated sim cameras with documented extrinsics |
| V14-2 | Release pick-place paths do not read hardcoded ball / `pick_xy` as ground truth |
| V14-3 | Place / dispenser pose comes only from scenario parameters |
| V14-4 | Physics smoke: FSM reaches DONE and ball enters place volume (seeded) |
| V14-5 | Behaviour parity checklist vs SC-v13b / SC-v14b on shared seeds |

### Implementation tasks

| ID | Task |
|---|---|
| T14-01 | Camera mount MJCF + calibration YAML for OM-X / OMY cells |
| T14-02 | MuJoCo image adapter → `fret.vision` |
| T14-03 | Replace hardcoded ball pose in runners with vision output |
| T14-04 | SC-v16 scenarios + tests; update showcase matrix when clips are ready |

---

## v1.5 — Dynamic ball + industrial place

### Goal

Upgrade the manipulation cell toward a more **industrial** workflow: a ball
**rolls on the floor** and stops at a **random** pose; vision **detects** it and
labels it **pickable or not**; if pickable, run the existing pipeline into an
improved **container / dispenser**. Asset research is part of the release.

### Scope

| Item | Detail |
|---|---|
| Delivery | Rolling ball → random rest pose (MuJoCo contacts) |
| Classification | Pickable vs reject (workspace, size, rest stability — exact rules in FR-VIS) |
| Place geometry | Survey Menagerie / AWS / other assets; ship improved container mesh |
| Cadence | **Provisional:** one ball per cycle for the release scenario; multi-ball optional stretch |
| TB3 | Still out of CV scope |

### Scenario IDs

| ID | Description |
|---|---|
| SC-v17 | Dynamic ball delivery + pickability + place into improved container |

### Acceptance criteria

| # | Criterion |
|---|---|
| V15-1 | Written asset survey (container / dispenser candidates) with chosen mesh |
| V15-2 | Ball released / rolled; final XY not scripted as the pick target |
| V15-3 | Vision emits pickability; non-pickable balls do not start grasp |
| V15-4 | Pickable balls are placed in the improved container under physics |
| V15-5 | Documented decision: single-ball release vs multi-ball (default: single) |

### Implementation tasks

| ID | Task |
|---|---|
| T15-01 | Asset research note under `docs/vision/` + MJCF container upgrade |
| T15-02 | Ball emitter / roll-out mechanism in MJCF |
| T15-03 | Pickability classifier + SC-v17 scenario + tests |
| T15-04 | Cadence decision recorded; multi-ball deferred or stretch-tagged |

---

## v2.x — Hardware integration line

### Goal

After **v1.5** is validated in simulation, bring the same contracts onto
**physical hardware** in **modular** steps. Each minor tag may ship one layer.

| Step | Focus | Notes |
|---|---|---|
| **v2.0** | Low-level target bridge | Micro-ROS / serial; `/joint_commands` ↔ drivers; encoders → `/joint_states` |
| **v2.1** | Real image pipeline | Same `fret.vision` interfaces; real camera drivers |
| **v2.2** | Real manipulator actuation | Closed-loop joint tracking on hardware |
| **v2.3** | Full HITL pick-and-place | Vision + plan + control + hardware together |

Targets (indicative): Raspberry Pi 5 + Arduino Mega + Nema-class actuators;
exact BOM frozen when v2.0 planning opens.

Module stub today: [modules/hardware.md](modules/hardware.md) (retargeted to v2.x).

### Acceptance criteria (era-level)

| # | Criterion |
|---|---|
| V2x-1 | Each step has its own tag, tests, and does not require the next step to merge |
| V2x-2 | Algorithm packages (`fret.vision`, planning, control) stay simulator-agnostic |
| V2x-3 | At least one end-to-end hardware pick-place smoke before declaring the era done |

*Detailed T2x-* task lists are written when v1.5 closes.*

---

## v3.0 — Definitive product (north-star)

**v3.0** is the release where simulation-proven algorithms and v2.x hardware
modules are validated together as the **default product**. After that, work is
incremental (debugs, features). **No task breakdown for 3.0+ is maintained in
this repository documentation** — see [roadmap.md](roadmap.md).

---

## Version tagging

| Tag | Content | Prerequisite |
|---|---|---|
| `v1.0.0` | Superseded bootstrap tag | — |
| `v1.1.0` | Dubins dual race — first product showcase | T11-* ✅ |
| `v1.1.x` | Physics-bridge iterations | See [§ retrospective](#v11x--v12-retrospective) |
| `v1.2.0` | MuJoCo physics SITL (Dubins) | T12-* ✅ |
| `v1.2.3` | OpenMANIPULATOR-X tabletop | T123-* ✅ |
| `v1.2.4` | OMY 6-DOF challenge | T124-* ✅ |
| `v1.2.5` | Patch: OMY clutter showcase detour | — ✅ |
| `v1.2.6` | Patch: telemetry on R2 (FR-SIM-12) | — ✅ |
| `v1.2.7` | Patch: Dubins showcase encode / clip length | — ✅ |
| `v1.3.0` | **CV pipeline + algorithm selection** | T13-* ✅ |
| `v1.4.0` | CV ↔ manipulation + MuJoCo cameras | T14-* |
| `v1.5.0` | Dynamic ball + industrial place | T15-* |
| `v2.0.0+` | Hardware line (modular) | T2x-* |
| `v3.0.0` | Definitive product (north-star) | — |

Python package on `main` is **1.3.0** for the open CV development line
(`pyproject.toml`). Product tags remain `v1.3.0`, `v1.4.0`, `v1.5.0`, then
`v2.x` / `v3.0.0`.

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

**Historical “deferred” items (status today):**

| Item | Status |
|---|---|
| OMY 6-DOF (was “deferred to v1.2.4+”) | ✅ Shipped |
| Hardware HITL (was “deferred to v1.3+”) | **Moved to v2.x** — v1.3 is CV |
| Optional TB3 race finish-time polish | Non-blocking; TB3 accepted as case study |

### Release showcase videos (v1.2.3+)

Canonical matrix: [`src/fret/config/release/showcase.yml`](../src/fret/config/release/showcase.yml).

| Robot class | Scenarios (today) | Required cameras |
|---|---|---|
| **mobile** | `dubins_race` (TB3); future mobile robots | `overview` (isometric); `follow` optional / local |
| **static** | OM-X Γ-maze: `omx_wall_maze_rrt` + `omx_wall_maze_sst`; OMY pick-place `omy_pick_place` + clutter `omy_clutter_rrt` + `omy_clutter_sst` | `overview` only |

OM-X and OMY clutter release clips share the same wall geometry and joint
tracking; they differ only in the transfer planner (**RRT\*** vs **SST**).
Simpler OM-X / OMY demos (`omx_reach`, `omy_reach`, desk clutter) stay in the
repo for development but are not release artifacts.

**Rules:**

- **Overview** must frame the robots and ideally start + goal (tune zoom /
  azimuth per scenario MJCF / runtime camera constants).
- **Follow** is optional for mobile and **not** on the blocking release path.
  Dubins ships overview-only at `fps: 20`, `960×540`, `clip_duration_s: 40`
  (full ~38 s race + short hold → 40 s video; 800 frames) so the encode job
  stays under ~10 min. A separate non-blocking follow job was considered and
  **deferred**: a second camera roughly doubles encode wall time. Render
  follow locally when needed:

  ```bash
  ./scripts/video.sh --model dubins --scenario dubins_race \
    --camera follow --full-duration --video-duration 40 --fps 20 \
    --width 960 --height 540 --physics-mode \
    --collision-backend mujoco --planner-algorithm sst \
    -o /tmp/dubins_race_follow.mp4
  ```

  Static / tabletop arms must **not** export follow on release.
- Dubins release uses `clip_duration_s` as the **video** length (full race +
  hold/trim). Optional `clip_scale` remains for scale×sim_time clips.
  Full-race acceptance stays in pytest.
- Dubins race uses `--physics-mode`; OM-X clips step MuJoCo actuators.
- Only clean semver tags (`vX.Y.Z`) update `latest/`. Suffixed tags
  (`-dev`, probes, …) upload under `releases/<tag>/` only.
- Showcase renders also write matching telemetry
  (`<scenario>_overview.csv` + `.json`) next to the overview MP4; the
  R2 uploader publishes them beside the videos (FR-SIM-12). Download with
  `./scripts/download_showcase.sh --tag v1.2.6 --with-telemetry`.

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

**v1.4+ note:** showcase / SITL **perception cameras** (ball tracking) are
distinct from release **overview** encode cameras. Perception mounts are
specified under [vision/camera-layout.md](vision/camera-layout.md).

---

## Deprecated / removed

The following documents and goals are **superseded** by this release spec:

- MS-6 / MS-7 milestone framing (MuJoCo + TBD showcase)
- SC-01 – SC-05 bootstrap scenarios (retired with SCARA/RRP purge)
- Platform study 2026 Q3 dual-demo proposal
- **v1.3 as Hardware HITL** — hardware moved to **v2.x**; v1.3 is the CV pipeline
- “Vision post-v1.3” framing in older roadmap copies