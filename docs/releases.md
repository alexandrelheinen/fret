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

### Release matrix

| Release | Robot / focus | Scenario theme | Backend | Status |
|---|---|---|---|---|
| **v1.1** | Dubins / TB3 × 2 | Dual-robot race A→B | MuJoCo | ✅ |
| **v1.2** | Dubins physics | Actuator-driven SITL with contacts | MuJoCo | ✅ |
| **v1.2.3** | OpenMANIPULATOR-X | Tabletop reach → pick-place → Γ-wall maze | MuJoCo | ✅ |
| **v1.2.4** | OpenMANIPULATOR-Y | 6-DOF reach → pick-place → clutter | MuJoCo | ✅ |
| **v1.3** | Vision algorithms | Ball track pipeline + algorithm selection | Fixtures (+ optional MuJoCo frames) | ✅ |
| **v1.4** | OM-X / OMY + CV | MuJoCo cameras → replace hardcoded ball pose | MuJoCo + CV | ✅ |
| **v1.5.0** | Robot-model-agnostic core | Registry-based robot dispatch; hardware-neutral docs; hygiene | MuJoCo | ✅ |
| **v2.0.0** | SO-101 | Hardware study + SO-101 MuJoCo simulation twin | MuJoCo | ✅ |
| **v2.x** | Hardware HITL | Modular bridge → cameras → arm → full loop | Hardware | 🔲 |
| **v3.0** | Product | Definitive integrated system | Sim + HW | ○ |

**TB3 / Dubins** never consumes CV — it exists to prove ARCO SE(2) racing under
MuJoCo. **Only manipulators** use vision (they interact with graspable objects).

**Platform stack (v1.x):**

- **Planning / control:** ARCO (RRT*, SST, occupancy, path-following + joint MPC)
- **Middleware:** ROS 2 Jazzy
- **Simulation:** MuJoCo (physics, contacts, cameras, rendering, SITL)
- **Vision:** `fret.vision` — pure-Python pipeline; ROS I/O thin

---

## Engineering foundation

Product robots are drawn from the **ROBOTIS MuJoCo Menagerie** submodule
(TurtleBot3, OpenMANIPULATOR-X/Y, …).

**Decisions locked for the CV line** (detail in [vision/](vision/README.md)):

| Topic | Decision |
| --- | --- |
| CV consumers | Manipulators only (`open_manipulator_x`, `omy`) |
| Ball pose | From vision; no hardcoded ball / `pick_xy` in release paths |
| Place / dispenser | **Known scenario parameter** (fixed fixture) — not required from CV |
| Primary algorithm | **HSV blob + table-plane lift** (OpenCV); contracts remain swappable |
| Dynamic-ball cadence (provisional, simulation backlog) | **One ball at a time** for the release scenario; multi-ball deferred |

---

## Shipped releases

Acceptance criteria below remain the regression contract. Implementation task
lists for closed lines are not maintained here — see git history and
[roadmap.md](roadmap.md) for the shipped summary.

### v1.1 — Dubins dual-robot race ✅

Two **Dubins vehicles** race A→B through rectangular structures (posts, walls,
alcoves), reproducing the ARCO `vehicle` race format (RRT* vs SST).

| | |
|---|---|
| Model | `dubins` (SE(2); ARCO Pure Pursuit + DubinsVehicle) |
| Scenario | **SC-v11** — `config/scenarios/dubins_race.yml` |
| Environment | 10 m × 10 m lab; start (1.2, 1.2); goal (8.8, 8.8) |

| # | Criterion |
|---|---|
| V11-1 | Two robots plan independently; both reach B without collision |
| V11-2 | Race video shows simultaneous motion (follow + overview) |
| V11-3 | Dubins curvature constraint respected |
| V11-4 | MP4 artifacts uploaded on release tag |

### v1.2 — MuJoCo physics SITL ✅

Upgrade from kinematic mirroring to actuator-driven MuJoCo physics for the
Dubins showcase. Spec: [mujoco.md](mujoco.md).

| # | Criterion |
|---|---|
| V12-1 | `physics_mode:=true` SITL launches for Dubins without error |
| V12-2 | Agents reach B with column contact response; no ghosting through walls |
| V12-3 | `/joint_states` from sim clock; no open-loop pose injection |
| V12-4 | Contact log artifact produced in CI |
| V12-5 | Controller tuning guide in [mujoco.md](mujoco.md) |
| V12-6 | Physics regression tests in `tests/integration/` |

### v1.2.3 — OpenMANIPULATOR-X tabletop ✅

Menagerie OM-X (4-DOF + gripper): empty-cell A→B, pick-and-place (Ø 40 mm ball →
place cone), desk clutter, Γ-wall maze.

| ID | File | Description |
|---|---|---|
| SC-v13a | `omx_reach.yml` | Empty tabletop, EE pose A → B |
| SC-v13b | `omx_pick_place.yml` | Stretch pick green→red (FSM + MuJoCo physics) |
| SC-v13c | `omx_desk_clutter.yml` | Mid-cell wall forces planned retract detour |
| SC-v13d | `omx_wall_maze.yml` | Γ wall forces retract → climb → place |

| # | Criterion |
|---|---|
| V123-1 | Empty-cell A→B under physics (EE error ≤ 5 mm) |
| V123-2 | Pick-and-place FSM places ball without drop mid-transfer |
| V123-3 | Cluttered cell plans a collision-free detour |
| V123-4 | Showcase videos: Γ-maze overview for RRT* and SST |
| V123-5 | Robot from Menagerie; plain MuJoCo ball; place cone |
| V123-6 | Γ-wall path retracts and climbs before placing |

### v1.2.4 — 6-DOF OpenMANIPULATOR-Y ✅

Menagerie OMY (6 revolute joints): reach, floor pick-place, clutter detour.

| ID | File | Description |
|---|---|---|
| SC-v14a | `omy_reach.yml` | Empty tabletop joint-space A→B |
| SC-v14b | `omy_pick_place.yml` | Floor ball pick → cone place |
| SC-v14c | `omy_clutter.yml` | Mid-cell wall forces planned transfer detour |

| # | Criterion |
|---|---|
| V124-1 | Empty-cell A→B under MuJoCo physics |
| V124-2 | Ground pick-and-place places ball in cone |
| V124-3 | Cluttered cell plans a detour (path length ≥ 2) |
| V124-4 | Robot from Menagerie OMY; floor ball + place cone |

### v1.3 — Computer vision pipeline ✅

Pure-Python ball-tracking pipeline with unit tests and algorithm selection.
No manipulation closed loop in this tag. Architecture:
[vision/architecture.md](vision/architecture.md). Selection ADR:
[vision/algorithm-selection.md](vision/algorithm-selection.md).

| # | Criterion |
|---|---|
| V13-1 | `FR-VIS-*` and `docs/modules/vision.md` published |
| V13-2 | Algorithm selection records candidates and chosen primary |
| V13-3 | Unit tests: ball centre error ≤ threshold on fixtures |
| V13-4 | Pipeline runs without ROS |
| V13-5 | Multi-camera interface defined (primary algo may be monocular) |

### v1.4 — CV ↔ manipulation integration ✅

Connect vision to OM-X / OMY pick-and-place so release behaviours match physics
smoke **without hardcoded ball positions**. Place / dispenser pose remains a
known scenario parameter. Cameras: rear structural gate (dual corner cams).

| ID | File | Description |
|---|---|---|
| SC-v16a | `omx_pick_place_cv.yml` | OM-X CV pick-place |
| SC-v16b | `omy_pick_place_cv.yml` | OMY CV pick-place |

| # | Criterion |
|---|---|
| V14-1 | Scenario MJCF includes calibrated sim cameras |
| V14-2 | Release pick-place paths do not use hardcoded ball / `pick_xy` as GT |
| V14-3 | Place / dispenser pose comes only from scenario parameters |
| V14-4 | Physics smoke: FSM reaches DONE; ball enters place volume (seeded) |
| V14-5 | Behaviour parity vs SC-v13b / SC-v14b on shared seeds |

Showcase honesty on this line also covers JointSpaceMPC C-space wall barriers,
wall-contact FAULT, physical OM-X grasp adhesion, and Ø 40 mm pick ball.

---

## v1.5.0 — Robot-model-agnostic core ✅

**Decided by Alexandre, 2026-11-17 (Q2):** `v1.5.0` is the last minor of
major 1. The per-model string-literal dispatch across `control/`,
`planning/`, `ros/` and `launch/` is replaced by a robot registry
(`fret.robots`): a scenario names a `model:`, a profile YAML
(`config/robots/<model>.yml`) describes it, and per-model behavior comes
from a registry of variants. Config/MJCF resources resolve through the
package share from any working directory. Hardware assumptions are removed
from documentation outside `docs/robots/` and `docs/hardware/`
(FR-SYS-13); the v2.x hardware target is undefined pending redefinition.
Hygiene: version alignment, Gazebo legacy removal (the backend could not
run — no `urdf/` directory existed), the unregistered `bridge_node`
launch reference, and the known `docs/dev-environment.md` breakages.

Full spec: [robot-model-agnostic.md](robot-model-agnostic.md).

### Deferred to a simulation backlog

The scope formerly planned for "v1.5" — a ball that rolls to a random rest
pose, a pickability classifier, and an improved place container/dispenser
(scenario `SC-v17`, requirements `FR-VIS-07`/`FR-VIS-08`) — is **not** part
of `v1.5.0`. It moves to a simulation backlog, to be scheduled as its own
release once picked up again:

| Item | Detail |
|---|---|
| Delivery | Rolling ball → random rest pose (MuJoCo contacts) |
| Classification | Pickable vs reject (workspace, size, rest stability — exact rules in FR-VIS) |
| Place geometry | Survey Menagerie / AWS / other assets; ship improved container mesh |
| Cadence | **Provisional:** one ball per cycle for the release scenario; multi-ball optional stretch |
| Scenario ID | `SC-v17` |
| Requirements | `FR-VIS-07`, `FR-VIS-08` |

---

## v2.x — Hardware integration line

### Goal

After **v1.5** is validated in simulation, bring the same contracts onto
**physical hardware** in **modular** steps. Each tag ships one layer,
validated against a hardware-free mock before anything touches a real
port.

The hardware target (robot, servo bus, deployment host) is decided in PR2
(`v2.0.0`) and documented under [`hardware/`](hardware/README.md) and
[`robots/so101.md`](robots/so101.md), which this release line stays
neutral to by reference rather than naming here (`FR-SYS-13`). FR-HW-01..03
(the earlier microcontroller-based assumption) are superseded; FR-HW-10
onward (`docs/requirements.md`) define the decided target.

| Step | Phase | Focus | Result |
|---|---|---|---|
| **v2.0.0** | PR2 | Hardware study + SO-101 MuJoCo simulation twin (new robot variant, pure-NumPy kinematics, position-actuator sim IO, friction grasp) | ✅ (sim twin core; friction-grasp/vision pick-place deferred, see `docs/decisions.md`) |
| **v2.1.0** | PR3 | Hardware-free test infrastructure: mock servo bus, the real bridge developed TDD against it, recorded replays, fault injection, `fret-hw` servo tools | 🔲 |
| **v2.2.0** | PR4 | Safety layer: e-stop detection, bridge-side limits, command watchdog, soft start, torque-off on shutdown / hold on fault; validated in sim and on the PR3 mock | 🔲 |
| **v2.3.0-rc.1** → **v2.3.0** | PR5 | Deployment + bring-up kit (arm64 build incl. the ARCO aarch64 wheel, systemd, udev, camera node, calibration, diagnostics); promoted to `v2.3.0` after the OPS-08 hardware-arrival checklist | 🔲 |
| **v2.4.0** | (post-arrival) | Hardware-validated closed loop + HITL pick-and-place, tolerances set from measured data | 🔲 |

Module stub today: [hardware/bridge.md](hardware/bridge.md). Full phase
sequence, decisions and operations: see the hardware roadmap package
referenced from [hardware/README.md](hardware/README.md).

### Acceptance criteria (era-level)

| # | Criterion |
|---|---|
| V2x-1 | Each step has its own tag, tests, and does not require the next step to merge |
| V2x-2 | Algorithm packages (`fret.vision`, planning, control) stay simulator-agnostic |
| V2x-3 | At least one end-to-end hardware pick-place smoke before declaring the era done |

---

## v3.0 — Definitive product (north-star)

**v3.0** is the release where simulation-proven algorithms and v2.x hardware
modules are validated together as the **default product**. After that, work is
incremental (debugs, features). **No task breakdown for 3.0+ is maintained in
this repository documentation** — see [roadmap.md](roadmap.md).

---

## Version tagging

| Tag | Content | Status |
|---|---|---|
| `v1.1.0` | Dubins dual race — first product showcase | ✅ |
| `v1.2.0` | MuJoCo physics SITL (Dubins) | ✅ |
| `v1.2.3` | OpenMANIPULATOR-X tabletop | ✅ |
| `v1.2.4` | OMY 6-DOF challenge | ✅ |
| `v1.2.5`–`v1.2.7` | Patches (OMY clutter, telemetry R2, Dubins encode) | ✅ |
| `v1.3.0` | CV pipeline + algorithm selection | ✅ |
| `v1.4.0` | CV ↔ manipulation + MuJoCo cameras | ✅ |
| `v1.4.1` | Showcase honesty (MPC barriers, OM-X grasp, Ø40 mm ball) | ✅ |
| `v1.4.2` | ARCO v0.3.7 MPCC + full-arm obstacle contacts | ✅ |
| `v1.4.3` | OMY APPROACH_PICK plans around colliding place-bin shell | ✅ |
| `v1.4.4` | ARCO v0.5.0 compiled core (Rust planners, maps, controllers); `omy_pick_place` out of the release render matrix | ✅ |
| `v1.5.0` | Robot-model-agnostic core (last minor of major 1); dynamic-ball scope deferred to a simulation backlog | ✅ |
| `v2.0.0` | SO-101 hardware study + MuJoCo simulation twin | ✅ |
| `v2.1.0+` | Hardware line continued (mock bridge, safety, Pi deployment) | 🔲 |
| `v3.0.0` | Definitive product (north-star) | ○ |

---

## Release showcase videos

Canonical matrix: [`src/fret/config/release/showcase.yml`](../src/fret/config/release/showcase.yml).

| Robot class | Scenarios (today) | Required cameras |
|---|---|---|
| **mobile** | `dubins_race` (TB3) | `overview` (isometric); `follow` optional / local |
| **static** | OM-X Γ-maze (`omx_wall_maze_rrt` / `_sst`); OMY clutter RRT*/SST | `overview` only |

OM-X and OMY clutter release clips share the same wall geometry and joint
tracking; they differ only in the transfer planner (**RRT\*** vs **SST**).
Simpler demos stay in the repo for development but are not release artifacts.

**Rules:**

- **Overview** must frame the robots and ideally start + goal.
- **Follow** is optional for mobile and **not** on the blocking release path.
  Dubins ships overview-only at `fps: 20`, `960×540`, `clip_duration_s: 40`
  so the encode job stays under ~10 min. Render follow locally when needed:

  ```bash
  ./scripts/video.sh --model dubins --scenario dubins_race \
    --camera follow --full-duration --video-duration 40 --fps 20 \
    --width 960 --height 540 --physics-mode \
    --collision-backend mujoco --planner-algorithm sst \
    -o /tmp/dubins_race_follow.mp4
  ```

  Static / tabletop arms must **not** export follow on release.
- Dubins release uses `clip_duration_s` as the **video** length. Full-race
  acceptance stays in pytest.
- Dubins race uses `--physics-mode`; OM-X clips step MuJoCo actuators.
- Only clean semver tags (`vX.Y.Z`) update `latest/`. Suffixed tags upload under
  `releases/<tag>/` only.
- Showcase renders also write matching telemetry
  (`<scenario>_overview.csv` + `.json`) next to the overview MP4 (FR-SIM-12).
  Download with `./scripts/download_showcase.sh --tag <tag> --with-telemetry`.

Orchestration: `scripts/release/render_showcase.py` +
`.github/workflows/release.yml`.

Every release-tag MP4 **must** play back at **real-time simulation speed**.

Pipeline (`scripts/render_mujoco.py`, invoked by `scripts/video.sh` and
`.github/workflows/release.yml`):

1. Record frames at fixed `fps`; capture `sim_time_s` and `render_duration_s`.
2. Compute `real_time_factor = render_duration_s / sim_time_s`.
3. Post-process with **ffmpeg** (`setpts=PTS/rtf`) so on-screen motion matches
   `sim_time_s`. Skip only when `rtf ≈ 1`.

**RTF rules:**

- Release CI must **not** pass `--no-realtime-postprocess`.
- `--timing-json` is required on release renders.
- Development/debug renders may use `--no-realtime-postprocess`, but uploaded
  R2 showcase assets must always be real-time adjusted.

See [mujoco.md § Showcase rendering](mujoco.md#showcase-rendering-real-time-playback).

Perception cameras (ball tracking) are distinct from release **overview** encode
cameras. Perception mounts: [vision/camera-layout.md](vision/camera-layout.md).
