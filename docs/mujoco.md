# MuJoCo Simulation — FRET Integration Specification

> **Authoritative simulator specification.** MuJoCo is FRET's sole simulation
> engine for physics, contacts, rendering, and SITL validation.
>
> **Related:** [simulation.md](simulation.md) · [requirements.md](requirements.md) ·
> [releases.md](releases.md) · [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md) ·
> [modules/ros_nodes.md](modules/ros_nodes.md)

---

## Role in FRET

MuJoCo provides **all** simulation capabilities in FRET:

| Capability | MuJoCo API / mechanism | FRET layer |
|---|---|---|
| Rigid-body dynamics | `mj_step` | `MuJoCoBridgeCore` (v1.2+) |
| Contact resolution | `mj_collision`, contact forces | MJCF geoms + bridge |
| Actuator control | `<actuator>` + `data.ctrl` | Controller → bridge |
| Forward kinematics | `mj_forward` | Bridge, collision checker |
| Collision queries | `mj_geomDistance`, contacts | `CSpaceCheckerMujoco` |
| Visual rendering | `mujoco.viewer`, offscreen RGB | `render_mujoco.py`, `view_mujoco.py` |
| Scene assets | MJCF + meshes | `src/fret/mjcf/` |

FRET uses MuJoCo because it delivers accurate contact-rich physics and
publication-quality rendering in a single, headless-friendly engine suitable
for CI, SITL, and release showcase videos.

Algorithm code (planning, control, scene) remains **simulator-agnostic**
(FR-SIM-02). Only `fret.ros` and launch files talk to MuJoCo directly.

---

## Simulation modes

FRET runs MuJoCo in three modes. All modes share the same MJCF assets.

| Mode | Entry point | Physics | ROS |
|---|---|---|---|
| **Pure-Python** | `tests/`, `fret.scenario.*` | Physics or kinematic (`physics_mode` flag) | No |
| **Showcase** | `view.sh`, `video.sh`, `render_mujoco.py` | Physics default in release CI (`--physics-mode`) | No |
| **SITL** | `ros2 launch fret sitl.py …` | **Physics SITL (v1.2 default)** | Yes |

### Kinematic mirror (legacy / regression)

Controller outputs are integrated in pure Python. The bridge writes resulting
joint positions into MJCF (`qpos`) and calls `mj_forward` for visuals and
collision geometry sync. **No actuator forces or contact dynamics are applied.**

Used for: fast regression, optional `--kinematic-mode` renders, `physics_mode:=false`.

### Physics SITL (v1.2, default)

Controller velocity commands drive MuJoCo actuators. The simulation
advances with `mj_step` each control cycle. Contacts (columns, obstacles,
cargo weld, inter-agent) produce forces that the robot must overcome.

**No pose teleportation.** Joint state published on `/joint_states` comes from
simulated `qpos`, not from open-loop integration.

Used for: release showcase videos, ROS SITL default, physics integration tests.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Controller (50 Hz)  →  /joint_commands                 │
└───────────────────────────────┬─────────────────────────┘
                                ▼
┌─────────────────────────────────────────────────────────┐
│  MuJoCoBridgeNode / MuJoCoBridgeCore                      │
│    kinematic mode: integrate → set qpos → mj_forward    │
│    physics mode:   map commands → ctrl → mj_step        │
└───────────────────────────────┬─────────────────────────┘
                                ▼
┌─────────────────────────────────────────────────────────┐
│  MJCF scene (robot + world + actuators + contacts)      │
└───────────────────────────────┬─────────────────────────┘
                                ▼
                         /joint_states (50 Hz)
```

### Data flow (ROS SITL)

| Topic | Direction | Content |
|---|---|---|
| `/joint_commands` | Controller → bridge | Velocity or torque targets |
| `/joint_states` | Bridge → controller | Simulated positions + velocities |
| `/obstacle_cloud` | Perception → scene | Obstacle point cloud (world frame) |

Launch:

```bash
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
```

MuJoCo is the implicit backend for all SITL launches.

---

## MJCF asset layout

```
src/fret/mjcf/
├── ppp_unit.xml            # FR-SIM-11 Cartesian physics sandbox
├── ppp_warehouse.xml       # v1.0 PPP gantry + warehouse
├── dubins_race.xml         # v1.1 dual Dubins agents (holonomic SE(2) approx.)
├── assets/
│   └── aws_warehouse/      # MIT-0 meshes (PPP visuals)
└── (v1.3+) rrp_pillars.xml, six_dof_cell.xml, …
```

Unit sandboxes (`*_unit.xml`) are loaded by `fret.simulation` for open-loop
physics tests (SC-v12u). Showcase worlds remain separate.

### MJCF conventions

| Rule | Detail |
|---|---|
| Joint names | Must match controller config and `resolve_mjcf_path()` dispatch |
| Units | SI (meters, radians, seconds) |
| Collision geoms | Separate from visual geoms where possible; use `contype`/`conaffinity` |
| Actuators | `<motor>` or `<velocity>` per joint; gains in `config/simulation/mujoco.yml` |
| Free bodies | Welded cargo uses MJCF equality constraints or weld logic in bridge |
| Cameras | Named geoms for `render_mujoco.py` (`overview`, `follow`, …) |

### Importing external models

FRET imports third-party assets into MJCF:

| Source | Use | Script |
|---|---|---|
| AWS RoboMaker warehouse | PPP/Dubins floor and shelf visuals | `scripts/import_aws_warehouse_assets.py` |
| MuJoCo Menagerie (planned v1.4) | UR arm meshes for 6-DOF | TBD |
| URDF via `mujoco.MjModel.from_xml_path` (planned v1.3) | SCARA from existing xacro | TBD |

Imported meshes are converted to OBJ (meters, consistent origin). Collision
geometry for planning may remain analytic boxes in YAML when mesh collision is
too expensive.

---

## Bridge implementation

**Source:** `src/fret/ros/mujoco_bridge.py`  
**Config:** `src/fret/config/simulation/mujoco.yml`  
**Launch:** `src/fret/launch/mujoco.py`

### `MuJoCoBridgeCore` (Level 3)

| Method / symbol | Purpose |
|---|---|
| `resolve_mjcf_path(model, scenario)` | Map model/scenario → MJCF file |
| `make_mujoco_bridge_core(...)` | Factory for PPP, Dubins, future arms |
| `integrate_joint_velocities(...)` | Kinematic-mode Euler step + limit clip |
| `step_physics(...)` | **v1.2** — write `ctrl`, call `mj_step`, read `qpos`/`qvel` |

### `MuJoCoBridgeNode` (Level 4)

- Subscribes: `/joint_commands` (`Float64MultiArray`)
- Publishes: `/joint_states` (`JointState`) at 50 Hz
- Parameter: `physics_mode` (bool) — kinematic mirror vs `mj_step` (v1.2)

### Model dispatch

| `model` | MJCF | Joint names | Integration |
|---|---|---|---|
| `ppp` | `ppp_warehouse.xml` | `joint_x`, `joint_y`, `joint_z` | Prismatic |
| `dubins` | `dubins_race.xml` | `rrt_joint_*`, `sst_joint_*` | SE(2) per agent |
| `rrp` / `scara` | *(v1.3)* | `joint_arm_0`, `joint_arm_1`, `joint_extension` | Revolute + prismatic |
| `six_dof` | *(v1.4)* | TBD | 6× revolute |

---

## Collision and planning

PPP planning uses MuJoCo geometry directly:

**Module:** `src/fret/planning/cspace_checker_mujoco.py`

1. Set joint positions in `MjData`
2. `mj_forward` + `mj_collision`
3. Query contacts and `mj_geomDistance` for clearance

Dubins planning uses analytic rectangular footprints + KD-tree occupancy;
MuJoCo provides visual and (v1.2+) physical column contact.

Scenario parameter: `collision_backend: mujoco` in scenario YAML.

---

## Rendering pipeline

| Script | Purpose |
|---|---|
| `scripts/view_mujoco.py` | Interactive passive viewer |
| `scripts/view.sh` | Wrapper for local preview |
| `scripts/render_mujoco.py` | Headless offscreen frames → MP4 |
| `scripts/video.sh` | CI/release wrapper (`MUJOCO_GL=egl`) |
| `scripts/download_showcase.sh` | Fetch R2 release artifacts |

**CLI policy:** `view.sh`, `video.sh`, and their Python backends require **explicit
arguments** — no implicit model/scenario/render defaults. Zero-arg invocation
prints `missing arguments` and `--help`.

### Required flags

**`view.sh` / `view_mujoco.py`**

| Flag | Description |
|---|---|
| `--model` | Robot model (`ppp`, `dubins`, …) |
| `--scenario` | Scenario stem |
| `--duration` | Animation cycle length [s] |
| `--fps` | Playback frame rate |
| `--camera` | MJCF camera name |

Optional: `--dry-run`, `--no-loop`, `--mjcf`.

**`video.sh` / `render_mujoco.py`**

| Flag | Description |
|---|---|
| `--model`, `--scenario` | Robot + scenario |
| `--fps`, `--width`, `--height` | Render settings |
| `--collision-backend` | `mujoco` or `analytic` |
| `--planner-algorithm` | `rrt_star` or `sst` |
| `--duration` or `--full-duration` | Clip length or full sim time |

Output (one mode required):

- Single camera: `-o/--output` + `--camera`
- All showcase cameras: `--all-cameras` + `--output-dir`

Optional: `--no-tracking`, `--timing-json`, `--no-realtime-postprocess`, `--mjcf`.

### Showcase rendering (real-time playback)

Release-tag MP4s **must** be real-time adjusted: after rendering at fixed
`fps`, `render_mujoco.py` computes
`real_time_factor = render_duration_s / sim_time_s` and runs ffmpeg
(`setpts=PTS/rtf`) so playback duration matches simulated motion time. This is
**on by default**; release CI must never pass `--no-realtime-postprocess`.

| Flag | Release use |
|---|---|
| `--timing-json` | **Required** — per-clip `sim_time_s`, `render_duration_s`, RTF |
| `--no-realtime-postprocess` | **Forbidden** for published showcase assets |

---

```bash
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
./scripts/video.sh --model ppp --scenario ppp_warehouse --all-cameras \
  --output-dir /tmp/showcase --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm rrt_star --full-duration
```

WSL2 display notes: [wsl.md](wsl.md).

---

## Controller tuning workflow (v1.2)

Measured baselines on CI/dev VM (`main`, 2026-07):

| Scenario | Mode | Metric | Value |
| --- | --- | --- | --- |
| Dubins race | Kinematic | `race_duration_s` | ~33 s |
| Dubins race | Physics | `race_duration_s` | ~57 s (RTF ≈ **1.73×**) |
| PPP warehouse | Kinematic | `max_tracking_error_m` | ~5.5 mm |
| PPP warehouse | Physics | `max_tracking_error_m` | ~0.45–0.55 m (`ee_error_limit_physics_m`) |

PPP physics tracking uses the grasp `goal_radius` (0.5 m) as the practical V12-2
gate because prismatic velocity actuators lag the kinematic carrot. The kinematic
10 mm gate remains the regression target for `physics_mode:=false`.

1. **Baseline** — run kinematic mirror; record tracking error and path fidelity.
2. **Enable physics** — set `physics_mode:=true` (v1.2 parameter) or pass
   `--physics-mode` to `./scripts/video.sh` for Dubins showcase clips.
3. **Tune actuators** — adjust `kv` and force limits in
   `config/simulation/mujoco_physics.yml` (see [config.md](config.md#simulation-physics-v12)).
4. **Validate contacts** — enable `contact_log_enabled: true` in `mujoco.yml`;
   inspect `/tmp/fret_physics/<scenario_id>/contacts.jsonl` and
   `metrics.json` (penetration_violations, max_contact_force_n).
5. **Regression clip** — compare kinematic vs physics MP4 under
   `/tmp/fret_physics/<scenario_id>/regression/`; path-length ratio ≤ 1.15,
   SSIM ≥ 0.85 (warning only).

Example physics showcase (Dubins):

```bash
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
  --output-dir /tmp/showcase --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm rrt_star --full-duration \
  --physics-mode
```

Physics integration tests: `tests/integration/test_mujoco_physics_*.py`.

---

## Environment variables

| Variable | Values | When |
|---|---|---|
| `MUJOCO_GL` | `egl`, `glfw`, `osmesa` | Headless render (`egl` on Linux CI) |
| `PYOPENGL_PLATFORM` | `egl` | Pair with `MUJOCO_GL=egl` |

---

## Python dependencies

```bash
pip install -e ".[sim]"   # mujoco, imageio, imageio-ffmpeg
```

Optional asset import tools: `trimesh`, `pycollada`.

---

## Testing

| Layer | Tests |
|---|---|
| Bridge unit | `tests/ros/test_mujoco_bridge.py` |
| PPP collision | `tests/planning/test_cspace_checker_mujoco.py` |
| PPP E2E | `tests/integration/test_scenario_ppp_warehouse.py` |
| Dubins E2E | `tests/scenario/test_dubins_race_e2e.py` |
| Physics SITL (v1.2) | `tests/integration/test_mujoco_physics_*.py` |
| SITL smoke | `scripts/tests/smoke.sh` — PPP + Dubins MuJoCo launch |

---

## v1.2 physics deliverables (summary)

Full acceptance criteria: [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl).

**Implementation specification:** [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md)
(actuators, cargo weld, contact logs, regression tests).

Configuration schema: [config.md § Simulation](config.md#simulation-physics-v12).

| Robot | Physics work |
|---|---|
| PPP | Prismatic actuators; cargo weld contacts; pick-and-place validation |
| Dubins | Body/wheel actuation; column + floor contacts; optional inter-agent blocking |
| Shared | `mj_step` harness, contact logging, CI-green physics mode for both scenarios |

After v1.2, all new robots (SCARA v1.3, 6-DOF v1.4) ship with physics SITL
from day one — no kinematic-only phase.
