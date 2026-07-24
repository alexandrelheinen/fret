# FRET

<img src="docs/images/fret.svg" alt="FRET Logo" width="120" align="left">

**FRET** (Full-stack Robotic Effector Trajectories) is a ROS 2 robotics library that
connects the [ARCO](https://github.com/alexandrelheinen/arco) motion-planning stack to
MuJoCo simulation. Algorithm code lives in pure Python; a thin ROS 2 layer handles
topics, actions, and simulator I/O.

**Current release: v1.3.0** — `fret.vision` ball pipeline (OpenCV HSV blob +
table-plane lift), unit tests, algorithm selection; plus v1.2.x MuJoCo physics
SITL showcases (TB3 Dubins, OM-X / OMY) with PlotJuggler telemetry on R2.

<br clear="left">

---

## What FRET provides

| Capability | Mobile (TB3 / Dubins) | Manipulators (OM-X / OMY) |
|---|---|---|
| Role | ARCO SE(2) → MuJoCo physics case study | Tabletop pick-and-place (+ CV from v1.4) |
| Planning | RRT* / SST in SE(2) | Joint-space RRT* / SST + FSM |
| Control | Path-following MPC | JointSpaceMPC |
| Computer vision | **Not used** | Pipeline in v1.3; integrated v1.4–v1.5 |
| Sim | MuJoCo physics SITL | MuJoCo physics + (v1.4) cameras |

**Current release line:** **v1.3.0** ships the CV algorithm layer. Next:
**v1.4** CV↔manipulation (MuJoCo cameras, no hardcoded ball). See
[docs/roadmap.md](docs/roadmap.md) and [docs/releases.md](docs/releases.md).

---

## What's inside the repository

```
src/fret/
├── control/           # Per-robot kinematics and controllers
├── planning/          # PlannerNode, C-space checkers, trajectory post-processing
├── scenario/          # Pure-Python E2E orchestrators (Dubins race)
├── telemetry/         # Opt-in PlotJuggler CSV logger + layouts (FR-SIM-12)
├── scene/             # Scene acquisition → occupancy adapter
├── vision/            # Ball CV pipeline (v1.3: HSV + table-plane)
├── ros/               # MuJoCo bridge, perception bridge, race node
├── validation/        # Metrics and quality gates
├── hardware/          # HITL bridge stub (v2.x)
├── config/
│   ├── scenarios/     # Run definitions (start/goal, duration, config refs)
│   ├── planning/      # Algorithm tunables (clearance, trajectory, replanning)
│   ├── controllers/   # Per-model gains (dubins.yml, …)
│   ├── worlds/        # Obstacle layouts + Dubins vehicle/planner tuning
│   ├── vision/        # CV thresholds + camera extrinsics (from v1.3)
│   └── simulation/    # MuJoCo bridge settings
├── mjcf/              # MuJoCo scenes (primary visual backend)
├── launch/            # sitl.py, mujoco.py
└── scripts/           # CLI entry points (render, view, download showcase)
```

**Asset policy:** product robots and props load from git submodules (Menagerie + AWS).
Bootstrap planning primitives remain in the codebase for CI; they are not product
showcase targets.

---

## Architecture

### Design principles

- **ROS 2** is the runtime middleware (topics, services, actions, parameters).
- **ARCO** is a synchronous library inside planner nodes — not a separate ROS node.
- **C-space** is the planning domain for manipulators; **SE(2)** for Dubins agents.
- **MuJoCo** is the simulation engine for physics, contacts, rendering, cameras,
  and SITL. v1.2 ships **physics SITL** by default (`mj_step`, velocity
  actuators, contact dynamics). Kinematic mirroring remains for fast regression
  (`physics_mode:=false` or `--kinematic-mode`).
- **Computer vision** (`fret.vision`, from v1.3) serves **manipulators only**;
  TB3 / Dubins never consumes ball tracking. Place/dispenser poses stay known
  scenario parameters. Architecture: [docs/vision/](docs/vision/).
- **Hardware HITL** is a **v2.x** era — not part of v1.3–v1.5.
- **Simulator-specific code** lives only in `fret.ros` and `launch/`.
- **Configuration over code:** every tunable algorithm parameter (planning clearance,
  controller gains, trajectory limits, replanning thresholds, Dubins vehicle
  margins, vision HSV ranges, and similar) **must live in YAML under `src/fret/config/`**.
  Python and MJCF must not embed numeric defaults for those values — missing keys
  fail at load time. Rendering-only constants (camera presets, mesh colours, UI
  layout) may stay hardcoded. Full reference: [docs/config.md](docs/config.md).

### Configuration (mandatory standard)

FRET uses a layered YAML config system loaded by `fret.config_loader`:

| Layer | Path | Examples |
|---|---|---|
| Scenario | `config/scenarios/*.yml` | start/goal, `planning_timeout`, `planning_config` ref |
| Planning | `config/planning/*.yml` | `contact_radius`, `max_interp_step_m`, replanning |
| Controller | `config/controllers/*.yml` | `kp`, `fault_threshold`, `max_joint_velocity` |
| World | `config/worlds/*.yml` | box obstacles, Dubins `vehicle.clearance_margin` |

Each scenario declares which algorithm configs to use:

```yaml
/**:
  ros__parameters:
    planning_config: planning/dubins.yml
```

Per-scenario overrides merge into the referenced files (no Python edits required):

```yaml
    planning:
      contact_radius: 0.020
```

**Policy:** if changing a value alters planning or control behavior, change
YAML only. Do not add fallbacks like `.get("key", 0.015)` in source. CI and code
review treat hardcoded tunables as defects.

**Tuning examples**

| Goal | Edit |
|---|---|
| Dubins safety margin | `config/worlds/dubins_race_obstacles.yml` → `vehicle.clearance_margin` |
| Dubins controller gains | `config/controllers/dubins.yml` |

Load in Python:

```python
from fret.config_loader import load_scenario_bundle

bundle = load_scenario_bundle("config/scenarios/dubins_race.yml")
# scenario + planning/world overlays available on bundle
```

### Layer stack

```mermaid
flowchart TB
    TASK["TASK<br/>scenario YAML → planning / world"]
    PLANNING["PLANNING<br/>ARCO RRT* / SST, occupancy, pruner"]
    CONTROL["CONTROL<br/>per-robot kinematics + tracking loops"]
    SIMULATOR["SIMULATOR<br/>MuJoCo (physics, contacts, rendering)"]

    TASK --> PLANNING --> CONTROL --> SIMULATOR
```

### Dubins race data flow (v1.1 — first product showcase)

Scenario YAML references `planning/dubins.yml`; vehicle footprint and planner ARCO
tunables live in `config/worlds/dubins_race_obstacles.yml`. Two agents share one
structure occupancy map built from rectangular footprints and dead-end alcoves.
Each agent runs its own ARCO planner (RRT* vs SST), then a Pure Pursuit tracking
loop. `DubinsRaceRunner` (or `render_mujoco.py`) steps both agents in lockstep and
mirrors poses into `dubins_race.xml`.

```mermaid
flowchart TB
    subgraph config
        SY[dubins_race.yml]
        PL[planning/dubins.yml]
        WY[dubins_race_obstacles.yml]
    end

    subgraph fret_scenario
        DR[DubinsRaceRunner]
    end

    subgraph occupancy
        KD[KDTreeOccupancy from box footprints]
    end

    subgraph arco_planning
        RRT[RRTPlanner agent 1]
        SST[SSTPlanner agent 2]
        PP[Pure Pursuit × 2]
        DV[DubinsVehicle × 2]
    end

    subgraph fret_ros
        DB[DubinsRaceBridgeCore / mujoco_bridge]
    end

    subgraph MuJoCo
        MJ[dubins_race.xml]
    end

    SY --> DR
    PL --> DR
    WY --> KD
    KD --> RRT
    KD --> SST
    RRT --> PP
    SST --> PP
    PP --> DV
    DV --> DR
    DR --> DB
    DB <-->|rrt + sst joints| MJ
```

### Robot model routing

Launch selects kinematics, collision checker, MJCF, and controller config via
`model:=` and `scenario:=`:

```bash
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
```

| `model` | Release | Control strategy |
|---|---|---|
| `dubins` | v1.1 | ARCO Pure Pursuit + DubinsVehicle |
| `open_manipulator_x` | v1.2.3 | Jacobian / joint-space tracking (Menagerie OM-X) |
| `six_dof` | v1.2.4 | Jacobian + numerical IK (planned) |

Robot details: [docs/robots/README.md](docs/robots/README.md).

### ARCO boundary

| Owner | Responsibility |
|---|---|
| **ARCO** | KDTree occupancy, RRT* / SST, trajectory pruner, Dubins vehicle + Pure Pursuit |
| **FRET** | Scene acquisition, per-robot kinematics, ROS I/O, MJCF worlds, E2E runners |

See [docs/arco.md](docs/arco.md) for integration notes.

### ROS interface summary

| Data | Mechanism | Message / type |
|---|---|---|
| Obstacles | Topic | `sensor_msgs/PointCloud2` |
| Joint state | Topic | `sensor_msgs/JointState` |
| Trajectory | Topic | `trajectory_msgs/JointTrajectory` |
| Commands | Topic | `std_msgs/Float64MultiArray` |
| Planning | Action | `PlanRequest.action` |
| Fault | Topic | `std_msgs/String` |

Full QoS and typed contracts: [docs/interfaces.md](docs/interfaces.md).

---

## Quick start

All Python workflows **must** use a virtual environment at the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate   # Linux / macOS / WSL
```

Do not `pip install` FRET or its dependencies into the system interpreter. The
`.venv` directory is gitignored — create it once per clone, then re-activate in
every new shell before `pip`, `pytest`, or MuJoCo scripts.

### 1. Clone and install (algorithms only)

No ROS required — runs the pure-Python test suite and E2E scenario validators.

```bash
git clone https://github.com/alexandrelheinen/fret.git
cd fret
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

# Unit + scenario tests (excludes ROS launch_testing integration)
python3 -m pytest tests/ --ignore=tests/integration \
  -p no:launch_testing -p no:launch_ros -q
```

**Dubins race E2E only:**

```bash
python3 -m pytest tests/scenario/test_dubins_race_e2e.py -v
```

### 2. MuJoCo interactive viewer

Live 3D window (no ROS). All parameters are required — bare `./scripts/view.sh`
prints `missing arguments` and `--help`. With `.venv` activated:

```bash
source .venv/bin/activate
pip install -e ".[sim]"
./scripts/view.sh --model dubins --scenario dubins_race \
  --duration 30 --fps 60 --camera overview
```

Headless MJCF check (CI-friendly):

```bash
python3 scripts/view_mujoco.py --model dubins --scenario dubins_race \
  --duration 30 --fps 60 --camera overview --dry-run
```

### 3. Headless showcase videos

Renders MP4 clips using the same planners and controllers as release CI. With
`.venv` activated:

**Dubins race** — dual-agent RRT* vs SST with overview + split-screen follow:

```bash
source .venv/bin/activate
pip install -e ".[sim]"
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl   # headless Linux / CI

./scripts/video.sh \
  --model dubins \
  --scenario dubins_race \
  --collision-backend mujoco \
  --planner-algorithm sst \
  --full-duration \
  --all-cameras \
  --output-dir /tmp/fret_dubins \
  --fps 30 --width 1280 --height 720
```

Single-camera Python API:

```bash
python3 scripts/render_mujoco.py --model dubins --scenario dubins_race \
  --camera overview -o /tmp/dubins.mp4 --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration
```

Visual walkthrough: [docs/tutorial.md](docs/tutorial.md) · WSL notes: [docs/wsl.md](docs/wsl.md)

### 4. Download release showcase clips

After a version tag, CI uploads overview + follow MP4s to Cloudflare R2:

```bash
./scripts/download_showcase.sh --list
./scripts/download_showcase.sh --tag v1.2.6 --all
./scripts/download_showcase.sh --scenario dubins_race --camera follow
# Video + matching PlotJuggler CSV/JSON (same basename; from v1.2.6)
./scripts/download_showcase.sh --tag v1.2.6 --scenario dubins_race --with-telemetry
```

R2 layout is per scenario folder (`latest/<scenario>/<scenario>_overview.mp4`
alongside `<scenario>_overview.csv` / `.json`). Requires R2 credentials — see
[.env.example](.env.example).

### 5. Full ROS 2 workspace (SITL)

Ubuntu 24.04 + ROS 2 Jazzy. Create and activate `.venv` first so Python deps
(including `pytest` and MuJoCo extras) stay isolated from system packages:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev,sim]"

./scripts/install.sh -y
./scripts/setup.sh -y
./scripts/build.sh

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Dubins dual-agent race (physics default from v1.2)
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins

# Kinematic mirror (fast regression)
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins physics_mode:=false
```

Pre-push quality gate (matches CI; `.venv` must be active):

```bash
source .venv/bin/activate
bash scripts/check/pre_push.sh
```

---

## Telemetry logs (PlotJuggler)

Opt-in time-series logging for debugging motion quality (FR-SIM-12). Agents
register the fields they emit (`tb3_sst.position_enu.x`, …); each sim tick
appends one CSV row. Full PlotJuggler / layout guide:
[docs/modules/telemetry.md](docs/modules/telemetry.md) §5.3.

```bash
# Enable for any scenario runner / render
export FRET_TELEMETRY_ENABLED=1
# or pass telemetry_enabled=True to DubinsRaceRunner.run(...)

# Output (default):
#   /tmp/fret_telemetry/<run_id>/<basename>.csv
#   /tmp/fret_telemetry/<run_id>/<basename>.json   # manifest / units

# Matplotlib companion plots (also useful when PlotJuggler is unavailable)
python3 scripts/plot_telemetry.py \
  --csv /tmp/fret_telemetry/.../dubins_race_overview.csv \
  --output-dir /tmp/fret_telemetry_plots

# PlotJuggler + checked-in layout (src/fret/telemetry/layouts/, not docs/)
bash scripts/plotjuggler.sh \
  --csv /tmp/fret_telemetry/.../dubins_race_overview.csv \
  --scenario dubins_race
```

Layouts auto-load the main panes (Dubins: paths / yaw / speed / path error;
arms: EE + joints). Time axis is always column **`t`**.

Example from a validated Dubins / TB3 race (RRT* blue, SST green, dummy grey):

| ENU paths | Yaw / body speed |
|---|---|
| <img src="docs/images/telemetry/dubins_race_overview_xy.png" alt="Dubins telemetry XY" width="360" /> | <img src="docs/images/telemetry/dubins_race_overview_timeseries.png" alt="Dubins telemetry timeseries" width="360" /> |

---

## Documentation

| Topic | Document |
|---|---|
| Telemetry + PlotJuggler layouts | [docs/modules/telemetry.md](docs/modules/telemetry.md) §5.3 |
| Computer vision (v1.3–v1.5) | [docs/vision/README.md](docs/vision/README.md) |
| v1.2 physics implementation spec | [docs/mujoco_physics_v1.2.md](docs/mujoco_physics_v1.2.md) |
| Configuration reference | [docs/config.md](docs/config.md) |
| Scenario catalogue | [docs/scenarios.md](docs/scenarios.md) |
| Robot models | [docs/robots/README.md](docs/robots/README.md) |
| Dubins race | [docs/robots/dubins.md](docs/robots/dubins.md) |
| MuJoCo integration | [docs/mujoco.md](docs/mujoco.md) |
| Simulation guide | [docs/simulation.md](docs/simulation.md) |
| ARCO integration | [docs/arco.md](docs/arco.md) |
| Interface contracts | [docs/interfaces.md](docs/interfaces.md) |
| Functional requirements | [docs/requirements.md](docs/requirements.md) |
| Module reference | [docs/modules/](docs/modules/) |

**Project management & engineering process**

| Topic | Document |
|---|---|
| Release specification (v1.x / v2.x / v3.0) | [docs/releases.md](docs/releases.md) |
| Roadmap & eras | [docs/roadmap.md](docs/roadmap.md) |
| Contributing & V-cycle | [CONTRIBUTING.md](CONTRIBUTING.md) |
| Coding guidelines | [docs/guidelines.md](docs/guidelines.md) |

---

## CI

With `.venv` activated:

```bash
source .venv/bin/activate
bash scripts/check/pre_push.sh
```

| Workflow | Checks |
|---|---|
| `tests.yml` | Parallel: unit shards (4×), coverage gate, smoke, integration |
| `formatting.yml` | Black, isort, clang-format |
| `type_check.yml` | mypy strict |
| `release.yml` | Dubins showcase renders + R2 publish on version tags |

---

## License

MIT — see [LICENSE](LICENSE).
