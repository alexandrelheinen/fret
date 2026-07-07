# FRET

<img src="docs/images/fret.svg" alt="FRET Logo" width="120" align="left">

FRET (A Full-stack framework for Robotic End-effector control and Trajectory planning) is a
ROS 2 robotics project providing end-to-end trajectory planning and execution for manipulators,
from pure-Python simulation to dual-backend SITL (Gazebo + MuJoCo) and physical hardware.

**v1.0 direction (2026 Q3):** ARCO motion planning + Gazebo engineering SITL + MuJoCo visual showcase.
The SCARA was the bootstrap robot; the v1.0 showcase scenario is TBD. See [docs/v1.0.md](docs/v1.0.md).

The project uses a clean architecture: algorithm layers (kinematics, planning, scene acquisition,
validation) are pure Python with no ROS dependency, and a thin ROS 2 layer wires them into
nodes, topics, and actions.

<br clear="left">

---

## Simulation Results (End-Effector A→B)

The following results were produced by the latest CI run of the full end-to-end pipeline
(Milestone 3 — planning + tracking, 20 s simulation at 50 Hz):

| Metric | Value | Limit |
|---|---|---|
| Max EE tracking error | **0.56 mm** | 5 mm |
| RMS EE tracking error | 0.56 mm | — |
| Planning time | < 0.1 ms | 30 s |
| Fault triggered | No | — |
| Trajectory duration | 20 s | — |

Start: `[0.0, 0.0, 0.0]` → Goal: `[0.5, -0.3, 0.05]` (joint space, 50 Hz Jacobian control).

See [`docs/simulation.md`](docs/simulation.md) for the full tutorial.

---

## Quick Start

> **No Gazebo required for validation.** The pure-Python simulation pipeline runs all
> milestone algorithms offline.

```bash
# 1. Clone
git clone https://github.com/alexandrelheinen/fret.git && cd fret

# 2. Install Python deps
pip install numpy matplotlib pyyaml

# 3. Install the fret package (pure-Python, no ROS)
pip install -e . --no-deps

# 4. Run pure-Python validation (no ROS)
pip install -e . --no-deps
pytest tests/simulation/ -v
```

For the full Gazebo SITL tutorial (requires Ubuntu 24.04 + ROS 2 Jazzy), see
[`docs/simulation.md`](docs/simulation.md).

---

## Documentation Index

### Project

- [Simulation Tutorial (A-Z)](docs/simulation.md) — download, build, and run fretsim
- [v1.0 Goals and Release Criteria](docs/v1.0.md) — MuJoCo + Gazebo + ARCO target
- [Platform Study (2026 Q3)](docs/reports/simulation-platform-study-2026-q3.md) — simulator and planner comparison
- [Project Roadmap](docs/roadmap.md) — phases 0–7
- [Milestones](docs/milestones.md) — MS-1 through MS-7, completion status
- [Coding guidelines (authoritative)](docs/guidelines.md)
- [Contributing guide](CONTRIBUTING.md)

### Architecture & Design

- [Architecture overview](docs/architecture.md) — system diagram, data flows, package structure
- [Interface contracts](docs/interfaces.md) — typed data structures, QoS profiles, FSM tables
- [Functional requirements](docs/requirements.md) — FR-SYS, FR-SCN, FR-PLN, FR-CTL, FR-HW
- [Scenario library](docs/scenarios.md) — SC-01 through SC-05 definitions and pass criteria
- [ARCO integration](docs/arco.md) — planning library API, boundary, and technical choices

### Module Documentation

- [Control module](docs/modules/control.md) — Kinematics, ControllerNode, StateEstimator
- [Planning module](docs/modules/planning.md) — PlannerNode, CSpaceChecker, TrajectoryGenerator, ReplanningManager
- [Scene module](docs/modules/scene.md) — SceneAcquisition, OccupancyAdapter, WorkspaceOccupancyBuilder
- [Validation module](docs/modules/validation.md) — metrics, quality gates, CI integration
- [ROS nodes](docs/modules/ros_nodes.md) — PerceptionBridgeNode, StraightLineInjector, ArcInjector
- [Hardware module](docs/modules/hardware.md) — BridgeNode stub (Phase 3)

### Robot

- [SCARA robot specifications](docs/scara/scara.md)

---

## Architecture

```
                    ┌─────────────────────────────────┐
                    │  ARCO (planning library)        │
                    │  KDTree + SST + TrajectoryPruner│
                    └───────────────┬─────────────────┘
                                    │
Gazebo (engineering) ◄──► FRET pipeline ◄──► MuJoCo (visual, MS-6)
    │  /joint_states         │                      │  /joint_states
    │  /joint_commands       │                      │  /joint_commands
    ▼                        ▼                      ▼
PerceptionBridge ──► SceneAcquisition ──► PlannerNode ──► ControllerNode
```

**Key design decisions:**
- Algorithm layers (`kinematics`, `planning`, `scene`, `validation`) are pure Python — no ROS imports — making them fast to test.
- ROS 2 nodes (`*RosNode`, `*Node` in `fret.ros`) are thin wrappers that own only I/O.
- **ARCO** is the motion planner for v1.0; optional `try/except ImportError` with linear fallback when absent.
- **Gazebo** is the engineering SITL backend; **MuJoCo** is the visual showcase backend (MS-6).
- Configuration via YAML files and ROS 2 parameters; no magic numbers in source.

Full diagrams: [docs/architecture.md](docs/architecture.md) · [platform study](docs/reports/simulation-platform-study-2026-q3.md)

---

## Continuous Integration

| Workflow | Trigger | What it checks |
|---|---|---|
| `tests.yml` | PR, push | ROS 2 build, pytest, smoke tests |
| `formatting.yml` | PR | Black, isort, clang-format |
| `type_check.yml` | PR | mypy strict on `src/` |
| `integration.yml` | PR (non-draft) | launch_testing inter-node scenarios |

Run the same checks locally before pushing:

```bash
bash scripts/check/pre_push.sh
```

---

## System Specification

| Layer | Technology |
|---|---|
| High-level controller | Raspberry Pi 5 (Ubuntu 24.04) |
| Middleware | ROS 2 Jazzy |
| Low-level controller | Arduino Mega (Phase 3) |
| Communication | Micro-ROS serial bridge (Phase 3) |
| Engineering simulation | Gazebo Harmonic + RViz 2 |
| Visual simulation | MuJoCo (MS-6, v1.0 target) |
| Motion planning | ARCO SST (C-space, sampling-based) |
| Control | Jacobian pseudoinverse, 50 Hz |
| Bootstrap robot | SCARA 3-DOF RRP (MS-1–5) |
| v1.0 showcase robot | TBD (design session) |
| Physical target | Delta-like robot (Phase 4) |

---

## Requirements

- Ubuntu 24.04 (native or WSL2)
- Python 3.10+ (for pure-Python pipeline)
- ROS 2 Jazzy + Gazebo Harmonic (for full SITL)
- `git` installed and configured

---

## Install and Build

```bash
# Install ROS 2 Jazzy + Gazebo Harmonic (requires sudo)
./scripts/install.sh -y

# Set up the ROS workspace
./scripts/setup.sh -y

# Build all packages
./scripts/build.sh

# Activate the environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## Running SITL

```bash
# Visualize the SCARA in RViz
ros2 launch fret view.py model:=scara

# Run Gazebo simulation (manual joint control)
ros2 launch fret sim.py model:=scara

# Full SITL pipeline (planner + controller + scene)
ros2 launch fret sitl.py model:=scara scenario:=static_reach

# Milestone 1: straight-line trajectory (controller only)
ros2 launch fret sitl.py model:=scara scenario:=straight_line

# Arc trajectory scenario
ros2 launch fret sitl.py model:=scara scenario:=arc
```

See [`docs/simulation.md`](docs/simulation.md) for all scenarios, recording, and log locations.

---

## Models

| Model | Source | Description |
|---|---|---|
| `scara` | Local | 3-DOF RRP SCARA. Geometry inspired by [HiCNC RSR6600](https://www.hcnc-group.com/industrial-robot/scara-robot/4-axis-scara-robot.html). See [`scara.md`](docs/scara/scara.md). |
| `ur3`, `ur5`, ... | External ROS package | Industrial robot models from ROS description packages. |

### Project Structure

```
fret/
├── src/fret/
│   ├── control/        # Kinematics, ControllerNode (Level 3 + 4)
│   ├── planning/       # PlannerNode, CSpaceChecker, TrajectoryGenerator, ...
│   ├── scene/          # SceneAcquisition, OccupancyAdapter, WorkspaceOccupancyBuilder
│   ├── ros/            # ROS bridge nodes (PerceptionBridge, injectors)
│   ├── validation/     # Metrics and quality gates
│   ├── hardware/       # BridgeNode stub (Phase 3)
│   ├── launch/         # view.py, sim.py, sitl.py, hardware.py
│   ├── config/         # YAML configs (scenarios, controllers, perception)
│   ├── urdf/           # SCARA XACRO model
│   └── worlds/         # Gazebo SDF world files
├── tests/              # Unit and integration tests (mirrors src/)
├── scripts/            # CI and developer automation scripts
└── docs/               # Architecture, requirements, module docs
```

---

## Python Tooling

```bash
# Format
isort src && black src

# Type check
mypy src/

# Unit tests (no ROS required)
pytest tests/ -v
```

---

## v1.0 Release Assessment

**FRET is approaching v1.0 but not yet ready to tag `v1.0.0`.** See [docs/v1.0.md](docs/v1.0.md)
for the full acceptance criteria and task list.

### What is done (MS-1–5)

- MS-1 through MS-5 pass in pure-Python CI (including pillar avoidance).
- EE tracking error is well within spec (0.56 mm vs. 5 mm limit).
- Core algorithm modules tested (>90% coverage target).
- Architecture, interfaces, and requirements fully documented.
- Platform study completed; direction locked: **ARCO + Gazebo + MuJoCo**.

### What remains for v1.0

| Gap | Milestone | Priority |
|---|---|---|
| Gazebo SITL end-to-end validation (pillar scenario) | MS-5 | High |
| ARCO SST active in CI (not linear fallback) | MS-5 / v1.0 B-1 | High |
| MuJoCo visual backend | MS-6 | High |
| v1.0 showcase scenario (robot + environment) | MS-7 | **Next: design session** |
| Article-ready demo assets (video, benchmark table) | MS-7 | Medium |
| Hardware HITL | Phase 3 | Deferred past v1.0 |

**Version tagging plan:**

| Tag | When |
|---|---|
| `v0.9.0` | MS-1–5 algorithm core (now) |
| `v1.0.0-rc1` | MS-6 MuJoCo backend functional |
| `v1.0.0` | MS-7 showcase scenario on both backends |

---

## Log Analysis

| Log type | Location |
|---|---|
| colcon build | `./log/latest_build/logger_all.log` |
| ROS 2 runtime | `~/.ros/log/latest/launch.log` |
| Gazebo server | `~/.gz/sim/log/<timestamp>/server_console.log` |
| Simulation results | `/tmp/sim_<name>/results.env` |

For time-series analysis, use [PlotJuggler](https://plotjuggler.io/):

```bash
sudo apt install ros-jazzy-plotjuggler-ros
ros2 run plotjuggler plotjuggler
```

---

## Contributing

[CONTRIBUTING.md](CONTRIBUTING.md) is the single source of truth for humans and AI
agents: SDD workflow, 4-level V-cycle stages, quality gates, and merge policy.
Coding conventions live in [docs/guidelines.md](docs/guidelines.md).

---

## References

- Craig, J. J. (2005). *Introduction to Robotics.* Pearson Education.
- Siciliano et al. (2009). *Robotics: Modelling, Planning and Control.* Springer.
- Lynch & Park (2017). *Modern Robotics.* Cambridge University Press.
- ROS 2 Documentation: https://docs.ros.org/

## License

MIT License. See [LICENSE](LICENSE).
