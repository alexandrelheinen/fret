# FRET

<img src="docs/images/fret.svg" alt="FRET Logo" width="120" align="left">

FRET (A Full-stack framework for Robotic End-effector control and Trajectory planning) is a
ROS 2 robotics project providing end-to-end trajectory planning and execution for a SCARA
manipulator, from pure-Python simulation to Gazebo SITL and physical hardware.

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

# 4. Run the full A-to-B simulation (Milestone 3: planning + 50 Hz tracking)
python3 scripts/simulate_milestone3_pipeline.py --output /tmp/sim_ms3

# 5. Run the arc scenario (SC-05)
bash scripts/simulate_arc.sh --output /tmp/sim_arc
```

For the full Gazebo SITL tutorial (requires Ubuntu 24.04 + ROS 2 Jazzy), see
[`docs/simulation.md`](docs/simulation.md).

---

## Documentation Index

### Project

- [Simulation Tutorial (A-Z)](docs/simulation.md) — download, build, and run fretsim
- [Project Roadmap](docs/roadmap.md) — phases 0–7
- [Milestones](MILESTONES.md) — requirements table, completion status, MS-1 through MS-5
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
Gazebo (Ignition)
    │  /joint_states
    │  /tf (obstacle transforms)
    ▼
PerceptionBridgeNode ──► /obstacle_cloud ──► SceneAcquisitionNode
                                                     │ OccupancyUpdatePayload
                                                     ▼
                                             PlannerNodeRos (Action server)
                                             ├── CSpaceChecker (FK + KDTree)
                                             └── TrajectoryGenerator (prune → optimize → B-spline)
                                                     │ /joint_trajectory
                                                     ▼
                                             ControllerRosNode (50 Hz, Jacobian)
                                                     │ /joint_commands
                                                     ▼
                                               Gazebo joint controllers
```

**Key design decisions:**
- Algorithm layers (`kinematics`, `planning`, `scene`, `validation`) are pure Python — no ROS imports — making them fast to test.
- ROS 2 nodes (`*RosNode`, `*Node` in `fret.ros`) are thin wrappers that own only I/O.
- ARCO is an optional dependency: `try/except ImportError` pattern; linear interpolation fallback when absent.
- Configuration via YAML files and ROS 2 parameters; no magic numbers in source.

---

## Continuous Integration

| Workflow | Trigger | What it checks |
|---|---|---|
| `tests.yml` | PR, push | ROS 2 build, pytest, smoke tests |
| `formatting.yml` | PR | Black, isort, clang-format |
| `type_check.yml` | PR | mypy strict on `src/` |
| `simulations_ci.yml` | PR (non-draft) | MS-1 through MS-4, SC-01, SC-05 pure-Python simulations |
| `release.yml` | version tags | full suite |

Run the same checks locally before pushing:

```bash
bash scripts/pre_push.sh
```

---

## System Specification

| Layer | Technology |
|---|---|
| High-level controller | Raspberry Pi 5 (Ubuntu 24.04) |
| Middleware | ROS 2 Jazzy |
| Low-level controller | Arduino Mega (Phase 3) |
| Communication | Micro-ROS serial bridge (Phase 3) |
| Simulation | Gazebo Harmonic + RViz 2 |
| Motion planning | ARCO SST (C-space, sampling-based) |
| Control | Jacobian pseudoinverse, 50 Hz |
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

**FRET is not yet ready for a functional v1.0 release.** The following blockers remain:

1. **Milestone 5 not complete:** The pillar-avoidance scenario (`pillar_avoidance.yml`,
   `pillar_scenario.sdf`) has not been implemented. This is the final milestone that
   demonstrates full autonomous obstacle avoidance in Gazebo.

2. **ARCO not validated in CI:** The CI uses the linear-interpolation fallback planner
   because ARCO is not installed. Full ARCO SST integration requires a dedicated CI job
   with `pip install -e ../arco/` (or a pip-published ARCO package).

3. **Gazebo SITL not smoke-tested in CI:** All CI simulations are pure-Python (no Gazebo).
   The `sitl.py` launch file exists and is structurally correct but has not been validated
   against a live Gazebo instance in CI. At least `view.py` and `sim.py` smoke tests should
   be re-validated on the current codebase.

4. **No ROS bag for a real Gazebo run:** The tutorial logs above come from pure-Python
   simulation. A real Gazebo bag has not been recorded and uploaded.

**What is v1.0-ready (pure-Python simulation):**
- MS-1 through MS-4 all pass CI.
- EE tracking error is well within spec (0.56 mm vs. 5 mm limit).
- All core algorithm modules are tested (>90% coverage target).
- Architecture, interfaces, and requirements are fully documented.

**Recommendation:** Tag `v0.9.0` to mark the completion of Milestones 1–4 and the
documentation update. Target `v1.0.0` after Milestone 5 is complete and Gazebo SITL
is validated end-to-end in CI.

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
