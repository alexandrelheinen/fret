# FretSim — Simulation Tutorial

This document provides step-by-step instructions to download, build, and run the
FRET simulation stack (FretSim) from scratch.

Two modes are available:

| Mode | Requirements | Purpose |
|---|---|---|
| **Pure-Python** | Python 3.10+, numpy, matplotlib | Validate all milestone algorithms offline |
| **Full SITL** | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic | Real-time simulation with physics |

---

## Prerequisites

- Ubuntu 24.04 (or WSL2 on Windows)
- `git` installed and configured
- Internet access for package installation

---

## Step 1 — Clone the repository

```bash
git clone https://github.com/alexandrelheinen/fret.git
cd fret
```

---

## Step 2 — Install system dependencies

Run the provided installer (requires `sudo`). It installs ROS 2 Jazzy,
Gazebo Harmonic, Python tools, and clang-format:

```bash
./scripts/install.sh -y
```

To install and set up the ROS workspace in one command:

```bash
./scripts/setup.sh --install -y
```

> **What gets installed:**
> - ROS 2 Jazzy Desktop (`ros-jazzy-desktop`, `ros-dev-tools`)
> - Gazebo Harmonic (`gz-harmonic`, `ros-jazzy-ros-gz`)
> - Build tools: `colcon`, `rosdep`, `vcstool`
> - Python tools: `black`, `isort`, `pytest`
> - C++ tools: `clang-format`

---

## Step 3 — Build the workspace

```bash
./scripts/build.sh
```

This runs `colcon build`, generates URDF files from XACRO, generates STL meshes,
and installs all launch and config files.

Build artefacts are written to `build/`, `install/`, and `log/`.

---

## Step 4 — Activate the environment

Every terminal that runs FRET nodes must source both overlays:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## Step 5A — Pure-Python validation (no Gazebo required)

Milestone and scenario acceptance criteria are enforced by **pytest** under
`tests/simulation/` and `tests/integration/` (the latter needs ROS; see Step 5B).

### Install the package

```bash
pip install -e ".[dev]" --no-deps
```

### Milestone 1 — Straight-line tracking (MS-1)

```bash
pytest tests/simulation/test_scenario_straight_line.py -v
```

Validates FR-CTL-02 (EE error ≤ 5 mm) and straight-line corridor constraints
without a live ROS context.

### Milestones 2–5 and scenarios

| Milestone / scenario | Test location |
|---|---|
| MS-2 planning | `tests/planning/` |
| MS-3 static reach | `tests/integration/test_scenario_static_reach_full.py` (ROS) |
| MS-4 occupancy | `tests/scene/test_workspace_occupancy.py` |
| MS-5 pillar avoidance | `tests/integration/test_scenario_pillar_avoidance.py` (ROS) |
| SC-05 arc | `tests/planning/test_arc_injector.py` |

Run the full unit suite (no ROS):

```bash
pytest tests/ --ignore=tests/integration -v
```

Run integration scenarios (requires Steps 1–4 and `xvfb`):

```bash
bash scripts/tests/integration.sh
```

---

## Step 5B — Full SITL with Gazebo

> **Requires:** Steps 1–4 completed, Gazebo running with a display.

### Visualize the SCARA model in RViz

```bash
ros2 launch fret view.py model:=scara
```

### Run the SCARA in Gazebo (manual joint control)

```bash
ros2 launch fret sim.py model:=scara
```

### Run the full SITL pipeline (planning + control + execution)

```bash
ros2 launch fret sitl.py model:=scara scenario:=static_reach
```

Other available scenarios:

```bash
# Milestone 1: controller only (no planner), straight-line trajectory
ros2 launch fret sitl.py model:=scara scenario:=straight_line

# SC-05: arc trajectory
ros2 launch fret sitl.py model:=scara scenario:=arc

# SC-02: obstacle avoidance
ros2 launch fret sitl.py model:=scara scenario:=obstacle_avoidance

# SC-03: planning failure / timeout
ros2 launch fret sitl.py model:=scara scenario:=planning_timeout
```

### Record a ROS bag

```bash
ros2 launch fret sitl.py model:=scara scenario:=static_reach
# In a second terminal:
source /opt/ros/jazzy/setup.bash && source install/setup.bash
mkdir -p log/bags
ros2 bag record -o log/bags/sitl_run /joint_states /joint_commands /joint_trajectory
```

---

## Running Unit Tests

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pytest tests/ -v
```

---

## Log Locations

| Log type | Path |
|---|---|
| Build (colcon) | `./log/latest_build/logger_all.log` |
| ROS 2 runtime | `~/.ros/log/latest/launch.log` |
| Gazebo server | `~/.gz/sim/log/<timestamp>/server_console.log` |
| Simulation results | `/tmp/sim_<name>/results.env` (or `--output` path) |

---

## Known Issues

See the [v1.0 release assessment](../README.md#release-assessment) for a full
list of known limitations.
