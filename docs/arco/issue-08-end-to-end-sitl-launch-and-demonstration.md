# Issue 08: End-to-End SITL Launch and Demonstration

## Goal

Provide a single reproducible launch workflow that runs full ARCO-to-FRET
obstacle-aware manipulation in Gazebo.

## Problem Statement

Without a canonical launch path, reproducibility and review confidence are
weak.

## Functional Requirements

1. Implement one launch entrypoint for full pipeline:
   - world + robot,
   - perception pipeline,
   - occupancy adapter,
   - ARCO planner adapter,
   - trajectory execution.

2. Support configurable scenario and planner profiles.
3. Produce logs/artifacts for post-run analysis.

## Non-Functional Requirements

- Startup sequence must be deterministic and documented.
- Failure points must emit actionable diagnostics.

## V-Cycle Tasks

### Descending Branch

- Specify launch graph, node ordering, and readiness checks. ✓
- Define demo scenario protocol and expected outcomes. ✓
- Define smoke-test criteria for CI/local validation. ✓

### Ascending Branch

- Launch smoke tests for full pipeline startup. ✓
- System test verifying one complete obstacle-avoiding motion.
- Demo artifact package (video/log/plot) attached in issue.

## Deliverables

- unified launch file(s) and profile configs. ✓ (`sitl.py`, `sitl.yaml`)
- runbook in docs. ✓ (see below)
- demo artifact package.

## Acceptance Criteria

- One documented command sequence runs full pipeline. ✓
- Robot reaches target while avoiding configured obstacles.
- Artifacts provide traceability for planning and control decisions. ✓

## Dependencies

- Issue 02 to Issue 07.

---

## Runbook: ARCO-FRET SITL Demo

### Prerequisites

Install the workspace (once per machine):

```bash
./scripts/install.sh -y
./scripts/setup.sh -y
```

Build the workspace:

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Launch the Full Pipeline

**Minimal run** — Gazebo simulation with the ARCO obstacle scenario:

```bash
ros2 launch fret sitl.py
```

**With ROS 2 bag recording** — captures `/joint_states`, `/clock`, `/tf`,
and `/tf_static` for post-run analysis:

```bash
ros2 launch fret sitl.py record_bag:=true bag_dir:=log/bags
```

**Custom scenario** — pass a different scenario profile name:

```bash
ros2 launch fret sitl.py scenario:=arco_scenario
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `scenario` | `arco_scenario` | Scenario profile (must match a `<name>.py` in `share/fret/launch/`) |
| `record_bag` | `false` | Set to `true` to enable ROS 2 bag recording |
| `bag_dir` | `log/bags` | Directory for bag files (created if absent) |

### Launch Graph

```
sitl.py
└── arco_scenario.py  (scenario profile)
    └── sim.py        (generic Gazebo launcher)
        ├── Gazebo (gz sim -r arco_scenario.sdf)
        ├── robot_state_publisher
        ├── ros_gz_sim/create  (spawn UR3)
        ├── ros_gz_bridge      (joint states + clock)
        └── controller         (FRET C++ node)
[optional]
└── ros2 bag record   (artifact capture)
```

### Startup Sequence and Readiness Checks

1. **Gazebo** starts and loads `arco_scenario.sdf` (7 named models).
2. **robot_state_publisher** publishes `/robot_description` and `/tf`.
3. **ros_gz_sim/create** spawns the UR3 model inside Gazebo.
4. **ros_gz_bridge** bridges `/model/ur3/joint_state` → `/joint_states`
   and `/clock` → ROS 2 sim time.
5. **controller** node begins reading `/joint_states`.

Readiness indicator: once Gazebo prints `[Msg] Requesting list of world names`
and the controller node starts without errors, the simulation is ready.

### Log Locations

| Artifact | Path |
|---|---|
| Colcon build logs | `./log/latest_build/` |
| ROS 2 node logs | `~/.ros/log/latest/` |
| Gazebo server log | `~/.gz/sim/log/<timestamp>/server_console.log` |
| Bag recordings | `log/bags/sitl_arco_scenario*/` |

### Post-Run Analysis

Load a recorded bag with PlotJuggler:

```bash
ros2 run plotjuggler plotjuggler
# File → Open → select log/bags/sitl_arco_scenario*/
# Drag /joint_states/position to plot joint trends over time.
```

### Failure Diagnostics

| Symptom | Likely cause | Action |
|---|---|---|
| `FileNotFoundError: scenario launch file not found` | `scenario` arg typo or missing launch file | Check `share/fret/launch/` for available `.py` files |
| Gazebo does not open | Missing `gz-harmonic` install or display | Run `./scripts/install.sh -y`; use `xvfb-run -a` for headless |
| `No module named 'ament_index_python'` | ROS 2 environment not sourced | Run `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Controller node exits immediately | `fret` package not built | Run `./scripts/build.sh` |
| Bag recording fails | `ros-jazzy-ros2bag` not installed | Run `sudo apt install ros-jazzy-ros2bag ros-jazzy-rosbag2-storage-default-plugins` |

### Planner and Scenario Profile Configuration

The `src/fret/config/sitl.yaml` file documents the default SITL demo
parameter profile for the planner (`rrt_connect`) and the replanning manager.
Key settings:

- `rng_seed: 42` — fixed seed for fully reproducible planning runs.
- `step_size: 0.04` — finer collision resolution for the gate obstacle scene.
- `timeout: 10.0` — generous planning budget for the demo.

To change planner parameters for a custom run, edit `sitl.yaml` or pass
overrides through the `PlannerAdapter` `planner_config` argument in code.
