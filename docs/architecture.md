# FRET Architecture

> **Related specification documents**
> - Functional requirements and operational envelope: [docs/requirements.md](requirements.md)
> - Cross-module interface contracts, QoS assignments, node FSMs, error propagation: [docs/interfaces.md](interfaces.md)
> - Named SITL validation scenarios: [docs/scenarios.md](scenarios.md)
> - Development workflow (SDD, V-cycle): [CONTRIBUTING.md](../CONTRIBUTING.md)
> - Coding standards: [docs/guidelines.md](guidelines.md)

## 1. UX / Developer Experience

### User roles

FRET is a developer tool. Three distinct roles interact with it at different stages of the project:

- **Simulation engineer**: runs SITL scenarios, observes robot behavior, tunes scenario parameters
- **Control developer**: implements and validates kinematics, runs unit tests, inspects trajectory execution
- **Hardware engineer** (phases 3+): connects physical hardware, monitors telemetry, calibrates actuators

### Inputs

| Input type | Format | Examples |
|---|---|---|
| Robot model | XACRO → URDF (build-time) | `scara` (SITL); delta (Phase 4+) |
| Scene / world | Gazebo world file (`.sdf`) | obstacles, workpiece, fixtures |
| Scenario config | YAML | start pose, goal pose, planner params, controller gains |
| Goal / task | ROS topic or YAML | target end-effector pose in world frame |
| Hardware config | YAML | serial port, baud rate, joint calibration offsets |

### Launch interface

The user interacts with FRET through a small set of named launchers with a clear scope:

```bash
# Validate the robot description in RViz (no simulator)
ros2 launch fret view.py model:=scara

# Run the robot in Gazebo (no planning, manual joint control)
ros2 launch fret sim.py model:=scara

# Run the full SITL pipeline: scene + planning + control + execution
ros2 launch fret sitl.py scenario:=pick_place model:=scara

# Run hardware stack (HITL / physical)
ros2 launch fret hardware.py model:=scara config:=config/hardware.yml
```

The **scenario YAML** is the primary user-facing configuration file. It specifies everything needed to reproduce a run: robot model, world file, start/goal poses, planner selection, controller gains, and recording options. `scenario:=` is a first-class argument on every launcher where it is meaningful.

### Developer setup workflow

```
1. ./scripts/install.sh -y          # system + ROS 2 + tools
2. ./scripts/setup.sh -y            # rosdep, workspace init
3. ./scripts/build.sh               # colcon build (generates URDF, meshes)
4. source install/setup.bash        # activate overlay
5. ros2 launch fret sitl.py ...     # run
```

## 2. Architecture and Data Flow

### Design principles

- **ROS 2 is the middleware** for all runtime inter-process communication. Topics carry time-stamped data between nodes; services handle request-response calls; the parameter server holds configuration.
- **ARCO is a library, not a node.** It is called synchronously inside the `PlannerNode`. Its outputs are plain Python objects (lists of joint configurations) that the node converts to ROS messages.
- **Configuration space is the planning domain.** FRET plans in joint space (C-space), not task space. Obstacle avoidance uses ARCO's `KDTreeOccupancy`, but the collision check is: FK(q) → world-frame point → nearest obstacle distance.
- **Gazebo owns the engineering ground truth.** The Gazebo simulator is the authoritative source of robot state for CI and the hardware path. MuJoCo is a visual showcase backend (v1.0) that must expose equivalent joint I/O.
- **Simulator backends are replaceable.** Algorithm layers are simulator-agnostic. Simulator-specific code lives in `fret.ros` and `launch/` only.
- **ARCO is the sole planner for v1.0.** OMPL integration is a post-v1.0 research option. See [reports/simulation-platform-study-2026-q3.md](reports/simulation-platform-study-2026-q3.md).

### Layers

```
┌────────────────────────────────────────────────────────┐
│  TASK LAYER           (what to do)                     │
│  Goal specification, scenario loading, task sequencing │
└────────────────────────────────────────────────────────┘
                         ▼
┌────────────────────────────────────────────────────────┐
│  PLANNING LAYER       (how to get there)               │
│  C-space occupancy,  ARCO planner,  path post-proc     │
└────────────────────────────────────────────────────────┘
                         ▼
┌────────────────────────────────────────────────────────┐
│  CONTROL LAYER        (how to execute it)              │
│  Kinematics (FK/IK/Jacobian), trajectory tracking      │
└────────────────────────────────────────────────────────┘
                         ▼
┌────────────────────────────────────────────────────────┐
│  HARDWARE LAYER       (physical actuation)             │
│  Serial bridge to Arduino / motor drivers              │
└────────────────────────────────────────────────────────┘
```

### Block diagram — SITL data flow

```
┌──────────────┐     /world_state      ┌──────────────────────────┐
│   Gazebo     │ ─────────────────────▶│  Scene Acquisition       │
│  (simulator) │                       │  point cloud → world frame│
│              │◀── /joint_commands ── │  → KDTreeOccupancy       │
└──────────────┘                       └────────────┬─────────────┘
       │                                            │ OccupancyModel
       │ /joint_states                              ▼
       │                               ┌──────────────────────────┐
       ▼                               │  Planner Node            │
┌─────────────┐   /joint_states        │  (ARCO SST / RRT*)       │
│  State      │ ─────────────────────▶│  C-space collision check  │
│  Estimator  │                        │  → JointPath []          │
│  FK → EE   │                        └────────────┬─────────────┘
└─────────────┘                                     │ JointPath
                                                    ▼
                                       ┌──────────────────────────┐
                                       │  Trajectory Generator    │
                                       │  pruner + optimizer      │
                                       │  + B-spline interp       │
                                       │  → JointTrajectory (ROS) │
                                       └────────────┬─────────────┘
                                                    │ JointTrajectory
                                                    ▼
                                       ┌──────────────────────────┐
                                       │  Controller Node         │
                                       │  Jacobian tracking       │
                                       │  error → velocity cmd    │
                                       └────────────┬─────────────┘
                                                    │ /joint_commands
                                                    ▼
                                              ┌──────────┐
                                              │  Gazebo  │
                                              │ (closes  │
                                              │  loop)   │
                                              └──────────┘
```

### Middleware summary

Full QoS profile specifications are in [docs/interfaces.md](interfaces.md).

| Arc / data | ROS 2 mechanism | Message type | QoS (short) |
|---|---|---|---|
| World geometry (obstacles) | Topic | `sensor_msgs/PointCloud2` | Reliable, Transient Local, depth=1 |
| Joint positions | Topic | `sensor_msgs/JointState` | Best Effort, Volatile, depth=10 |
| EE pose | TF2 broadcast | `geometry_msgs/TransformStamped` | Best Effort, Volatile, depth=100 |
| Planning request (goal) | Action | custom `PlanRequest.action` | — (Action transport) |
| Planned joint path | internal Python call | `list[np.ndarray]` | — (in-process) |
| Joint trajectory | Topic | `trajectory_msgs/JointTrajectory` | Reliable, Volatile, depth=1 |
| Joint velocity commands | Topic | `std_msgs/Float64MultiArray` | Best Effort, Volatile, depth=1 |
| Replanning trigger | Topic | `std_msgs/Bool` | Reliable, Volatile, depth=1 |
| Fault notification | Topic | `std_msgs/String` | Reliable, Transient Local, depth=10 |
| Scenario config | Parameter / YAML | loaded at launch | — |

**Why ROS 2 Actions for planning?** Planning can take 1–30 seconds. An Action gives feedback (iteration count, cost progress from ARCO telemetry), allows cancellation, and delivers the result asynchronously — the correct abstraction for a long-running request.

**ARCO telemetry:** ARCO exposes live metrics (iteration count, cost, stop criteria). The `PlannerNode` polls these and forwards them as Action feedback, giving the operator visibility into planner progress.

**Node state machines and error propagation:** Full FSM definitions for `PlannerNode` and `ControllerNode` (states, transition tables, fault paths) are in [docs/interfaces.md](interfaces.md).

**ARCO dependency:** ARCO is installed as an editable local package: `pip install -e ../arco/`. The minimum API surface FRET depends on and the CI validation step are documented in [docs/interfaces.md](interfaces.md).

## 3. File Organization

### What currently exists and its fate

| Path | Status | Decision |
|---|---|---|
| `src/fret/urdf/scara.xacro` | ✅ Keep | Production URDF — untouched |
| `src/fret/mesh/scara.py` | ✅ Keep | Build-time STL generator |
| `src/fret/rviz/default.rviz` | ✅ Keep | RViz config |
| `src/fret/rviz/scara.rviz` | ✅ Keep | Model-specific RViz config |
| `src/fret/hooks/fret.sh` | ✅ Keep | PYTHONPATH environment hook |
| `src/fret/CMakeLists.txt` | 🔄 Evolve | Keep XACRO/mesh build loop; remove deleted node targets |
| `src/fret/__init__.py` | ✅ Keep | Package marker |
| `scripts/install.sh`, `build.sh`, `setup.sh` | ✅ Keep | Dev workflow scripts |
| `scripts/check/pre_push.sh` | ✅ Keep | Local CI gate runner |
| `src/fret.egg-info/` | 🗑 Ignore | Build artifact, in `.gitignore` |

### Source tree

```
src/fret/
├── CMakeLists.txt               ← build: XACRO→URDF, meshes, launch install
├── package.xml                  ← ROS 2 manifest
├── __init__.py                  ← package marker (empty)
│
├── urdf/                        ← robot descriptions
│   └── scara.xacro
│
├── mesh/                        ← build-time STL generators
│   └── scara.py
│
├── launch/                      ← all Python launch files
│   ├── view.py                  ← RViz visualization (model:=)
│   ├── sim.py                   ← Gazebo simulation (model:=)
│   ├── sitl.py                  ← full SITL pipeline (scenario:=, model:=, backend:=)
│   ├── mujoco.py                ← MuJoCo visual backend (MS-6, planned)
│   └── hardware.py              ← HITL / physical stack (Phase 3)
│
├── mjcf/                        ← MuJoCo models (MS-6, planned)
│   └── scara.xml
│
├── config/                      ← YAML parameter files
│   ├── scenarios/               ← one YAML per runnable scenario
│   │   ├── static_reach.yml
│   │   ├── straight_line.yml
│   │   ├── arc.yml
│   │   ├── obstacle_avoidance.yml
│   │   └── planning_timeout.yml
│   ├── controllers/             ← gain files per controller type
│   │   └── jacobian.yml
│   ├── perception.yaml          ← obstacle definitions for PerceptionBridgeNode
│   ├── benchmark.yaml           ← quality gate thresholds
│   └── trajectory.yaml          ← trajectory post-processing config
│
├── scene/                       ← Scene Acquisition layer
│   ├── __init__.py
│   ├── acquisition.py           ← PointCloud → world-frame obstacle set
│   ├── acquisition_node.py      ← ROS 2 node wrapper
│   ├── occupancy_adapter.py     ← FRET adapter: C-space KDTreeOccupancy
│   └── workspace_occupancy.py  ← 20 cm voxel-grid occupancy builder
│
├── planning/                    ← Planning layer (wraps ARCO)
│   ├── __init__.py
│   ├── planner_node.py          ← pure-Python planning core (Level 3)
│   ├── planner_node_ros.py      ← ROS 2 Action server (Level 4)
│   ├── cspace_checker.py        ← FK + KDTreeOccupancy collision check
│   ├── trajectory_generator.py  ← pruner + optimizer + B-spline
│   ├── trajectory_converter.py  ← trapezoidal velocity profiles
│   └── replanning_manager.py   ← replanning FSM
│
├── control/                     ← Control layer
│   ├── __init__.py
│   ├── controller_node.py       ← ControllerNode (L3) + ControllerRosNode (L4)
│   ├── kinematics.py            ← FK, IK, Jacobian for SCARA RRP
│   └── state_estimator.py       ← joint states → EE pose + TF2 broadcast
│
├── ros/                         ← ROS bridge nodes
│   ├── perception_bridge.py     ← obstacle cloud publisher (from config YAML)
│   ├── mujoco_bridge.py         ← MuJoCo joint I/O adapter (MS-6, planned)
│   ├── straight_line_injector.py← Milestone 1 trajectory injector
│   └── arc_injector.py          ← SC-05 arc trajectory injector
│
├── validation/                  ← Metrics and quality gates
│   ├── __init__.py
│   ├── metrics.py               ← path_length, tracking_rmse, clearance, ...
│   └── quality_gates.py        ← QualityGate, GateResult, evaluate_gates
│
└── hardware/                    ← Hardware layer (Phase 3, stub)
    ├── __init__.py
    └── bridge_node.py           ← serial / Micro-ROS bridge to Arduino
```

### Key organizational decisions

**`scene/` is a new layer.** Separating point-cloud acquisition and the C-space collision-check adapter from planning keeps ARCO completely decoupled: ARCO receives a plain `Occupancy` object and never touches ROS or Gazebo.

**`planning/` wraps ARCO, it is not ARCO.** `planner_node.py` is the ROS 2 shell (Action server, parameter loading). `cspace_checker.py` is the adapter that bridges FRET's FK with ARCO's occupancy interface. `trajectory_generator.py` is the post-processing chain (ARCO `TrajectoryPruner` → `TrajectoryOptimizer` → `BSplineInterpolator`).

**`control/` is language-agnostic at the design level.** The Jacobian kinematics engine is the performance-critical piece. It may start as Python (for rapid iteration in SITL), then be moved to C++ for HITL. Keeping `kinematics.py` as a single replaceable module makes that migration contained.

**`launch/` and `config/` are top-level under `src/fret/`.** ROS 2 convention — `launch/` files are installed to `share/fret/launch/` and `config/` to `share/fret/config/`. The scenario YAML in `config/scenarios/` is the single file a user edits to set up a run.

**`hardware/` is a stub directory.** Creating it now (with empty `__init__.py` and a stub `bridge_node.py`) keeps the architecture complete in the tree without implementing anything. Phase 3 fills it.

**No `src/fret/src/` (C++ convention) until needed.** The old `include/fret/` and `src/fret/src/control/` tree has been cleared. C++ will return in a dedicated sub-tree once the kinematics engine migration decision is made; premature C++ structure adds noise.

**`tests/` mirrors `src/fret/`.** Following the existing guidelines:
```
tests/
├── scene/
├── planning/
├── control/
└── hardware/
```
Each test file is `test_<module>.py` and uses `pytest` with mocked ROS dependencies.