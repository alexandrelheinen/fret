# ROS Nodes

**Package:** `fret.ros`
**Source:** `src/fret/ros/`
**Tests:** `tests/ros/`, `tests/scene/`, `tests/integration/`

> MuJoCo is the sole simulator (FR-SIM-01). See [releases.md](../releases.md)
> and [mujoco.md](../mujoco.md). Level-4 ROS nodes for other packages
> (`controller_node`, `planner_node`, `scene_acquisition_node`) are
> documented in [control.md](control.md) and [planning.md](planning.md);
> this page covers `fret.ros` plus the registered executables in
> `pyproject.toml` `[project.scripts]`.

---

## Responsibility

`fret.ros` contains ROS 2 nodes bridging **MuJoCo** to FRET algorithm
layers, plus the Dubins-race orchestration node.

---

## Registered executables (`pyproject.toml` `[project.scripts]`)

| Executable | Entry point | Package |
|---|---|---|
| `fretsim` | `fret._cli:fretsim` | `fret` (CLI) |
| `controller_node` | `fret.control.controller_node:main` | `fret.control` |
| `scene_acquisition_node` | `fret.scene.acquisition_node:main` | `fret.scene` |
| `planner_node` | `fret.planning.planner_node_ros:main` | `fret.planning` |
| `perception_bridge` | `fret.ros.perception_bridge:main` | `fret.ros` |
| `mujoco_bridge` | `fret.ros.mujoco_bridge:main` | `fret.ros` |
| `dubins_race_node` | `fret.ros.dubins_race_node:main` | `fret.ros` |

---

## Components

### `mujoco_bridge.py` — MuJoCoBridgeNode

Simulator I/O for all MuJoCo-backed scenarios except `dubins_race` (which
has its own node below). Subscribes to velocity commands and publishes
joint state at 50 Hz.

| Symbol | Description |
|---|---|
| `MuJoCoBridgeCore` | Level-3 joint integration against MJCF limits |
| `make_mujoco_bridge_core()` | Factory for model/scenario dispatch |
| `integrate_joint_velocities()` | Kinematic-mode Euler step + limit clip |
| `step_physics()` | Actuator commands + `mj_step` (v1.2+) |
| `resolve_mjcf_path()` | Resolve MJCF path for model/scenario |

**Topics:**

| Topic | Type | Direction |
|---|---|---|
| `/joint_commands` | `std_msgs/Float64MultiArray` | Subscribe |
| `/joint_states` | `sensor_msgs/JointState` | Publish |

**Config:** `src/fret/config/simulation/mujoco.yml`

**Parameters:**

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `physics_mode` | bool | `false` | Kinematic mirror vs `mj_step` physics SITL |
| `update_rate` | float | `50.0` | `/joint_states` publish rate [Hz] |
| `substeps_per_tick` | int | `25` | MuJoCo steps per control callback |
| `simulation_config` | string | — | Loaded from scenario or launch |

Spec: [../mujoco.md](../mujoco.md) · [../config.md](../config.md).

**Tests:** `tests/ros/test_mujoco_bridge.py`.

---

### `perception_bridge.py` — PerceptionBridgeNode

Reads obstacle definitions from `config/perception.yaml` and publishes a
`sensor_msgs/PointCloud2` on `/obstacle_cloud` at 1 Hz. Subscribes to `/tf`
to obtain entity poses when obstacles are frame-relative.

Supported obstacle types (configured in `perception.yaml`):
- `box` — samples surface points on all 6 faces
- `cylinder` — samples lateral surface and top-cap points

This node provides the obstacle cloud that feeds `SceneAcquisition`
(`fret.scene`). It is **not** the ball computer-vision pipeline
(`fret.vision` / v1.3+).

**Configuration:** `src/fret/config/perception.yaml`

```yaml
obstacles:
  - name: obstacle_box_a
    type: box
    size: [0.10, 0.10, 0.10]
    pose: {x: 0.25, y: 0.10, z: 0.05}
```

**Tests:** `tests/scene/test_perception_bridge.py`.

---

### `dubins_race_node.py` — DubinsRaceRosNode

Orchestrates the `dubins_race` scenario end to end (planning, tracking, and
MuJoCo stepping for both agents) in one node, rather than the generic
planner/controller/bridge wiring used by manipulator scenarios.

**Tests:** `tests/integration/test_sitl_dubins_race_launch.py`,
`tests/integration/test_mujoco_physics_dubins.py`.

---

### `mujoco_physics_log.py` — physics-mode contact/telemetry logging

Helper module (not a standalone node) used by `mujoco_bridge.py` and
`dubins_race_node.py` to record contact and physics telemetry during
physics-mode runs. See [telemetry.md](telemetry.md).

**Tests:** `tests/ros/test_mujoco_physics_log.py`.

---

## Launch Integration

Scenario-conditional launch lives in `launch/sitl.py` (FR-SYS-06: resolves
`model` through the robot registry via the migrated call sites it uses,
`config_loader`/`sitl_config`):

```python
# sitl.py
include mujoco.py                       # when scenario != "dubins_race"

if scenario == "dubins_race":
    dubins_race_node                    # DubinsRaceRosNode (this package)
else:
    planner_node                        # fret.planning.planner_node_ros
    scene_acquisition_node              # fret.scene.acquisition_node
    perception_bridge                   # this package
    controller_node (omx or omy variant)  # fret.control.controller_node
```

---

## Topic Summary

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/obstacle_cloud` | `PointCloud2` | `PerceptionBridgeNode` | `SceneAcquisitionNode`, `PlannerRosNode` |
| `/joint_trajectory` | `JointTrajectory` | `PlannerRosNode` | `ControllerRosNode` |
| `/joint_commands` | `Float64MultiArray` | `ControllerRosNode` | `MuJoCoBridgeNode` |
| `/joint_states` | `JointState` | `MuJoCoBridgeNode` | `StateEstimator`, `ControllerRosNode`, `PlannerRosNode` |
| `/controller_fault` | `Bool` | `ControllerRosNode` | Operator / monitoring |
