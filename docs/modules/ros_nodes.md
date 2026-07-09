# ROS Nodes

**Package:** `fret.ros`  
**Source:** `src/fret/ros/`  
**Tests:** `tests/scene/`, `tests/planning/`

> v1.0 adds `mujoco_bridge.py` as the simulator I/O node. See [releases.md](../releases.md)
> and [mujoco.md](../mujoco.md).

---

## Responsibility

The `fret.ros` package contains ROS 2 nodes bridging **MuJoCo** to FRET algorithm layers.

---

## Components

### `mujoco_bridge.py` — MuJoCoBridgeNode (v1.0)

Simulator I/O for all MuJoCo-backed scenarios. Subscribes to velocity commands and
publishes joint state at 50 Hz.

| Symbol | Description |
|---|---|
| `MuJoCoBridgeCore` | Level-3 joint integration against MJCF limits |
| `make_mujoco_bridge_core()` | Factory for model/scenario dispatch |
| `integrate_joint_velocities()` | Kinematic-mode Euler step + limit clip |
| `step_physics()` | **v1.2** — actuator commands + `mj_step` |
| `resolve_mjcf_path()` | Resolve MJCF path for model/scenario |

**Topics:**

| Topic | Type | Direction |
|---|---|---|
| `/joint_commands` | `std_msgs/Float64MultiArray` | Subscribe |
| `/joint_states` | `sensor_msgs/JointState` | Publish |

**Config:** `src/fret/config/simulation/mujoco.yml`

**Demo:** `python3 scripts/demo_mujoco_bridge.py`  
**Tests:** `tests/ros/test_mujoco_bridge.py` — 11 unit tests.

---

### `perception_bridge.py` — PerceptionBridgeNode

Reads obstacle definitions from `config/perception.yaml` and publishes a
`sensor_msgs/PointCloud2` on `/obstacle_cloud` at 1 Hz. Subscribes to `/tf`
to obtain entity poses when obstacles are frame-relative.

Supported obstacle types (configured in `perception.yaml`):
- `box` — samples surface points on all 6 faces
- `cylinder` — samples lateral surface and top-cap points

This node provides the obstacle cloud that feeds `SceneAcquisition`.

**Configuration:** `src/fret/config/perception.yaml`

```yaml
obstacles:
  - name: obstacle_box_a
    type: box
    size: [0.10, 0.10, 0.10]
    pose: {x: 0.25, y: 0.10, z: 0.05}
```

**Tests:** `tests/scene/test_perception_bridge.py`

---

### `straight_line_injector.py` — StraightLineInjector (Milestone 1)

Reads `config/scenarios/straight_line.yml`, generates a linear joint-space
trajectory from `start_configuration` to `goal_configuration`, and publishes
it once on `/joint_trajectory`. Used in the `straight_line` scenario to validate
the controller in isolation (without a planning node).

---

### `arc_injector.py` — ArcInjector (Scenario SC-05)

Generates a Cartesian circular-arc trajectory in the SCARA horizontal plane and
publishes it on `/joint_trajectory`. Used in the `arc` scenario.

Arc parameters (from `config/scenarios/arc.yml`):
- Centre: `(0.30, 0.00)` m
- Radius: `0.15` m
- Angular range: `−45° → +45°`
- Duration: `4 s`, `200` waypoints

**Tests:** `tests/planning/test_arc_injector.py`

---

## Launch Integration

These nodes are wired into `sitl.py` via scenario-conditional launch:

```python
# sitl.py — MuJoCo SITL (T10-06)
launch mujoco.py              # /joint_states publisher

if scenario == "ppp_warehouse":
    perception_ppp_warehouse.yaml
    controller ppp.yml
elif scenario == "dubins_race":
    dubins race orchestration
elif scenario == "straight_line":
    launch StraightLineInjector
elif scenario == "arc":
    launch ArcInjector
else:
    launch PlannerNodeRos + SceneAcquisitionNode + perception.yaml
```

---

## Topic Summary

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/obstacle_cloud` | `PointCloud2` | `PerceptionBridgeNode` | `SceneAcquisitionNode` |
| `/joint_trajectory` | `JointTrajectory` | Injectors / `PlannerNodeRos` | `ControllerRosNode` |
| `/joint_commands` | `Float64MultiArray` | `ControllerRosNode` | `MuJoCoBridgeNode` |
| `/joint_states` | `JointState` | `MuJoCoBridgeNode` | `StateEstimator`, `ControllerRosNode` |
| `/controller_fault` | `Bool` | `ControllerRosNode` | Operator / monitoring |
