# ROS Nodes

**Package:** `fret.ros`  
**Source:** `src/fret/ros/`  
**Tests:** `tests/scene/`, `tests/planning/`

> v1.0 adds `mujoco_bridge.py` as the primary simulator I/O node. See [releases.md](../releases.md).

---

## Responsibility

The `fret.ros` package contains ROS 2 nodes bridging simulators (MuJoCo, Gazebo) to
FRET algorithm layers.

---

## Components

### `perception_bridge.py` — PerceptionBridgeNode

Reads obstacle definitions from `config/perception.yaml` and publishes a
`sensor_msgs/PointCloud2` on `/obstacle_cloud` at 1 Hz. Subscribes to `/tf`
to obtain Gazebo entity poses.

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
# sitl.py — scenario routing
if scenario == "straight_line":
    launch StraightLineInjector
elif scenario == "arc":
    launch ArcInjector
else:                           # static_reach, obstacle_avoidance, ...
    launch PlannerNodeRos + SceneAcquisitionNode + PerceptionBridgeNode
```

---

## Topic Summary

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/obstacle_cloud` | `PointCloud2` | `PerceptionBridgeNode` | `SceneAcquisitionNode` |
| `/joint_trajectory` | `JointTrajectory` | Injectors / `PlannerNodeRos` | `ControllerRosNode` |
| `/joint_commands` | `Float64MultiArray` | `ControllerRosNode` | Gazebo / `BridgeNode` |
| `/joint_states` | `JointState` | Gazebo | `StateEstimator`, `ControllerRosNode` |
| `/controller_fault` | `Bool` | `ControllerRosNode` | Operator / monitoring |
