# Scenario Specification – Issue 02: Gazebo Obstacle World

## 1. Scope

This document specifies the deterministic Gazebo scenario delivered for
ARCO milestone Issue 02.  It defines world geometry, obstacle identifiers,
target pose definitions, reachability assumptions, and startup success
criteria so that the scenario can be independently replicated and validated.

---

## 2. Robot Selection

| Property         | Value                                    |
|------------------|------------------------------------------|
| Model            | Universal Robots UR3                     |
| Degrees of Freedom | 6 revolute joints                      |
| Nominal reach    | ≈ 500 mm from base                       |
| Package          | `ur_description` (ROS 2 Jazzy)           |
| URDF source      | `ur_description/urdf/ur.urdf.xacro`      |
| Launch default   | `model:=ur3`                             |

The UR3 was selected because it is:
- Already declared as a dependency in `package.xml`.
- Supported by `fret.launch.model.resolve_robot_model`.
- Compact enough to exercise narrow-passage constraints within a 1 m × 1 m
  tabletop volume.

---

## 3. Coordinate Frame

All positions are expressed in the **world** frame (REP-105 / ISO 8855):
- **X** – forward (away from the robot base)
- **Y** – left
- **Z** – up

The robot `base_link` is placed at the world origin `(0, 0, 0)`.

---

## 4. World File

| File                                        | Format      |
|---------------------------------------------|-------------|
| `src/fret/worlds/arco_scenario.sdf`         | SDF 1.9     |

The world is installed to `share/fret/worlds/arco_scenario.sdf` by the
`CMakeLists.txt` install directive.

---

## 5. Obstacle Definitions

All obstacle models are declared `<static>true</static>` so the scenario
can be replayed with an identical initial state.

| Model name              | Shape | Pose `(x, y, z)` m | Half-extents `(dx/2, dy/2, dz/2)` m | Challenge              |
|-------------------------|-------|---------------------|---------------------------------------|------------------------|
| `obstacle_pillar_left`  | box   | (0.30,  0.22, 0.25) | (0.025, 0.025, 0.25)                  | Left wall of gate      |
| `obstacle_pillar_right` | box   | (0.30, −0.22, 0.25) | (0.025, 0.025, 0.25)                  | Right wall of gate     |
| `obstacle_crossbar`     | box   | (0.30,  0.00, 0.52) | (0.025, 0.25,  0.025)                 | Top bar / occlusion    |
| `obstacle_block_a`      | box   | (0.45,  0.10, 0.05) | (0.05,  0.05,  0.05)                  | Post-gate local minimum|
| `obstacle_block_b`      | box   | (0.45, −0.10, 0.05) | (0.05,  0.05,  0.05)                  | Post-gate local minimum|

### 5.1 Challenge Description

**Gate structure** (`obstacle_pillar_left`, `obstacle_pillar_right`,
`obstacle_crossbar`): forms a U-shaped frame with a 390 mm lateral opening
at base level and a 440 mm vertical clearance at the top of the pillars.
The arm must thread its links through this opening to reach the target region.

**Post-gate blocks** (`obstacle_block_a`, `obstacle_block_b`): introduce
symmetric local minima that challenge greedy planners when the end-effector
approaches the target region from the far side of the gate.

---

## 6. Region Definitions

Region markers are **visual-only** (no collision geometry) and serve as
reference points for specifying start and goal poses.

| Model name      | Color | Pose `(x, y, z)` m   | Purpose                          |
|-----------------|-------|-----------------------|----------------------------------|
| `region_start`  | green | (0.15,  0.30, 0.005)  | Canonical start pose projection  |
| `region_target` | red   | (0.42, −0.18, 0.005)  | Canonical goal  pose projection  |

### 6.1 Canonical Joint-Space Poses

The following joint-space vectors correspond to end-effector poses above
each floor marker.  Angles are in radians; zero is the UR3 fully-extended
upright posture.

**Start pose** (end-effector above `region_start`):

```
[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
= [0.70, -1.40, 1.40, -1.57, -0.70, 0.00]
```

**Goal pose** (end-effector above `region_target`):

```
[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
= [-0.45, -1.20, 1.20, -1.57,  0.45, 0.00]
```

These values are indicative; exact IK solutions depend on the loaded URDF
and the TF tree published at runtime.

---

## 7. Support Structures

| Model name    | Description                                     |
|---------------|-------------------------------------------------|
| `ground_plane`| 20 m × 20 m static plane at z = 0              |

No additional pedestals are required; the UR3 is mounted directly on the
floor, consistent with the default `spawn_z=0.0` in `sim.py`.

---

## 8. Launch Integration

```bash
# Single-command launch (acceptance criterion)
ros2 launch fret arco_scenario.py
```

The `arco_scenario.py` launcher composes `sim.py` with fixed overrides:

| Argument | Value                                      |
|----------|--------------------------------------------|
| `model`  | `ur3`                                      |
| `world`  | `<share>/worlds/arco_scenario.sdf`         |

The `sim.py` launcher was extended with a `world` argument
(`default_value="empty.sdf"`) to preserve backward compatibility with the
existing `scara` smoke tests.

**Topics published at runtime:**

| Topic                           | Type                             | Source                  |
|---------------------------------|----------------------------------|-------------------------|
| `/robot_description`            | `std_msgs/String`                | `robot_state_publisher` |
| `/joint_states`                 | `sensor_msgs/JointState`         | gz_bridge               |
| `/tf`                           | `tf2_msgs/TFMessage`             | `robot_state_publisher` |
| `/tf_static`                    | `tf2_msgs/TFMessage`             | `robot_state_publisher` |
| `/clock`                        | `rosgraph_msgs/Clock`            | gz_bridge               |

---

## 9. Startup Success Criteria

| Criterion                                   | Timeout | Verification method                    |
|---------------------------------------------|---------|----------------------------------------|
| Gazebo process starts without error         | 15 s    | exit code 0 or 124 (timeout) from CI  |
| `robot_state_publisher` emits `/tf`         | 5 s     | `ros2 topic echo /tf --once`           |
| All 7 SDF models present in entity list     | 10 s    | `gz topic -e /world/arco_scenario/...` |
| No immediate ROS error logs                 | 5 s     | smoke-test log inspection              |

The CI smoke test uses a 20 s wall-clock timeout and accepts exit codes
0 (clean exit) and 124 (timeout / still running) as success.

---

## 10. Reproducibility

- All obstacle poses are hard-coded in `arco_scenario.sdf`; no random
  seed or dynamic spawning is involved.
- The SDF file is version-controlled in `src/fret/worlds/`.
- `<static>true</static>` prevents any physics drift across runs.
- The launch file passes no dynamic arguments to Gazebo, so every run
  starts from the same world state.

---

## 11. Validation Artifacts

A screenshot of the loaded scenario (Gazebo GUI + RViz) is to be captured
during integration testing and stored under `docs/arco/images/` once a
display is available in CI.

The smoke test in `.github/workflows/tests.yml` (`arco_scenario_launch`)
serves as the automated validation artifact for every pull request.

---

## 12. Dependencies

- Issue 01: Frame and interface contracts (defines the `world` canonical
  frame and `/joint_states` topic contract used here).
