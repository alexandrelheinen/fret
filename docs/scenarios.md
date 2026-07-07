# FRET Scenario Library

Named SITL scenarios for **Level 1 (functional)** and **Level 2 (integration)** validation.
Each scenario is a named, reproducible run defined by a YAML file in `config/scenarios/`.

A scenario **passes** when all its listed pass criteria are met simultaneously.
Scenarios are executed via `launch_testing` in `tests/integration/` and can also be
run manually with `ros2 launch fret sitl.py scenario:=<name>`.

---

## SC-01 — Static Reach

**File:** `config/scenarios/static_reach.yml`
**Purpose:** Validate the full end-to-end SITL pipeline (scene → planning →
post-processing → control → execution) in an obstacle-free environment.
**Requirements:** FR-SYS-02, FR-PLN-01, FR-PLN-06, FR-CTL-01, FR-CTL-02

### Setup

| Parameter | Value |
|---|---|
| Robot | SCARA |
| World | Empty (no obstacles) |
| Start configuration | `[0.0, 0.0, 0.10]` — home pose |
| Goal configuration | `[0.785, 0.524, 0.05]` — approx. 45°, 30°, 5 cm |
| Planning timeout | 30 s |
| Controller gains | `config/controllers/jacobian.yml` defaults |

### Pass Criteria

1. Planning Action returns `SUCCESS` within 30 s.
2. Post-processed trajectory has ≥ 2 waypoints and no discontinuities (C² smooth).
3. Controller publishes to `/joint_commands` at ≥ 45 Hz for the full trajectory duration.
4. Maximum end-effector position error in world space ≤ 5 mm at every timestep.
5. No `/fault` message is published at any point during execution.

---

## SC-02 — Obstacle Avoidance

**File:** `config/scenarios/obstacle_avoidance.yml`
**Purpose:** Validate C-space collision avoidance. The planner must find a path that
clears a known static obstacle; direct joint-space interpolation would collide.
**Requirements:** FR-SCN-01–04, FR-PLN-01, FR-PLN-02

### Setup

| Parameter | Value |
|---|---|
| Robot | SCARA |
| World | Single box obstacle: center `(0.20, 0.10, 0.15)` m in world frame, size `0.05 × 0.05 × 0.05` m |
| Start configuration | `[0.0, 0.0, 0.10]` |
| Goal configuration | `[1.047, 0.0, 0.05]` — approx. 60°, 0°, 5 cm |
| Planning timeout | 30 s |

### Pass Criteria

1. Planning Action returns `SUCCESS` within 30 s.
2. For every waypoint `q` in the returned path: `FK(q)` world-frame end-effector
   position has clearance ≥ 0.02 m from the obstacle surface (as reported by
   `KDTreeOccupancy.clearance()`).
3. Controller execution end-effector error ≤ 5 mm throughout.
4. No `/fault` message is published.

---

## SC-03 — Planning Failure / Timeout

**File:** `config/scenarios/planning_timeout.yml`
**Purpose:** Validate the failure-mode behavior. The goal maps to a configuration
whose FK falls inside a bounding obstacle, making the planning problem infeasible.
**Requirements:** FR-PLN-04, FR-PLN-05, FR-PLN-07

### Setup

| Parameter | Value |
|---|---|
| Robot | SCARA |
| World | Box obstacle at `(0.30, 0.0, 0.10)` m, size `0.30 × 0.30 × 0.20` m (large, enclosing goal) |
| Goal configuration | `[0.785, 0.0, 0.10]` — FK maps inside obstacle bounding box |
| Planning timeout | 10 s (shortened to keep CI fast) |

### Pass Criteria

1. Planning Action returns `ABORTED` within `timeout + 2 s` (12 s total).
2. Error code is `TIMEOUT` or `NO_PATH_FOUND` (either is acceptable for this scenario).
3. No `JointTrajectory` is published on `/joint_trajectory`.
4. No `/fault` message is published (fault is a controller-layer concern only).

---

## SC-04 — Straight-Line Trajectory Tracking

**File:** `config/scenarios/straight_line.yml`
**Purpose:** Validate the controller in isolation using a pre-computed trajectory
published directly, bypassing the planning node entirely.
**Requirements:** FR-CTL-01, FR-CTL-02, FR-CTL-03, FR-CTL-04

### Setup

| Parameter | Value |
|---|---|
| Robot | SCARA |
| Trajectory source | Analytically generated: linear interpolation in joint space from `[0.0, 0.0, 0.10]` to `[0.785, 0.0, 0.05]`, duration 5 s, 250 waypoints |
| How injected | Published directly to `/joint_trajectory` by the test harness (no planning node) |

### Pass Criteria

1. Controller publishes to `/joint_commands` at ≥ 45 Hz (50 Hz target, 10% tolerance)
   for the full 5 s trajectory duration.
2. Maximum end-effector position error ≤ 5 mm at every timestep.
3. All published joint velocity commands are within the operational envelope limits
   (≤ 1.57 rad/s for revolute, ≤ 0.1 m/s for prismatic).
4. TF2 broadcast `base_link → tool0` is present and updated throughout execution.
5. No `/fault` message is published.

---

## Running Scenarios

### Manual execution

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch fret sitl.py scenario:=static_reach      model:=scara
ros2 launch fret sitl.py scenario:=obstacle_avoidance model:=scara
ros2 launch fret sitl.py scenario:=planning_timeout   model:=scara
ros2 launch fret sitl.py scenario:=straight_line      model:=scara
```

### Automated (CI)

Integration tests in `tests/integration/` wrap these scenarios with quantitative
assertions using `launch_testing`. Each `test_scenario_<name>.py` file:

1. Launches the SITL stack via `launch_testing.actions.launch_process`.
2. Subscribes to relevant topics with a timeout.
3. Asserts quantitative pass criteria from the scenario definition above.
4. Tears down the launch cleanly on pass or fail.

The integration test workflow is defined in `.github/workflows/integration.yml`.

---

## Scenario YAML Schema

Each scenario YAML must conform to the following structure:

```yaml
scenario:
  name: static_reach                   # matches file name
  description: "..."

robot:
  model: scara                         # selects URDF and kinematics config

world:
  file: worlds/empty.sdf               # Gazebo world file path (relative to share/fret/)
  obstacles: []                        # list of programmatic obstacles (optional)

task:
  start_configuration: [0.0, 0.0, 0.10]
  goal_configuration:  [0.785, 0.524, 0.05]

planner:
  algorithm: sst                       # sst | rrt_star
  planning_timeout: 30.0              # seconds

controller:
  config: config/controllers/jacobian.yml

recording:
  enabled: false                       # set true to record a ROS bag
  topics: [/joint_states, /joint_commands, /joint_trajectory]
```
