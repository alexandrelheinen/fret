# FRET Scenario Library

Scenarios are defined in `src/fret/config/scenarios/` and launched via:

```bash
ros2 launch fret sitl.py scenario:=<name> model:=<model>
```

**Release scenarios** (product targets): SC-v10 – SC-v14.
**Regression scenarios** (bootstrap SCARA): SC-01 – SC-05.

Full release spec: [releases.md](releases.md).

---

## Release scenarios

### SC-v10 — PPP warehouse pick-and-place (v1.0)

**File:** `config/scenarios/ppp_warehouse.yml`  
**Model:** `ppp`

**Purpose:** Gantry moves a cargo box from start to goal through warehouse box obstacles
using magnetic grasp.

| Parameter | Value |
|---|---|
| Workspace | 12 × 4 × 3 m (1:5 MJCF preview; full ARCO layout in `ppp_warehouse_obstacles.yml`) |
| Obstacles | Static boxes + shelf racks (ARCO `ppp.yml` preview layout) |
| Grasp | Magnetic weld |
| Planner | ARCO RRT* (MuJoCo collision contacts) |
| Collision | `collision_backend:=mujoco`, `plan_include_cargo:=true` |
| Timeout | 30 s |
| Sim mode | Kinematic mirror (v1.0); physics mode from v1.2 |

**Pass criteria:** See [releases.md § v1.0](releases.md#v10--ppp-gantry-warehouse-pick-and-place).

---

### SC-v11 — Dubins dual race (v1.1)

**File:** `config/scenarios/dubins_race.yml`  
**Model:** `dubins`

**Purpose:** Two Dubins robots race A → B through a warehouse structure forest
(RRT* blue vs SST green). Rectangular posts, walls, and U-shaped dead-end alcoves
create multiple corridors — only proper planning avoids traps.

| Parameter | Value |
|---|---|
| Agents | 2 (RRT* + SST) |
| Environment | AWS warehouse floor + rectangular structure forest |
| Workspace | 80 × 80 m |
| Control | ARCO Pure Pursuit + DubinsVehicle |
| Planner | ARCO RRT* (agent 1) + SST (agent 2) |
| Sim mode | Kinematic mirror (v1.1); physics mode from v1.2 |

**Pass criteria:** See [releases.md § v1.1](releases.md#v11--dubins-dual-robot-race).
Robot details: [robots/dubins.md](robots/dubins.md).

---

### SC-v12 — MuJoCo physics validation (v1.2)

**Scenarios:** SC-v10 + SC-v11 with `physics_mode:=true`

**Purpose:** Validate actuator-driven SITL with contact dynamics for all shipped
robots. No new scenario YAML — physics mode is a parameter on existing scenarios.

**Pass criteria:** See [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl).

Implementation spec: [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md) ·
Config: [config.md § Simulation](config.md#simulation-physics-v12).

---

### SC-v13a — RRP pillars and slabs (v1.3)

**File:** `config/scenarios/rrp_pillars.yml` *(planned)*  
**Model:** `rrp` / `scara`

**Purpose:** Reproduce ARCO `map/rrp.yml` — 3-D C-space with pillars and horizontal slabs.

**Pass criteria:** See [releases.md § v1.3](releases.md#v13--rrp--scara-arco-reproduction).

---

### SC-v13b — RR planar arm (v1.3)

**File:** `config/scenarios/rr_planar.yml` *(planned)*  
**Model:** `rr`

**Purpose:** Reproduce ARCO `map/rr.yml` — 2-DOF planar arm with box obstacles.

---

### SC-v14 — 6-DOF challenge (v1.4)

**File:** `config/scenarios/six_dof_challenge.yml` *(planned)*  
**Model:** `six_dof`

**Purpose:** Capstone — 6-DOF manipulator in cluttered cell.

**Pass criteria:** See [releases.md § v1.4](releases.md#v14--6-dof-manipulator-final-challenge).

---

## Regression scenarios (bootstrap SCARA)

These scenarios validated MS-1–5. They remain in pure-Python CI as regression tests
until v1.3.

| ID | File | Purpose | Status |
|---|---|---|---|
| SC-01 | `static_reach.yml` | Empty-world reach | ✅ Regression |
| SC-02 | `obstacle_avoidance.yml` | Single box avoidance | ✅ Regression |
| SC-03 | `planning_timeout.yml` | Timeout / ABORTED | ✅ Regression |
| SC-04 | `straight_line.yml` | Controller-only tracking | ✅ Regression |
| SC-05 | `arc.yml` | Arc injector | ✅ Regression |
| — | `pillar_avoidance.yml` | Pillar world (MS-5) | ✅ Regression |

---

## Scenario YAML schema

All scenario files use the ROS 2 parameters YAML format (`/**:` →
`ros__parameters:`). They are installed to `share/fret/config/scenarios/` and
loaded by launch, runners, and tests.

### Common fields

| Key | Type | Required | Description |
| --- | --- | --- | --- |
| `scenario_id` | string | yes | Stable identifier (matches filename stem) |
| `model` | string | yes* | Robot model (`ppp`, `dubins`, `scara`, …) |
| `backend` | string | no | Simulator backend (`mujoco` for release scenarios) |
| `start_configuration` | float[] | yes† | Start joint / SE(2) state |
| `goal_configuration` | float[] | yes† | Goal joint / SE(2) state |
| `planning_timeout` | float | no | Planner timeout [s] (default from launch) |
| `duration` | float | no | Scenario run length [s] |
| `physics_mode` | bool | no | `true` → MuJoCo `mj_step` SITL (v1.2, SC-v12) |
| `simulation_config` | string | no | Path to `config/simulation/*.yml` (default `simulation/mujoco.yml`) |
| `recording` | object | no | Bag record toggles and topic list |

\* Regression SCARA scenarios omit `model`; launch passes `model:=scara`.

† Some regression scenarios use `goal_position` + IK instead of explicit
`start_configuration` (see `static_reach.yml`).

### PPP warehouse (`ppp_warehouse.yml`)

```yaml
/**:
  ros__parameters:
    scenario_id: "ppp_warehouse"
    model: "ppp"
    backend: "mujoco"

    start_configuration: [2.0, 1.0, 2.4]
    goal_configuration: [10.5, 1.2, 0.59]

    planning_timeout: 30.0
    duration: 60.0

    collision_backend: analytic      # analytic | mujoco
    planner_algorithm: rrt_star
    plan_include_cargo: true

    physics_mode: false              # true for SC-v12

    grasp:
      capture_radius: 0.45
      goal_radius: 0.5
      weld_offset: [0.0, 0.0, -0.34]
      box_half_extent: 0.25

    recording:
      enabled: true
      topics:
        - /joint_states
        - /joint_commands
        - /joint_trajectory
```

Grasp keys must match [mujoco_physics_v1.2.md §2](mujoco_physics_v1.2.md#2-cargo-weld-physics-ppp)
for physics weld `relpose`.

### Dubins race (`dubins_race.yml`)

```yaml
/**:
  ros__parameters:
    scenario_id: "dubins_race"
    model: "dubins"
    backend: "mujoco"

    start_configuration: [6.0, 6.0, 0.785]
    goal_configuration: [74.0, 74.0, 0.0]

    planning_timeout: 30.0
    duration: 35.0
    race_timeout: 300.0
    simulation_dt: 0.05

    obstacle_file: "worlds/dubins_race_obstacles.yml"

    physics_mode: false
    inter_agent_contact: true        # v1.2 physics: agent–agent collision

    agents:
      rrt_star:
        algorithm: rrt_star
        label: "1"
        color: [0.267, 0.467, 0.800]
      sst:
        algorithm: sst
        label: "2"
        color: [0.267, 0.667, 0.400]

    recording:
      enabled: true
      topics:
        - /joint_states
```

Obstacle layout and Dubins vehicle margins: `config/worlds/dubins_race_obstacles.yml`.

### Regression SCARA (`static_reach.yml` pattern)

```yaml
/**:
  ros__parameters:
    scenario_id: "static_reach"
    goal_position: [-0.2349, 0.0855, 0.05]
    goal_configuration: [1.8273, 2.2974, 0.05]
    skip_ik: false
    planning_timeout: 10.0
    duration: 20.0
    obstacles: []
    recording:
      enabled: true
      topics: [/joint_states, /joint_commands, /joint_trajectory]
```

### v1.2 physics overlay (SC-v12)

No separate scenario file. Enable physics on existing release scenarios:

```yaml
    physics_mode: true
    simulation_config: simulation/mujoco.yml
```

Or at launch: `physics_mode:=true` (overrides YAML when passed explicitly).

Full simulation parameter schema: [config.md](config.md#simulation-physics-v12).

---

## Running scenarios

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# v1.0 PPP warehouse
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp

# v1.1 Dubins race
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins

# v1.2 physics mode (when implemented)
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp physics_mode:=true
```

Pure-Python regression (no ROS):

```bash
python3 -m pytest tests/integration/test_scenario_pillar_avoidance.py -v
```
