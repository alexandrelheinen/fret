# FRET Scenario Library

Scenarios are defined in `src/fret/config/scenarios/` and launched via:

```bash
ros2 launch fret sitl.py scenario:=<name> model:=<model>
```

**Release scenarios** (product targets): SC-v11 – SC-v14 (shipped);
SC-v15 – SC-v17 (vision line, planned).

Full release spec: [releases.md](releases.md). Vision program: [vision/README.md](vision/README.md).

---

## Release scenarios

### SC-v11 — Dubins dual race (v1.1)

**File:** `config/scenarios/dubins_race.yml`  
**Model:** `dubins`

**Purpose:** Two real-scale TurtleBot3 agents race A → B through a compact lab
structure forest (RRT* blue vs SST green). Narrow aisles force planning; wheel
actuators use Menagerie limits (≈ 0.22 m/s).

| Parameter | Value |
|---|---|
| Agents | 2 (RRT* + SST) on freejoint TB3 bodies |
| Environment | 10 m × 10 m floor + AWS clutter visuals on analytic boxes |
| Workspace | 10 × 10 m · start (1.2, 1.2) · goal (8.8, 8.8) |
| Speeds | cruise 0.18 m/s · max 0.22 m/s (real TB3) |
| Control | ARCO path-following MPC (SE(2)) → FRET wheel bridge; grey foil PP |
| Planner | ARCO RRT* (agent 1) + SST (agent 2) |
| Sim mode | Physics SITL (v1.2 default); kinematic mirror via `physics_mode:=false` |

**Pass criteria:** See [releases.md § v1.1](releases.md#v11--dubins-dual-robot-race).
Robot details: [robots/dubins.md](robots/dubins.md).

---

### SC-v12 — MuJoCo physics validation (v1.2)

**Scenarios:** SC-v11 with `physics_mode:=true`

**Purpose:** Validate actuator-driven SITL with contact dynamics for the shipped
Dubins showcase. No new scenario YAML — physics mode is a parameter on the
existing scenario.

**Pass criteria:** See [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl).

Implementation spec: [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md) ·
Config: [config.md § Simulation](config.md#simulation-physics-v12).

---

### SC-v12u — Unit robot physics sandboxes (foundation)

**Assets:** `mjcf/turtlebot3_unit.xml` (primary), `mjcf/diffdrive_unit.xml` (procedural twin)  
**API:** `fret.simulation.TurtleBot3UnitRobot`  
**Requirement:** FR-SIM-11

**Purpose:** Prove the TurtleBot3 physical model works under pure MuJoCo physics
with open-loop commands — before race orchestration.

| Robot | Commands | Pass criteria |
|---|---|---|
| TurtleBot3 unit | `(v, ω)` → wheel speeds | Forward travel; spin-in-place with translation ≤ 5 cm over 2 s; constant `(v, ω)` produces sustained yaw change (arc) |

**CI policy:** optional / local (`tests/simulation/test_*_robot_unit.py`); not required
on every PR gate.

---

### SC-v13a — OpenMANIPULATOR-X empty reach (v1.2.3)

**File:** `config/scenarios/omx_reach.yml`  
**Model:** `open_manipulator_x`

**Purpose:** Validate the full command chain on an empty tabletop — EE pose A → B,
no obstacles.

**Pass criteria:** See [releases.md § v1.2.3](releases.md#v123--openmanipulator-x-tabletop).

### SC-v13b — OpenMANIPULATOR-X pick-and-place (v1.2.3)

**File:** `config/scenarios/omx_pick_place.yml`  
**Model:** `open_manipulator_x`

**Purpose:** Validate the manipulation FSM under full MuJoCo physics — stretch to
pick a Ø 40 mm ball from the floor (green start zone) and drop it into an open
square **place bin** whose walls collide with the proximal arm and the ball.
No transfer-wall obstacles.

**Pass criteria:** See [releases.md § v1.2.3](releases.md#v123--openmanipulator-x-tabletop).

### SC-v13c — OpenMANIPULATOR-X desk clutter (v1.2.3 / v1.4)

**File:** `config/scenarios/omx_desk_clutter.yml`  
**Model:** `open_manipulator_x`

**Purpose:** Floor pick on −Y and **front** place bin on +X; a mid-chord wall
blocks the straight transfer so the planner + controller must retract around
it under full MuJoCo physics (place-bin shell is also in planner occupancy).

### SC-v13d — OpenMANIPULATOR-X dual Γ-wall maze (v1.2.3 / v1.4)

**File:** `config/scenarios/omx_wall_maze.yml`  
**Model:** `open_manipulator_x`

**Purpose:** Front place bin on +X with a **dual Γ** obstacle — stems flank
the place corridor and caps overhang ±Y — so the arm cannot swing either side
and must back out → climb → place. Arm↔wall and arm↔bin collision bitmasks
stay active.

---

### SC-v14a — OpenMANIPULATOR-Y empty reach (v1.2.4)

**File:** `config/scenarios/omy_reach.yml`  
**Model:** `omy`

**Purpose:** Empty tabletop joint-space A→B — validates kinematics, MJCF, and
SITL wiring for the 6-DOF arm (same role as SC-v13a for OM-X).

### SC-v14b — OpenMANIPULATOR-Y floor pick-and-place (v1.2.4 / v1.4)

**File:** `config/scenarios/omy_pick_place.yml`  
**Model:** `omy`

**Purpose:** Pick a Ø 86 mm ball from the **floor** at ~0.46 m forward
reach (same OMX SC-v13b pattern: pads pinch the equator) and place it in an
open square **place bin** at a loaded-reachable pose — 6-DOF analogue of
SC-v13b.

### SC-v14c — OpenMANIPULATOR-Y cluttered floor pick-and-place (v1.2.4 / v1.4)

**File:** `config/scenarios/omy_clutter.yml`  
**Model:** `omy`

**Purpose:** Same floor ball + **front** place bin as SC-v14b, plus a
mid-chord wall between pick (−Y) and place (+X) so the transfer must detour
(6-DOF analogue of SC-v13c).

**Pass criteria:** See [releases.md § v1.2.4](releases.md#v124--6-dof-manipulator-challenge).

---

## Vision line scenarios (planned)

### SC-v15 — Vision pipeline fixtures (v1.3)

**Purpose:** Unit / fixture gates for `fret.vision` (ball centre accuracy). Not a
robot showcase. Algorithm selection is part of the release:
[vision/algorithm-selection.md](vision/algorithm-selection.md).

**Pass criteria:** [releases.md § v1.3](releases.md#v13--computer-vision-pipeline).

### SC-v16a — OpenMANIPULATOR-X CV pick-and-place (v1.4)

**File:** `config/scenarios/omx_pick_place_cv.yml`  
**Model:** `open_manipulator_x` · **MJCF:** same as SC-v13b (`omx_pick_place`)

**Purpose:** Floor ball pick-place with **gate-camera** `BallObservation` driving
pick IK. Place / dispenser stays YAML. `pick_xy` is a test oracle only.

### SC-v16b — OpenMANIPULATOR-Y CV pick-and-place (v1.4)

**File:** `config/scenarios/omy_pick_place_cv.yml`  
**Model:** `omy` · **MJCF:** same as SC-v14b (`omy_pick_place`)

**Purpose:** OMY analogue of SC-v16a. Showcase matrix keeps id `omy_pick_place`
(vision-on by default); this YAML is the explicit SC-v16 contract for tests.

**Pass criteria:** [releases.md § v1.4](releases.md#v14--cv--manipulation-integration).
Camera mount: [vision/camera-layout.md](vision/camera-layout.md).

### SC-v17 — Dynamic ball + industrial place (v1.5)

**Purpose:** Ball rolls to a random rest pose; vision detects and classifies
pickability; pickable balls go into an improved container / dispenser.
Provisional cadence: one ball per cycle.

**Pass criteria:** [releases.md § v1.5](releases.md#v15--dynamic-ball--industrial-place).

---

## Scenario YAML schema

All scenario files use the ROS 2 parameters YAML format (`/**:` →
`ros__parameters:`). They are installed to `share/fret/config/scenarios/` and
loaded by launch, runners, and tests.

### Common fields

| Key | Type | Required | Description |
| --- | --- | --- | --- |
| `scenario_id` | string | yes | Stable identifier (matches filename stem) |
| `model` | string | yes* | Robot model (`dubins`, `open_manipulator_x`, …) |
| `backend` | string | no | Simulator backend (`mujoco` for release scenarios) |
| `start_configuration` | float[] | yes† | Start joint / SE(2) state |
| `goal_configuration` | float[] | yes† | Goal joint / SE(2) state |
| `planning_timeout` | float | no | Planner timeout [s] (default from launch) |
| `duration` | float | no | Scenario run length [s] |
| `physics_mode` | bool | no | `true` → MuJoCo `mj_step` SITL (v1.2, SC-v12) |
| `simulation_config` | string | no | Path to `config/simulation/*.yml` (default `simulation/mujoco.yml`) |
| `recording` | object | no | Bag record toggles and topic list |


† Some regression scenarios use `goal_position` + IK instead of explicit
`start_configuration` (see `static_reach.yml`).

### Dubins race (`dubins_race.yml`)

```yaml
/**:
  ros__parameters:
    scenario_id: "dubins_race"
    model: "dubins"
    backend: "mujoco"

    start_configuration: [1.2, 1.2, 0.785]
    goal_configuration: [8.8, 8.8, 0.0]

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

### v1.2 physics overlay (SC-v12)

No separate scenario file. Enable physics on the release scenario:

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

# v1.1 Dubins race
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins

# v1.2 physics mode
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins physics_mode:=true
```

Pure-Python regression (no ROS):

```bash
```
