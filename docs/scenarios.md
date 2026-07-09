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

```yaml
scenario:
  name: ppp_warehouse
  release: v1.0                    # v1.0 | v1.1 | v1.2 | v1.3 | v1.4
  description: "..."

robot:
  model: ppp                       # ppp | dubins | rrp | scara | rr | six_dof

simulation:
  physics_mode: false              # true from v1.2 for mj_step SITL

world:
  obstacles: []                    # or inline box list; MJCF in src/fret/mjcf/

task:
  start_configuration: [1.0, 1.0, 0.0]
  goal_configuration:  [59.0, 19.0, 0.0]

grasp:
  mode: magnetic                   # v1.0 only
  capture_radius: 0.3
  goal_radius: 0.5

planner:
  algorithm: sst
  planning_timeout: 30.0
  collision_backend: mujoco

controller:
  config: config/controllers/ppp.yml

recording:
  enabled: true
  video: true
  topics: [/joint_states, /joint_commands, /joint_trajectory]
```

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
