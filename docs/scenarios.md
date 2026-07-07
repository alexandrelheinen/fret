# FRET Scenario Library

Scenarios are defined in `src/fret/config/scenarios/` and launched via:

```bash
ros2 launch fret sitl.py scenario:=<name> model:=<model> backend:=mujoco
```

**Release scenarios** (product targets): SC-v10 – SC-v13.
**Regression scenarios** (bootstrap SCARA): SC-01 – SC-05.

Full release spec: [releases.md](releases.md).

---

## Release scenarios

### SC-v10 — PPP warehouse pick-and-place (v1.0)

**File:** `config/scenarios/ppp_warehouse.yml`  
**Model:** `ppp` · **Backend:** `mujoco`

**Purpose:** Gantry moves a cargo box from start to goal through warehouse box obstacles
using magnetic grasp.

| Parameter | Value |
|---|---|
| Workspace | 60 × 20 × 6 m |
| Obstacles | Static boxes (ARCO `ppp.yml` layout) |
| Grasp | Magnetic weld |
| Planner | ARCO SST |
| Timeout | 30 s |

**Pass criteria:** See [releases.md § v1.0](releases.md#v10--ppp-gantry-warehouse-pick-and-place).

---

### SC-v11 — Dubins dual race (v1.1)

**File:** `config/scenarios/dubins_race.yml` *(planned)*  
**Model:** `dubins` · **Backend:** `mujoco`

**Purpose:** Two Dubins robots race A → B through columns of varied height.

| Parameter | Value |
|---|---|
| Agents | 2 |
| Environment | Column forest + optional walls |
| Control | ARCO Pure Pursuit |
| Planner | ARCO SST per agent |

**Pass criteria:** See [releases.md § v1.1](releases.md#v11--dubins-dual-robot-race).

---

### SC-v12a — RRP pillars and slabs (v1.2)

**File:** `config/scenarios/rrp_pillars.yml` *(planned)*  
**Model:** `rrp` / `scara`

**Purpose:** Reproduce ARCO `map/rrp.yml` — 3-D C-space with pillars and horizontal slabs.

**Pass criteria:** See [releases.md § v1.2](releases.md#v12--rrp--scara-arco-reproduction).

---

### SC-v12b — RR planar arm (v1.2)

**File:** `config/scenarios/rr_planar.yml` *(planned)*  
**Model:** `rr`

**Purpose:** Reproduce ARCO `map/rr.yml` — 2-DOF planar arm with box obstacles.

---

### SC-v13 — 6-DOF challenge (v1.3)

**File:** `config/scenarios/six_dof_challenge.yml` *(planned)*  
**Model:** `six_dof`

**Purpose:** Capstone — 6-DOF manipulator in cluttered cell.

**Pass criteria:** See [releases.md § v1.3](releases.md#v13--6-dof-manipulator-final-challenge).

---

## Regression scenarios (bootstrap SCARA)

These scenarios validated MS-1–5. They remain in CI as regression tests until v1.2.

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
  release: v1.0                    # v1.0 | v1.1 | v1.2 | v1.3
  description: "..."

robot:
  model: ppp                       # ppp | dubins | rrp | scara | rr | six_dof

simulation:
  backend: mujoco                  # mujoco | gazebo

world:
  file: worlds/ppp_warehouse.sdf   # backend-specific (optional)
  obstacles: []                    # or inline box list

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

# v1.0 (when implemented)
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco

# Regression (bootstrap SCARA)
ros2 launch fret sitl.py scenario:=static_reach model:=scara backend:=gazebo
```
