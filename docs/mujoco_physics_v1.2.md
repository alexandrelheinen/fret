# MuJoCo physics SITL — v1.2 implementation specification

> **Authoritative v1.2 engineering spec.** Product acceptance criteria remain in
> [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl). Overview and tuning
> workflow: [mujoco.md](mujoco.md).
>
> **Related:** [config.md § Simulation](config.md#simulation-physics-v12) ·
> [scenarios.md § SC-v12](scenarios.md#sc-v12--mujoco-physics-validation-v12) ·
> [requirements.md FR-SIM-07–09](requirements.md#simulation-mujoco)

This document closes the implementation gaps for v1.2: actuator design, cargo weld
physics, configuration schema, contact logging, and regression test contracts.

---

## 1. Robot actuators and MJCF design

### Shared timing model

| Parameter | Value | Source |
| --- | --- | --- |
| Control rate | 50 Hz | `controllers/*.yml` → `update_rate` |
| MuJoCo integrator timestep | 0.002 s | MJCF `<option timestep="0.002">` |
| Substeps per control tick | 25 | `floor(1 / (0.002 × 50))` |

Each bridge timer callback (50 Hz):

1. Read the latest `/joint_commands` velocity vector.
2. Write commands into `data.ctrl` (one actuator per commanded DOF).
3. Call `mj_step` **25 times** (no pose injection between substeps).
4. Read `qpos` / `qvel` and publish `/joint_states`.

Kinematic mirror mode (v1.0–v1.1) is unchanged: integrate in Python, write
`qpos`, call `mj_forward` only.

### Actuator type: velocity servos

FRET v1.2 standardizes on MuJoCo **velocity actuators** (`<actuator><velocity …>`)
for all shipped robots.

| Alternative considered | Why not chosen |
| --- | --- |
| `<motor>` (torque) | Controllers already output joint/world velocities; torque tuning adds an extra gain layer with no v1.0 baseline |
| `<position>` servos | Trajectory is feedforward velocity from PPP P-control / Pure Pursuit, not position PD |
| Direct `qpos` writes in physics mode | Violates FR-SIM-07 (pose teleportation) |

Velocity actuators map 1:1 to existing `/joint_commands` semantics: commanded
value = target joint velocity [m/s] or [rad/s].

Actuator **gains and force limits** live in `config/simulation/mujoco.yml` (see
[config.md](config.md#simulation-physics-v12)). MJCF declares actuator *names* and
joint bindings only; numeric gains must not be hardcoded in XML.

### PPP gantry (`ppp_warehouse.xml`)

**Joints (unchanged):** `joint_x`, `joint_y`, `joint_z` — prismatic slides on the
existing kinematic chain.

**New MJCF block** (insert before `</mujoco>`):

```xml
<actuator>
  <velocity name="act_joint_x" joint="joint_x" kv="1" forcelimited="true" forcerange="-5000 5000"/>
  <velocity name="act_joint_y" joint="joint_y" kv="1" forcelimited="true" forcerange="-5000 5000"/>
  <velocity name="act_joint_z" joint="joint_z" kv="1" forcelimited="true" forcerange="-8000 8000"/>
</actuator>
```

`kv="1"` in MJCF is a placeholder; runtime loads tuned values from YAML via
`model.actuator_gainprm` (T12-02).

**Command mapping** (`MuJoCoBridgeCore.step_physics`):

| `ctrl` index | Actuator | `/joint_commands[i]` |
| --- | --- | --- |
| 0 | `act_joint_x` | X velocity [m/s] |
| 1 | `act_joint_y` | Y velocity [m/s] |
| 2 | `act_joint_z` | Z velocity [m/s] |

**Collision geoms for physics:** enable contact on gantry leg/column geoms and
obstacle boxes (`obs_a`–`obs_d`). Visual-only clutter meshes remain
`contype="0" conaffinity="0"`. Floor plane already participates in contact.

**Validation target:** EE tracking error ≤ 10 mm (releases V12-2, FR-CTL-02).

### Dubins race (`dubins_race.xml`)

**Joints (unchanged):** per agent, `*_joint_x` (slide X), `*_joint_y` (slide Y),
`*_joint_yaw` (hinge Z). Six actuators total.

**New MJCF block:**

```xml
<actuator>
  <!-- Agent 1 (RRT*) -->
  <velocity name="act_rrt_x" joint="rrt_joint_x" kv="1" forcelimited="true" forcerange="-2000 2000"/>
  <velocity name="act_rrt_y" joint="rrt_joint_y" kv="1" forcelimited="true" forcerange="-2000 2000"/>
  <velocity name="act_rrt_yaw" joint="rrt_joint_yaw" kv="1" forcelimited="true" forcerange="-500 500"/>
  <!-- Agent 2 (SST) -->
  <velocity name="act_sst_x" joint="sst_joint_x" kv="1" forcelimited="true" forcerange="-2000 2000"/>
  <velocity name="act_sst_y" joint="sst_joint_y" kv="1" forcelimited="true" forcerange="-2000 2000"/>
  <velocity name="act_sst_yaw" joint="sst_joint_yaw" kv="1" forcelimited="true" forcerange="-500 500"/>
</actuator>
```

**Command mapping** (`DubinsRaceBridgeCore` physics path):

Pure Pursuit / `DubinsVehicle` integrate holonomically in SE(2) today. The bridge
maps each agent's world-frame velocity `(v_x, v_y, ω)` to the three joint
actuators — the same joint layout used for kinematic mirroring in v1.1.

| Agent | `ctrl` indices | Source |
| --- | --- | --- |
| RRT* | 0–2 | world `(v_x, v_y, ω)` from tracking loop |
| SST | 3–5 | world `(v_x, v_y, ω)` from tracking loop |

**Holonomic approximation:** independent X/Y slides do not enforce a non-holonomic
Dubins constraint at the physics layer. Planning still uses ARCO `DubinsVehicle`;
execution uses the existing three-DOF joint mirror with **contact-rich** dynamics.
True differential-drive wheel actuators are deferred to a post-v1.2 refinement if
needed.

**Collision geoms:** `rrt_collision`, `sst_collision`, and all `str_*_col` boxes
must use matching `contype` / `conaffinity` (default contact). Visual meshes stay
non-colliding.

**Inter-agent contact (optional, V12-3):** enabled by default — both collision
boxes participate in the same contact group. Disable via scenario override
`inter_agent_contact: false` when debugging single-agent physics.

**Validation target:** both agents reach goal B without structure penetration;
column contact produces non-zero logged forces (see §4).

### Bridge API additions (T12-01)

| Symbol | Responsibility |
| --- | --- |
| `step_physics(commands, substeps)` | Write `ctrl`, run `mj_step` × substeps, refresh internal state from `qpos`/`qvel` |
| `set_actuator_gains(yaml_section)` | Apply `kv` and force limits from `mujoco.yml` |
| `physics_mode` property | When `True`, timer uses `step_physics`; when `False`, existing `step()` |

Implementation lives in `src/fret/ros/mujoco_bridge.py`. No open-loop
`set_positions()` calls are permitted while `physics_mode` is active (FR-SIM-07).

---
