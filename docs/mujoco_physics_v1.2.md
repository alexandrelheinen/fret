# MuJoCo physics SITL — v1.2 implementation specification

> **v1.2 physics SITL engineering spec (shipped).** Product acceptance criteria:
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

## 2. Cargo weld physics (PPP)

### Problem

v1.0 magnetic grasp is **logical**: `MagneticGraspFSM` sets cargo pose relative to
the EE during TRANSPORT, and the bridge writes `cargo` body `qpos` directly
(kinematic mirror). Planning uses the welded AABB via `cargo_corners()`.

v1.2 requires the cargo to follow the EE through **physical coupling** while
contacts with obstacles produce forces (V12-2).

### Chosen approach: toggled MJCF equality weld

| Alternative | Verdict |
| --- | --- |
| Fixed weld for entire episode | Cannot release at goal |
| Mocap `cargo_floor` body | Mocap bodies do not participate in contact dynamics the same way as free bodies |
| Manual per-step pose sync in physics mode | Pose teleportation — violates FR-SIM-07 |
| **`<equality type="weld">` toggled by bridge** | **Selected** — native MuJoCo constraint, release by deactivating constraint |

### MJCF changes (`ppp_warehouse.xml`)

1. **Cargo body** — add a `freejoint` on the `cargo` body so it can settle after
   release (remove fixed child offset-only placement during physics episodes):

```xml
<body name="cargo" pos="0 0 -0.34">
  <freejoint name="cargo_free"/>
  <geom name="cargo_box" type="box" size="0.25 0.25 0.25" material="cargo" mass="2.0"/>
</body>
```

Initial spawn pose is set from scenario `start_configuration` + grasp
`weld_offset` at episode start (same as v1.0 pick zone).

2. **Weld constraint** — add near the end of the MJCF (inactive by default):

```xml
<equality>
  <!-- Activated by bridge when MagneticGraspFSM enters CAPTURE/TRANSPORT -->
  <weld name="cargo_weld" body1="z_hoist" body2="cargo"
        relpose="0 0 -0.34 1 0 0 0" active="false"/>
</equality>
```

`relpose` matches grasp config `weld_offset: [0.0, 0.0, -0.34]` from
`ppp_warehouse.yml`. Offset updates must stay in YAML, not Python defaults.

### Bridge ↔ FSM hook (T12-04)

| FSM transition | Bridge action |
| --- | --- |
| IDLE → CAPTURE (distance < `capture_radius`) | Set `model.eq_active[eq_id_cargo_weld] = 1`, call `mj_forward` |
| CAPTURE → TRANSPORT | Keep weld active |
| TRANSPORT → RELEASE (distance to goal < `goal_radius`) | Set `eq_active = 0`; cargo freejoint integrates under gravity |
| RELEASE → IDLE | Hide or reposition floor mocap cargo visual if used |

**Planning collision predicate** is unchanged: `MagneticGraspFSM.is_welded` and
`cargo_corners()` still drive `CSpaceCheckerPPP` / MuJoCo checker during TRANSPORT.
Physics weld and planning envelope must use the same `weld_offset` from scenario
grasp config.

### Contact behaviour during transport

- Welded cargo + obstacle contacts generate constraint forces on the gantry.
- Controller must overcome contact disturbances; tune Z actuator `forcerange` and
  `kv` if vertical deflection exceeds 10 mm.
- On RELEASE, cargo must rest inside the goal zone without interpenetrating
  obstacles (logged contact force peak < threshold at settle — see §4).

### Kinematic mode compatibility

When `physics_mode: false`, the bridge keeps v1.0 behaviour: set `cargo` pose
from FSM, ignore equality weld. Showcase scripts (`render_mujoco.py`) default to
kinematic mode until `--physics-mode` is passed (T12-07).

---

## 4. Contact logging and regression test contract

### Contact log format (T12-05)

When `contact_log_enabled: true`, the bridge appends one JSON object per line
(JSONL) after each control tick that reports `data.ncon > 0`.

**Default path:** `/tmp/fret_physics/<scenario_id>/contacts.jsonl`

| Field | Type | Description |
| --- | --- | --- |
| `sim_time` | float | MuJoCo simulation time [s] |
| `wall_time` | float | Unix timestamp [s] |
| `geom1` | string | First geom name |
| `geom2` | string | Second geom name |
| `force_norm` | float | Contact force magnitude [N] |
| `pos` | [float×3] | Contact position in world frame [m] |

Example line:

```json
{"sim_time": 12.04, "wall_time": 1710000000.12, "geom1": "cargo_box", "geom2": "obs_a", "force_norm": 42.3, "pos": [4.01, 1.18, 0.52]}
```

CI uploads this file as artifact `physics-contacts-<scenario_id>` (V12-5).

### Sim-time metrics file

Written once at scenario shutdown to `metrics_path` (default:
`/tmp/fret_physics/<scenario_id>/metrics.json`):

```json
{
  "scenario_id": "ppp_warehouse",
  "physics_mode": true,
  "sim_time_final": 58.2,
  "wall_time_elapsed": 61.4,
  "max_tracking_error_m": 0.008,
  "contact_event_count": 127,
  "max_contact_force_n": 156.0,
  "penetration_violations": 0
}
```

`penetration_violations` counts timesteps where agent–obstacle interpenetration
(`contact.dist < -1 mm`) persists for **two consecutive** physics ticks, filtering
single-tick numerical noise across MuJoCo builds.

### Regression test files (T12-06, V12-7)

| Test file | Scenario | Minimum assertions |
| --- | --- | --- |
| `tests/integration/test_mujoco_physics_ppp.py` | SC-v10 + `physics_mode` | SITL completes; weld engage/release; `max_tracking_error_m ≤ ee_error_limit_physics_m`; `penetration_violations == 0`; contact log exists |
| `tests/integration/test_mujoco_physics_dubins.py` | SC-v11 + `physics_mode` | Both agents reach goal; `penetration_violations == 0`; column contacts logged (`force_norm > 0`); optional inter-agent test |

Tests run under the integration job in `.github/workflows/tests.yml` when
physics implementation lands. Until then, mark `@pytest.mark.xfail` with reason
`T12-01 not implemented`.

### Kinematic vs physics regression clip (T12-07)

Showcase scripts accept `--physics-mode` (off by default):

```bash
./scripts/video.sh --model ppp --scenario ppp_warehouse … --physics-mode
```

Post-run comparison (manual or CI optional job):

| Metric | Threshold | Action if exceeded |
| --- | --- | --- |
| Path length ratio (physics / kinematic) | ≤ 1.15 | Review actuator gains |
| Goal position error | ≤ FR-CTL-02 limit | Fail V12-2 / V12-3 |
| MP4 frame SSIM | ≥ 0.85 | Warning only (visual regression) |

Store side-by-side clips under `/tmp/fret_physics/<scenario_id>/regression/`.

### Smoke extension

`scripts/tests/smoke.sh` gains a physics stanza (after T12-01):

```bash
timeout 30 ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp physics_mode:=true
timeout 30 ros2 launch fret sitl.py scenario:=dubins_race model:=dubins physics_mode:=true
```

Exit 0 within timeout satisfies V12-1 launch criterion.
