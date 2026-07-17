# MuJoCo physics SITL — v1.2 implementation specification

> **v1.2 physics SITL engineering spec (shipped).** Product acceptance criteria:
> [releases.md § v1.2](releases.md#v12--mujoco-physics-sitl). Overview and tuning
> workflow: [mujoco.md](mujoco.md).
>
> **Related:** [config.md § Simulation](config.md#simulation-physics-v12) ·
> [scenarios.md § SC-v12](scenarios.md#sc-v12--mujoco-physics-validation-v12) ·
> [requirements.md FR-SIM-07–09](requirements.md#simulation-mujoco)

This document closes the implementation gaps for v1.2: actuator design,
configuration schema, contact logging, and regression test contracts.

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

Kinematic mirror mode (v1.1) is unchanged: integrate in Python, write
`qpos`, call `mj_forward` only.

### Actuator type: velocity servos

FRET v1.2 standardizes on MuJoCo **velocity actuators** (`<actuator><velocity …>`)
for all shipped robots.

| Alternative considered | Why not chosen |
| --- | --- |
| `<motor>` (torque) | Controllers already output joint/world velocities; torque tuning adds an extra gain layer with no v1.1 baseline |
| `<position>` servos | Trajectory is feedforward velocity from Pure Pursuit, not position PD |
| Direct `qpos` writes in physics mode | Violates FR-SIM-07 (pose teleportation) |

Velocity actuators map 1:1 to existing `/joint_commands` semantics: commanded
value = target joint velocity [m/s] or [rad/s].

Actuator **gains and force limits** live in `config/simulation/mujoco.yml` (see
[config.md](config.md#simulation-physics-v12)). MJCF declares actuator *names* and
joint bindings only; numeric gains must not be hardcoded in XML.

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

ARCO Dubins / Pure Pursuit still plan and track in SE(2). Controllers emit
body-frame twist `(v, ω)` (or world `(v_x, v_y, ω)` which the bridge rotates into
body frame). The bridge maps that twist to left/right wheel rates and writes the
six wheel velocity actuators before `mj_step` — no post-step `qvel` surgery.

| Agent | `ctrl` actuators | Source |
| --- | --- | --- |
| RRT* | `rrt_wheel_left`, `rrt_wheel_right` | body `(v, ω)` → wheel rates |
| SST | `sst_wheel_left`, `sst_wheel_right` | body `(v, ω)` → wheel rates |
| Dummy | `dummy_wheel_left`, `dummy_wheel_right` | body `(v, ω)` → wheel rates |

**Unit sandbox (FR-SIM-11):** `mjcf/turtlebot3_unit.xml` +
`fret.simulation.TurtleBot3UnitRobot` is the primary true two-wheel model
(`diffdrive_unit` remains a procedural twin). Neither path calls
`enforce_slide_yaw_nonholonomic_qvel` — wheel friction alone provides non-holonomy.

**Collision geoms:** `rrt_collision`, `sst_collision`, and all `str_*_col` boxes
must use matching `contype` / `conaffinity` (default contact). Visual meshes stay
non-colliding.

**Inter-agent contact (optional, V12-2):** enabled by default — both collision
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

## 4. Contact logging and regression test contract

### Contact log format (T12-03)

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
{"sim_time": 12.04, "wall_time": 1710000000.12, "geom1": "rrt_collision", "geom2": "str_a_col", "force_norm": 42.3, "pos": [4.01, 1.18, 0.52]}
```

CI uploads this file as artifact `physics-contacts-<scenario_id>` (V12-4).

### Sim-time metrics file

Written once at scenario shutdown to `metrics_path` (default:
`/tmp/fret_physics/<scenario_id>/metrics.json`):

```json
{
  "scenario_id": "dubins_race",
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

### Regression test files (T12-04, V12-6)

| Test file | Scenario | Minimum assertions |
| --- | --- | --- |
| `tests/integration/test_mujoco_physics_dubins.py` | SC-v11 + `physics_mode` | Both agents reach goal; `penetration_violations == 0`; column contacts logged (`force_norm > 0`); optional inter-agent test |

Tests run under the integration job in `.github/workflows/tests.yml`.

### Kinematic vs physics regression clip (T12-05)

Showcase scripts accept `--physics-mode` (off by default):

```bash
./scripts/video.sh --model dubins --scenario dubins_race … --physics-mode
```

Post-run comparison (manual or CI optional job):

| Metric | Threshold | Action if exceeded |
| --- | --- | --- |
| Path length ratio (physics / kinematic) | ≤ 1.15 | Review actuator gains |
| Goal position error | ≤ FR-CTL-02 limit | Fail V12-2 |
| MP4 frame SSIM | ≥ 0.85 | Warning only (visual regression) |

Store side-by-side clips under `/tmp/fret_physics/<scenario_id>/regression/`.

### Smoke extension

`scripts/tests/smoke.sh` gains a physics stanza (after T12-01):

```bash
timeout 30 ros2 launch fret sitl.py scenario:=dubins_race model:=dubins physics_mode:=true
```

Exit 0 within timeout satisfies V12-1 launch criterion.
