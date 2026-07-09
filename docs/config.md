# FRET configuration reference

Runtime parameters live in YAML under `src/fret/config/`. Tunable algorithm and
simulation values must not be hardcoded in Python or MJCF (see
[docs/guidelines.md](guidelines.md#yaml-configuration-files)).

| Directory | Purpose |
| --- | --- |
| `config/scenarios/` | Per-run definitions (start/goal, timeouts, physics flags) |
| `config/controllers/` | Closed-loop gains (`ros__parameters` format) |
| `config/worlds/` | Static obstacle layouts, Dubins vehicle/planner tuning |
| `config/simulation/` | MuJoCo bridge and physics SITL (v1.2+) |

Scenario schema: [scenarios.md](scenarios.md#scenario-yaml-schema).

---

## Simulation / physics (v1.2)

**Spec:** [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md) · **Overview:**
[mujoco.md](mujoco.md)

### Parameter precedence

| Priority | Source | Example |
| --- | --- | --- |
| 1 (highest) | Launch argument | `physics_mode:=true` |
| 2 | Scenario YAML | `physics_mode: true` under `ros__parameters` |
| 3 | Bridge config file | `config/simulation/mujoco.yml` |
| 4 (lowest) | Code defaults | **Forbidden** for tunables — missing keys fail at load |

Launch and scenario values merge into `MuJoCoBridgeNode` parameters at startup.

### `physics_mode`

| Value | Behaviour |
| --- | --- |
| `false` | Kinematic mirror (v1.0–v1.1): Python integration → `qpos` write → `mj_forward` |
| `true` | Physics SITL (v1.2): `/joint_commands` → actuators → `mj_step` → `/joint_states` from sim |

Declared in:

- **Launch:** `ros2 launch fret sitl.py … physics_mode:=true`
- **Scenario:** `physics_mode: true` in `config/scenarios/<name>.yml`

Satisfies FR-SIM-09. SC-v12 runs SC-v10 and SC-v11 scenarios with
`physics_mode: true`.

### `config/simulation/mujoco.yml` schema (v1.2 target)

Consumed by `MuJoCoBridgeNode` / `MuJoCoBridgeCore`. All fields required when
`physics_mode: true`; bridge raises on missing keys.

```yaml
/**:
  ros__parameters:
    # Identity (v1.0)
    model: ppp
    scenario: ppp_warehouse
    update_rate: 50.0
    initial_joint_positions: [0.0, 0.0, 0.0]
    collision_backend: mujoco

    # v1.2 physics
    physics_mode: false
    mjcf_timestep: 0.002          # must match MJCF <option timestep>
    substeps_per_tick: 25         # floor(1 / (mjcf_timestep * update_rate))

    # Contact / regression logging (v1.2)
    contact_log_enabled: true
    contact_log_path: ""          # empty → /tmp/fret_physics/<scenario_id>/contacts.jsonl
    metrics_path: ""              # empty → /tmp/fret_physics/<scenario_id>/metrics.json

    # Per-model actuator tables (one block active per model)
    actuators:
      ppp:
        names: [act_joint_x, act_joint_y, act_joint_z]
        kv: [120.0, 120.0, 180.0]
        forcerange: [[-5000, 5000], [-5000, 5000], [-8000, 8000]]
      dubins:
        names: [act_rrt_x, act_rrt_y, act_rrt_yaw, act_sst_x, act_sst_y, act_sst_yaw]
        kv: [80.0, 80.0, 40.0, 80.0, 80.0, 40.0]
        forcerange: [[-2000, 2000], [-2000, 2000], [-500, 500],
                     [-2000, 2000], [-2000, 2000], [-500, 500]]

    # PPP cargo weld (T12-04)
    cargo_weld:
      equality_name: cargo_weld
      body_parent: z_hoist
      body_child: cargo
```

**MJCF vs YAML rule:** XML declares actuator *existence* and joint binding with
placeholder `kv="1"`. Runtime applies `actuators.<model>.kv` and `forcerange` via
MuJoCo API (`model.actuator_gainprm`, `model.actuator_forcerange`).

### Scenario YAML fields for physics (v1.2)

Add to release scenarios when enabling SC-v12:

```yaml
/**:
  ros__parameters:
    scenario_id: "ppp_warehouse"
    model: "ppp"
    backend: "mujoco"

    physics_mode: false            # set true for SC-v12 / physics SITL

    # Optional Dubins-only override (default true)
    inter_agent_contact: true

    simulation_config: simulation/mujoco.yml   # bridge parameter file
```

Existing v1.0/v1.1 fields (`start_configuration`, `planning_timeout`, `grasp`,
`recording`, …) are unchanged. See [scenarios.md](scenarios.md) for the full
schema.

### Controller configs (unchanged)

PPP and Dubins closed-loop gains remain in `config/controllers/ppp.yml` and
`config/controllers/dubins.yml`. Physics mode does **not** duplicate controller
gains — it only adds actuator execution layer tuning in `simulation/mujoco.yml`.

### Tuning workflow

1. Run kinematic baseline; record EE / pose error from existing tests.
2. Set `physics_mode: true`; start with default `kv` values above.
3. Increase `kv` until tracking error meets FR-CTL-02 limits without actuator saturation.
4. Inspect contact log (§4 in [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md)).
5. Commit tuned values to `simulation/mujoco.yml` only — not Python.
