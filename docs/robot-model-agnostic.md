# Robot-model-agnostic core (v1.5.0)

Durable spec for `fret.robots`, the robot registry introduced in `v1.5.0`
(the last minor of major 1). Linked from [README.md § Architecture](../README.md#architecture).
Traceability ids: `FR-SYS-06` through `FR-SYS-15` in
[requirements.md](requirements.md).

## Intent

Before `v1.5.0`, the robot model was resolved by hardcoded string-literal
dispatch chains spread across `control/kinematics.py`, `config_loader.py`,
`sitl_config.py`, `control/controller_node.py`,
`control/pick_place_planning.py`, `control/pick_place_vision.py`,
`control/pick_place_clutter_sim.py`, `control/omy_pick_place_sim.py` and
`ros/mujoco_bridge.py`. Adding a robot meant editing core algorithm
modules. `v1.5.0` makes the robot a configuration choice: a scenario names
a `model:`, a robot profile YAML describes it, and per-model behavior
(kinematics, controller construction, and — where migrated, see Status —
transfer-planner strategy, simulation I/O, default vision config) is
selected through a registry of variants (`fret.robots`). This is the
foundation PR2 builds on to add the SO-101 without touching core modules.

## Design

- **`src/fret/robots/profile.py`** — `RobotProfile`: a frozen dataclass
  loaded and validated from `config/robots/<model>.yml` (`FR-SYS-07`).
  Required fields: `name`, `kind` (`mobile` | `arm`), `joint_names`,
  `joint_limits`, `kinematics`, `controller_config`, `planning_config`.
  Optional: `aliases`, `mjcf`, `vision_config`, `grasp_modes`,
  `transfer_planner`, `uses_vision`.
- **`src/fret/robots/registry.py`** — `RobotRegistry`: resolves a model
  key or alias to its profile (`FR-SYS-06`) and, through per-kind variant
  tables, to a kinematics backend and (for arm models with a Level-4 ROS
  controller) a controller factory. `get_default_registry()` is a
  process-wide lazy singleton rebuilt from `config/robots/*.yml` each time
  it is constructed fresh (never mutated global state; see Edge cases).
  `register_kinematics` / `register_controller` /
  `register_transfer_planner` / `register_sim_io` are decorators variant
  modules use to register factories; a duplicate key raises
  `RobotRegistrationError`.
- **`src/fret/robots/protocols.py`** — `typing.Protocol` definitions:
  `KinematicsBackend` (moved here, public, from the private
  `_KinematicsBackend` that lived in `control/kinematics.py`),
  `ControllerFactory`, `TransferPlannerStrategy`, `SimIOAdapter`, `RobotIO`.
- **`src/fret/robots/variants/`** — one module per model family
  (`dubins.py`, `open_manipulator_x.py`, `open_manipulator_y.py`),
  registering factories with `@register_kinematics(...)` /
  `@register_controller(...)`.
- **`src/fret/resources.py`** (`FR-SYS-10`) — `resolve(kind, rel)`: finds
  `<package share>/<kind>/<rel>` through `ament_index_python` when
  available, falling back to the source tree for pure-Python runs; never
  resolves relative to `Path.cwd()`. `sitl_config.resolve_package_file` /
  `package_share_root` / `package_source_root` are thin wrappers over this
  module.
- **`control/kinematics.py::Kinematics(model)`** stays a thin facade over
  the registry so call sites and tests keep compiling (`FR-SYS-08`).

## Status as of v1.5.0

| Call site | Migrated to the registry? |
|---|---|
| `control/kinematics.py::Kinematics` | Yes |
| `config_loader.py::planning_config_for_model` | Yes |
| `sitl_config.py::controller_config_relative` | Yes |
| `control/controller_node.py::make_controller_node` | Yes (also fixed a latent OMY dof/config-file bug this uncovered) |
| `control/pick_place_planning.py` (`robot_model == "omy"`) | **Not yet** — tracked follow-up |
| `control/pick_place_vision.py` (`RobotId` literal) | **Not yet** — tracked follow-up |
| `control/pick_place_clutter_sim.py` (`_OMY_MODELS`) | **Not yet** — tracked follow-up |
| `ros/mujoco_bridge.py` | **Not yet** — tracked follow-up |
| `launch/sitl.py`, `launch/mujoco.py` model dispatch | **Not yet** — tracked follow-up |
| `RobotIO` / MuJoCo implementation / FSM wiring | Protocol defined; FSM not yet wired through it |

`tests/robots/test_no_model_literals.py` enforces this table: it AST-scans
every file under `src/fret` outside `fret.robots` for a model-literal
comparison, with the "not yet" rows above as named, explicit exceptions
(not a blanket allowance — a second test fails if an exception stops
containing a real violation). See [decisions.md](decisions.md) for why
this migration stopped here in `v1.5.0` and what completes it.

## Acceptance criteria

See `FR-SYS-06` through `FR-SYS-15` in [requirements.md](requirements.md)
for the full, authoritative list; this doc does not duplicate them.

## Edge cases

- A profile declares an alias that collides with another model's key or
  alias: loading the registry fails, naming both files.
- A scenario sets a `model` with different case or surrounding spaces:
  rejected (keys are matched exactly); the error lists valid keys.
- `kind: mobile` with `grasp_modes` declared: rejected at load (typed
  error, no silent default).
- A variant module registers twice under the same key: registration
  raises.
- Two registries built in one process from the same files agree and do
  not share mutable state (`RobotRegistry()` always re-reads the profile
  directory).

## Adding a model

See [docs/robots/README.md § Adding a model](robots/README.md#adding-a-model-since-v150).
