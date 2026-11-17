# FRET Robot Models

Models are selectable via `model:=` at launch. **Product robots come from git
submodules** (ROBOTIS MuJoCo Menagerie + AWS props).

| Model | Doc | Uses CV? |
|---|---|---|
| `dubins` (TB3 Burger) | [dubins.md](dubins.md) | **No** — ARCO→MuJoCo case study |
| `open_manipulator_x` | [open_manipulator_x.md](open_manipulator_x.md) | **Yes** |
| `six_dof` / `omy` (OpenMANIPULATOR-Y) | [six_dof.md](six_dof.md) | **Yes** |
| `so101` (Seeed SO-ARM101 Pro) | [so101.md](so101.md) | Deferred (PR2 ships the twin; `uses_vision: false` until `so101_pick_place_cv`) |

All robots run on **MuJoCo** for physics, rendering, and SITL. See
[mujoco.md](../mujoco.md).

**Roles:**

- **TB3 / Dubins** proves SE(2) planning and path-following under MuJoCo physics.
  It does not grasp objects and does not consume the vision pipeline.
- **OM-X / OMY** perform tabletop manipulation; ball pose comes from
  `fret.vision` ([vision/README.md](../vision/README.md)).

Unit physics sandboxes (no race): `turtlebot3_unit` via
`fret.simulation.TurtleBot3UnitRobot` (FR-SIM-11); procedural `diffdrive_unit`
remains as a lightweight twin. External MJCF sources: ROBOTIS MuJoCo Menagerie
and AWS RoboMaker warehouse (git submodules).

## Adding a model (since v1.5.0)

Full design: [robot-model-agnostic.md](../robot-model-agnostic.md). Per-model
behavior is resolved through `fret.robots.registry.RobotRegistry`
(FR-SYS-06, FR-SYS-08), not through string-literal dispatch in `control/`,
`planning/`, `ros/`, `scene/`, `vision/` or `launch/` (FR-SYS-09). Adding a
model means:

1. **One profile YAML** under `src/fret/config/robots/<model>.yml`, validated
   against the schema in `fret.robots.profile` (FR-SYS-07): `name`, `kind`
   (`mobile` or `arm`), `joint_names`, `joint_limits`, `kinematics`,
   `controller_config`, `planning_config`, plus any `aliases`, `mjcf`,
   `vision_config`, `grasp_modes`, `transfer_planner`, `uses_vision`.
2. **One variant module** under `src/fret/robots/variants/<model>.py`,
   registering factories under the profile's `kinematics` key with
   `@register_kinematics(...)` and, for arm models with a ROS controller,
   `@register_controller(...)`.
3. Import the new variant module from
   `src/fret/robots/variants/__init__.py`.

`tests/robots/test_toy_model.py` proves this end to end with a test-only
toy 2-DOF arm, through the real `Kinematics` facade and
`make_controller_node`, with zero edits outside `fret.robots` and the test
itself.

As of `v1.5.0`, `TransferPlannerStrategy`, `SimIOAdapter` and `RobotIO`
are defined protocols (`fret.robots.protocols`) that not every call site
consumes yet; `docs/decisions.md` tracks the remaining migration.
