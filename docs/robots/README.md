# FRET Robot Models

Each FRET release targets one robot class. Models are selectable via `model:=` at launch.
**Product robots come from git submodules** (ROBOTIS MuJoCo Menagerie + AWS props).

| Model | Release | Doc | Status | Uses CV? |
|---|---|---|---|---|
| `dubins` (TB3 Burger) | v1.1–v1.2 | [dubins.md](dubins.md) | ✅ Shipped | **No** — ARCO→MuJoCo case study |
| `open_manipulator_x` | v1.2.3 (+ CV from v1.4) | [open_manipulator_x.md](open_manipulator_x.md) | ✅ Shipped | **Yes** from v1.4 |
| `six_dof` / `omy` (OpenMANIPULATOR-Y) | v1.2.4 (+ CV from v1.4) | [six_dof.md](six_dof.md) | ✅ Shipped | **Yes** from v1.4 |

All robots run on **MuJoCo** for physics, rendering, and SITL. See [mujoco.md](../mujoco.md).

**Roles:**

- **TB3 / Dubins** proves SE(2) planning and path-following under MuJoCo physics.
  It does not grasp objects and does not consume the vision pipeline.
- **OM-X / OMY** perform tabletop manipulation; from **v1.4** ball pose comes from
  `fret.vision` ([vision/README.md](../vision/README.md)).

Unit physics sandboxes (no race): `turtlebot3_unit` via
`fret.simulation.TurtleBot3UnitRobot` (FR-SIM-11); procedural `diffdrive_unit`
remains as a lightweight twin. External MJCF sources and reuse policy:
[mujoco_models_benchmark.md](../mujoco_models_benchmark.md).
