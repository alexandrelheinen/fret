# FRET Robot Models

Each FRET release targets one robot class. Models are selectable via `model:=` at launch.
**Product robots come from git submodules** (ROBOTIS MuJoCo Menagerie + AWS props).

| Model | Release | Doc | Status |
|---|---|---|---|
| `dubins` (TB3 Burger) | v1.1–v1.2 | [dubins.md](dubins.md) | ✅ Shipped |
| `open_manipulator_x` | v1.3 | [open_manipulator_x.md](open_manipulator_x.md) | 🔲 In progress |
| `six_dof` (OMY preferred) | v1.4 | [six_dof.md](six_dof.md) | 🔲 Planned |

All robots run on **MuJoCo** for physics, rendering, and SITL. See [mujoco.md](../mujoco.md).

Unit physics sandboxes (no race): `turtlebot3_unit` via
`fret.simulation.TurtleBot3UnitRobot` (FR-SIM-11); procedural `diffdrive_unit`
remains as a lightweight twin. External MJCF sources and reuse policy:
[mujoco_models_benchmark.md](../mujoco_models_benchmark.md).
