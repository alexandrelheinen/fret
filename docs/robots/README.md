# FRET Robot Models

Each FRET release targets one robot class. Models are selectable via `model:=` at launch.

| Model | Release | Doc | Status |
|---|---|---|---|
| `dubins` | v1.1 | [dubins.md](dubins.md) | ✅ Shipped |
| `rrp` / `scara` | v1.3 | [rrp.md](rrp.md) | 🟡 Bootstrap exists |
| `six_dof` | v1.4 | [six_dof.md](six_dof.md) | 🔲 Planned |

The bootstrap `scara` model (RRP, 3-DOF) validated MS-1–5 and remains in the codebase
as the foundation for v1.3.

All robots run on **MuJoCo** for physics, rendering, and SITL. See [mujoco.md](../mujoco.md).

Unit physics sandboxes (no race): `turtlebot3_unit` via
`fret.simulation.TurtleBot3UnitRobot` (FR-SIM-11); procedural `diffdrive_unit`
remains as a lightweight twin. External MJCF sources and reuse policy:
[mujoco_models_benchmark.md](../mujoco_models_benchmark.md).
