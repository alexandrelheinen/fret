# FRET Robot Models

Models are selectable via `model:=` at launch. **Product robots come from git
submodules** (ROBOTIS MuJoCo Menagerie + AWS props).

| Model | Doc | Uses CV? |
|---|---|---|
| `dubins` (TB3 Burger) | [dubins.md](dubins.md) | **No** — ARCO→MuJoCo case study |
| `open_manipulator_x` | [open_manipulator_x.md](open_manipulator_x.md) | **Yes** |
| `six_dof` / `omy` (OpenMANIPULATOR-Y) | [six_dof.md](six_dof.md) | **Yes** |

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
