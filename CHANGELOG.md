# Changelog

All notable changes to FRET are documented in this file. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning
follows [Semantic Versioning](https://semver.org/). Seeded from
[`docs/releases.md` § Version tagging](docs/releases.md#version-tagging).

## [Unreleased]

## [2.2.0] - 2026-11-17

### Added

- Safety layer (`SafetySupervisor`): command watchdog, limit/step clamp,
  enable lease, latched category-2 `~/stop`, soft start, protection
  register write/readback, hold-on-fault, bus power-loss → `FAULT`
  (FR-HW-40..47, FR-HW-69, FR-HW-70). Validated on mock bus + MuJoCo twin.
- Mock `POWER_CUT` fault; bridge services `~/enable`/`~/disable`/
  `~/clear`/`~/stop`; `/controller_fault` handling.
- `docs/hardware/safety.md` (case a e-stop primary; case b fallback).

### Removed

- FR-SYS-18 mock-only torque interlock (replaced by FR-SYS-19: active
  supervisor required for real-port enable).

### Known gaps

- Not validated on real hardware (T4).

## [2.1.0] - 2026-11-17

### Added

- Hardware-free Feetech STS3215 test infra: protocol module, `MockStsBus`
  with fault injection, pty fake device, record/replay JSONL (FR-HW-68,
  FR-HW-71).
- SO-101 `BridgeNode` / `BridgeCore` developed TDD against the mock
  (FR-HW-20..25, FR-HW-45 integrity); identity `SafetySupervisor` seam
  for PR4; mock-only torque interlock (FR-SYS-18).
- `fret-hw` servo tooling CLI; `launch/hardware.py robot:=so101`;
  hardware `RobotIO`; example calibration YAML (FR-HW-26, FR-HW-27,
  FR-SYS-17).

### Known gaps

- Mock only: not validated on real hardware. Torque enable on real
  serial ports is refused until PR4.

## [2.0.0] - 2026-11-17

### Added

- SO-101 robot variant: Menagerie `robotstudio_so101` vendored under
  `third_party/`, pure-NumPy FK/IK (`kinematics_so101.py`), registry
  profile + configs, `VelocityToPositionIntegrator` (FR-CTL-07), and
  `so101_reach` empty-cell scenario (FR-SIM-15, FR-SIM-16, FR-SIM-17,
  FR-SIM-20).
- Friction-mode pick-and-place (`so101_pick_place`, SC-v20b): no MuJoCo
  adhesion actuator; contact-gated equality weld after pad contact;
  FSM slip → FAULT (FR-SIM-18 interim fidelity — see `docs/decisions.md`).
- Overhead CV pick-and-place (`so101_pick_place_cv`, SC-v20c) with
  level-pad IK and 9/10 seed acceptance (FR-SIM-19).
- PlotJuggler layout `so101_arm` and hardware study
  (`docs/hardware/arm-selection.md`, `docs/robots/so101.md`).
- Third-party license adjacency gate for vendored Menagerie assets
  (FR-SYS-16).

### Changed

- `make_mujoco_bridge_core` routes arm models through the robot registry
  (fixes the pre-v2.0.0 "Unknown MuJoCo bridge model" path for OMX/OMY/
  SO-101).
- Requirements / roadmap / releases docs rewritten for the v2.x hardware
  line; SO-101 is the decided target.
- `types.sh` green again: retargeted rclpy mypy ignores after stub drift
  (31 unused-ignore / attr-defined errors cleared on `main`-class files).

### Known gaps

- Contact-gated weld is an interim stand-in for pure friction lift with
  Menagerie jaw geometry; soft-weld stretch under MPC can exceed the
  original 5 mm wording (airborne hold + ≤100 mm stretch bounded in
  tests).
- Combined unit coverage still &lt;90% on `main` (pre-existing); gate not
  lowered.
- Remaining PR1 registry migrations (`pick_place_*` / launch EqualsSubstitution)
  still deferred — see `docs/decisions.md`.

## [1.5.0] - 2026-11-17

### Added

- Robot model registry (`fret.robots`): `RobotProfile` schema and loader,
  `RobotRegistry` (profile lookup by canonical key/alias, typed
  kinematics/controller/transfer-planner/sim-IO variant tables),
  structural protocols (`KinematicsBackend`, `ControllerFactory`,
  `TransferPlannerStrategy`, `SimIOAdapter`, `RobotIO`), and built-in
  variants for `dubins`, `open_manipulator_x`, `open_manipulator_y`
  (FR-SYS-06, FR-SYS-07, FR-SYS-08 partial, FR-SYS-09).
- `config/robots/{dubins,open_manipulator_x,open_manipulator_y}.yml`
  robot profiles.
- `fret.resources.resolve(kind, rel)`: package-share resource resolution
  independent of the current working directory (FR-SYS-10).
- `docs/robot-model-agnostic.md` (durable spec), `docs/decisions.md`,
  `docs/hardware/README.md`, `docs/hardware/bridge.md` (moved from
  `docs/modules/hardware.md`).
- `tests/robots/`, `tests/test_resource_resolution.py`,
  `tests/test_version_consistency.py`, `tests/test_docs_hardware_neutral.py`,
  `tests/control/test_controller_node.py`.
- A real-rclpy regression test for the `planner_node` `BYTE_ARRAY`
  parameter-inference crash recorded in `docs/dev-environment.md`
  (FR-SYS-15).

### Changed

- `control/kinematics.py::Kinematics`, `config_loader.py::planning_config_for_model`,
  `sitl_config.py::controller_config_relative`, and
  `control/controller_node.py::make_controller_node` resolve per-model
  behavior through the robot registry instead of hardcoded string-literal
  dispatch (FR-SYS-08).
- `ControllerNode` takes an explicit `dof` parameter and resolves its
  per-model controller YAML file name through the registry, fixing a
  latent bug where constructing it for OMY silently used OM-X's DOF (4)
  and controller config file.
- Documentation outside `docs/robots/` and `docs/hardware/` no longer
  assumes a specific hardware target; `FR-HW-01`..`FR-HW-03` are marked
  "superseded in v2.0, pending redefinition" without renumbering
  (FR-SYS-13). `docs/releases.md` v1.5.0 section records that the former
  "v1.5 dynamic ball + industrial place" scope (`SC-v17`,
  `FR-VIS-07`/`FR-VIS-08`) moves to a simulation backlog.
- `docs/modules/ros_nodes.md` rewritten from the real `src/fret/ros/`
  contents and `pyproject.toml` `[project.scripts]`.
- `scripts/tests/unit.sh` / `unit_shard.sh` pin
  `OMP_NUM_THREADS=MKL_NUM_THREADS=OPENBLAS_NUM_THREADS=1` for
  deterministic floating-point reduction order, fixing a wall-maze test
  flake (see `docs/decisions.md`).

### Fixed

- `src/fret/package.xml` `<version>` (`1.4.1`) now matches
  `pyproject.toml` (`1.4.4`); a unit test fails if they ever diverge again
  (FR-SYS-14).
- `planner_node_ros.py` mypy-strict errors (6) fixed with targeted,
  per-line `type: ignore[<code>]` comments naming the exact rclpy
  typing gap, no blanket ignore.
- `launch/hardware.py` previously referenced a `bridge_node` ROS
  executable that is not registered anywhere; it now fails fast with a
  clear "not available before v2.x" error instead of an opaque "executable
  not found".

### Removed

- The Gazebo simulator backend (`launch/sim.py`, the `is_gazebo` launch
  condition, and the related `package.xml` / CMakeLists.txt / install
  dependencies). It could not run: no `urdf/`, `worlds/`, or `mesh/`
  directory exists in this repository. MuJoCo remains the sole simulator
  (FR-SIM-01). The `backend` launch argument is kept for backward
  compatibility (default `mujoco`, accepted but ignored).

## [1.4.4] - 2026-??-??

ARCO v0.5.0 compiled core (Rust planners, maps, controllers);
`omy_pick_place` out of the release render matrix.

## [1.4.3]

OMY APPROACH_PICK plans around colliding place-bin shell.

## [1.4.2]

ARCO v0.3.7 MPCC + full-arm obstacle contacts.

## [1.4.1]

Showcase honesty (MPC barriers, OM-X grasp, Ø40 mm ball).

## [1.4.0]

CV ↔ manipulation integration + MuJoCo cameras.

## [1.3.0]

Computer vision pipeline + algorithm selection.

## [1.2.5] - [1.2.7]

Patches: OMY clutter, telemetry R2, Dubins encode.

## [1.2.4]

OMY 6-DOF challenge.

## [1.2.3]

OpenMANIPULATOR-X tabletop.

## [1.2.0]

MuJoCo physics SITL (Dubins).

## [1.1.0]

Dubins dual race — first product showcase.
