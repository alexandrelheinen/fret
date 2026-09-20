# AGENTS.md

This file provides guidance to AI coding agents (Antigravity, Gemini, Claude, Cursor, and other agents) when working with code in this repository.

@.guidelines/workflow/sdd.md
@.guidelines/workflow/integration.md
@.guidelines/workflow/tdd.md
@.guidelines/agents/writing.md
@.guidelines/style/naming.md
@.guidelines/languages/py.md
@.guidelines/languages/cpp.md
@.guidelines/languages/cmake.md
@.guidelines/languages/sh.md

For FRET's own project context, ecosystem, quality gates, and merge
policy, read [CONTRIBUTING.md](CONTRIBUTING.md). For FRET-specific coding
notes, read [docs/guidelines.md](docs/guidelines.md). For durable
environment/debugging notes (ROS sourcing, pytest plugin conflicts,
headless MuJoCo, known pre-existing breakages), read
[docs/dev-environment.md](docs/dev-environment.md).

Conflict order: direct maintainer request > CONTRIBUTING.md > `.guidelines/`
> `docs/guidelines.md` > ROS 2 official documentation.

## Quick reference: Common tasks

| Task | Command |
|---|---|
| Format Python & C++ | `bash scripts/check/formatting.sh` |
| Type check (mypy) | `bash scripts/check/types.sh` |
| Run touched pre-push gate | `bash scripts/check/pre_push_touched.sh` |
| Full pre-push quality gate | `bash scripts/check/pre_push.sh` |
| Run unit tests | `bash scripts/tests/unit.sh` |
| Run integration tests | `bash scripts/tests/integration.sh` |

## Architecture at a glance

FRET is a full-stack robotics framework bridging ARCO planning and control to MuJoCo physics simulation and ROS 2.

1. **Planning & Control (`src/fret/planning/`, `src/fret/control/`)**:
   - ARCO integration (RRT*, SST, kinematic & physics Dubins tracking, joint MPC).
   - Manipulator FSM and controllers (`open_manipulator_x`, `omy`).
2. **Simulation (`src/fret/simulation/`, `src/fret/mjcf/`)**:
   - MuJoCo physics and camera simulation, actuators, contacts, and telemetry.
3. **Vision (`src/fret/vision/`)**:
   - OpenCV-based pipeline (HSV blob tracking, table-plane projection, camera models).
4. **ROS 2 Bridges (`src/fret/ros/`)**:
   - Bridges for MuJoCo simulation, perception, and scenario execution under ROS 2 Jazzy.

## Pre-push gates (mandatory)

Do **not** treat `formatting.sh` + `types.sh` as enough whenever code or
tests change. Size the local gate to the blast radius, see
[CONTRIBUTING.md § Pre-push by blast radius](CONTRIBUTING.md#pre-push-by-blast-radius).
Run the helper (preferred) or the equivalent shard commands it prints:

```bash
bash scripts/check/pre_push_touched.sh
```

Minimum always:

```bash
bash scripts/check/formatting.sh
bash scripts/check/types.sh
```

When ROS is available and the change is broad, prefer the full gate:

```bash
bash scripts/check/pre_push.sh
```

Pushing with formatting, type-check, or required-shard failures is
unacceptable. After push, wait until CI is green for the affected jobs;
local subset pass does not mean done.

## Proof reports (mandatory)

Agents must prove that work works, not only implement and push. Report
depth is proportional to change size and problem difficulty. Load the
repo skill
[`.agents/skills/report-writing/SKILL.md`](.agents/skills/report-writing/SKILL.md)
whenever you create or update a PR, finish a cloud-agent task, or the
maintainer asks for evidence. Follow its tier table (T0-T4) and layer
stack. Minimum: cite commands run and pass/fail results.
