# Contributing to FRET

This document is FRET's own project context: ecosystem, quality gates,
ROS 2 contribution steps, and merge policy. Generic method (Spec-Driven
Development, the V-cycle, TDD), writing, and naming guidelines live in
[.guidelines/](.guidelines/), a shared submodule, not here.

Do not duplicate rules from the submodule in other files. `AGENTS.md`,
`CLAUDE.md`, and `.github/copilot-instructions.md` exist only as entry
points that point to `.guidelines/` and this file.

For **coding conventions** (naming, formatting, typing, ROS 2 layout), see
[docs/guidelines.md](docs/guidelines.md) for what is specific to FRET, and
[.guidelines/languages/py.md](.guidelines/languages/py.md) /
[.guidelines/languages/cpp.md](.guidelines/languages/cpp.md) for the shared
baseline. This file covers *how work is planned, verified, and merged*.

## Table of contents

1. [Ecosystem context](#ecosystem-context)
2. [Development setup](#development-setup)
3. [Method](#method)
4. [Requirement traceability](#requirement-traceability)
5. [Quality gates](#quality-gates)
6. [Definition of Ready and Definition of Done](#definition-of-ready-and-definition-of-done)
7. [Pull request workflow](#pull-request-workflow)
8. [ROS 2 contributions](#ros-2-contributions)
9. [Rules for AI agents](#rules-for-ai-agents)
10. [Reference documents](#reference-documents)
11. [Pre-merge checklist](#pre-merge-checklist)

---

## Ecosystem context

FRET is one project in a family of repositories that share the same SDD +
V-cycle methodology, refined for AI-assisted development. Understanding that
context helps contributors apply the method consistently.

| Project | Role | SDD maturity |
| --- | --- | --- |
| **[Personal website](https://alexandrelheinen.pages.dev)** | Publishes methodology articles and project portfolio entries | Explains *why* SDD works with agents ([SDD and agentic AI for production-quality code](https://alexandrelheinen.pages.dev/articles/2026-04-22-ai-agents-sdd/)) |
| **[Luthier](https://github.com/alexandrelheinen/luthier)** | Python photogrammetry library | Reference implementation: `CONTRIBUTING.md` as constitution, `AC-*` traceability, governance CI |
| **[Freshy](https://freshy-25e.pages.dev/explore)** | Mobile-first cooling-map app (Next.js, Cloudflare) | Product-level SDD in a private repo; stack documented on the website portfolio |
| **[ARCO](https://github.com/alexandrelheinen/arco)** | Motion planning and control algorithms | First proving ground for constrained agentic development |
| **FRET (this repo)** | ROS 2 end-to-end planning + control stack (Menagerie robots) | 4-level V-cycle mapped to `docs/` specifications and pytest/launch_testing validation |

### What FRET already provides

FRET is a ROS 2 workspace with **pure-Python algorithm layers** (kinematics,
planning, scene, validation) and a **thin ROS 2 layer** (nodes, topics, actions).
The specification stack is in place:

| Level | Specification artifacts | Validation artifacts |
| --- | --- | --- |
| 1 — Functional | [docs/requirements.md](docs/requirements.md) (`FR-*`), [docs/scenarios.md](docs/scenarios.md) (`SC-*`) | Scenario pass criteria, integration tests |
| 2 — Architecture | [README.md § Architecture](../README.md#architecture), [docs/interfaces.md](docs/interfaces.md) | `tests/integration/`, `.github/workflows/tests.yml` (integration job) |
| 3 — Module API | Typed stubs under `src/fret/` | `tests/` mirroring `src/fret/` |
| 4 — Implementation | Filled algorithms and ROS nodes | Full CI suite, SITL smoke tests |

Project status and roadmap: [docs/releases.md](docs/releases.md),
[docs/roadmap.md](docs/roadmap.md). User-facing usage: [README.md](README.md),
[docs/mujoco.md](docs/mujoco.md), [docs/simulation.md](docs/simulation.md).

---

## Development setup

**Requirements:** Ubuntu 24.04, ROS 2 Jazzy Desktop, Python 3.12+, Git.

WSL2 on Windows is supported for development. MuJoCo display and rendering on
WSL2 are covered in [docs/wsl.md](docs/wsl.md).

```bash
git clone https://github.com/alexandrelheinen/fret.git
cd fret
./scripts/install.sh -y
./scripts/setup.sh -y
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Pure-Python work** (no ROS runtime) is supported for algorithm modules:

```bash
pip install -e ".[dev]"
pytest tests/ --ignore=tests/integration -v
```

Run the same checks CI runs before opening or updating a pull request:

```bash
bash scripts/check/pre_push.sh          # full gate (requires ROS)
bash scripts/check/pre_push.sh --skip-ros   # formatting, types, unit tests only
```

---

## Method

Spec-Driven Development, the V-cycle, and TDD are defined once in
[.guidelines/workflow/sdd.md](.guidelines/workflow/sdd.md),
[.guidelines/workflow/integration.md](.guidelines/workflow/integration.md),
and [.guidelines/workflow/tdd.md](.guidelines/workflow/tdd.md). Follow
those. FRET's own additions:

- The project-level specification lives in `docs/` (requirements,
  scenarios, architecture, interfaces); a feature-level spec is a GitHub
  issue or PR description with acceptance criteria referencing existing
  `FR-*`/`SC-*` identifiers.
- Default rigor level: spec-first minimum, spec-anchored when module
  boundaries, interfaces, or requirements change.
- The V-cycle's four levels map to concrete FRET locations:

  | Level | Artifact | FRET location | Verification |
  | --- | --- | --- | --- |
  | 1 — Functional | Numbered functional requirements, operational envelope, failure policies | `docs/requirements.md` (`FR-*`) | Named scenarios with quantitative pass criteria, `docs/scenarios.md` (`SC-*`) |
  | 2 — Architecture | Layer decomposition, data flows, typed contracts, QoS, FSMs | `README.md § Architecture`, `docs/interfaces.md` | Inter-node contract tests, `tests/integration/` |
  | 3 — Module API | Public APIs: typed signatures, docstrings, `NotImplementedError` bodies | `src/fret/<module>/` | Unit tests written with the stubs, in `tests/` mirroring `src/fret/` |
  | 4 — Implementation | Algorithms, data structures, ROS node wiring | `src/fret/` | Full test suite, coverage >= 90%, smoke launches |

  Rule: no work at a level starts until that level's artifact exists (no
  Level 2 work on new behavior until the `FR-*` requirement and matching
  `SC-*` scenario exist; no Level 3 work on a new module boundary until
  `docs/interfaces.md` defines the typed interface).
- Stub marking convention: `@pytest.mark.xfail(strict=True,
  raises=NotImplementedError)` at Level 3, removed once Level 4 fills the
  stub.
- Agent instruction files (`AGENTS.md`, `CLAUDE.md`,
  `.github/copilot-instructions.md`) must only point to `.guidelines/` and
  this document, per [.guidelines/agents/context.md](.guidelines/agents/context.md).

---

## Requirement traceability

Requirements must be traceable in both directions: from an acceptance criterion
down to the test that proves it, and from any test back to the criterion.

| Identifier | Document | Example |
| --- | --- | --- |
| `FR-<LAYER>-<NN>` | [docs/requirements.md](docs/requirements.md) | `FR-CTL-02` |
| `SC-<NN>` | [docs/scenarios.md](docs/scenarios.md) | `SC-01` |
| Release | [docs/releases.md](docs/releases.md) | v1.x / v2.x / v3.0 |

**Rules:**

- Reference `FR-*` / `SC-*` in issue text, PR descriptions, or test docstrings
  when asserting observable behavior.
- Every new functional requirement needs a scenario or test; a requirement with
  no verification is an incomplete spec.
- Do not silently skip tests (`skip`, `xfail`) without a `reason=` naming the
  blocking requirement or tracking issue.

---

## Quality gates

### Pre-push by blast radius

Agents and humans must size the **local** pre-push gate to the files they
changed. Formatting + types alone is only enough for pure docs/comments.
Hand-picking “representative” pytest files is not a gate — run the **same**
CI shard script(s) that cover the blast radius.

| Change class | Required before `git push` |
| --- | --- |
| Docs / comments only (`*.md`, `docs/`, …) | `formatting.sh` (types optional) |
| Pure typing / format-only | `formatting.sh` + `types.sh` |
| `src/fret/control`, `mjcf`, `vision`, `hardware`, scenario/vision YAML, or `tests/control\|vision\|hardware` | format + types + **`unit_shard.sh control`** |
| `src/fret/planning` or `tests/planning` | format + types + **`unit_shard.sh planning`** |
| `src/fret/scene` or top-level `tests/test_*.py` | format + types + **`unit_shard.sh scene`** |
| `simulation` / `scenario` / `ros` / release render scripts / `tests/{simulation,scenario,ros,release}` | format + types + **`unit_shard.sh simulation`** |
| Cross-cutting / unsure (`pyproject.toml`, `.github/`, shared `src/fret/*`, …) | format + types + **all touched-risk shards** (or full `pre_push.sh`) |

Helper (maps `git diff` paths → shards and runs them):

```bash
bash scripts/check/pre_push_touched.sh
# optional: bash scripts/check/pre_push_touched.sh --base HEAD~1
```

**Sibling-assert sweep:** when changing a magic number or threshold, grep the
old value across `tests/` and update every twin assert in the same commit.

**Done means CI green:** after push, wait for the affected GitHub Actions jobs.
Do not claim completion while required checks are pending or red.

### Local validation

```bash
# Build and source
./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Path-aware (agents: default before push)
bash scripts/check/pre_push_touched.sh

# Recommended full gate
bash scripts/check/pre_push.sh

# Or step by step:
bash scripts/check/formatting.sh
bash scripts/check/types.sh
bash scripts/tests/unit.sh
bash scripts/tests/unit_shard.sh control  # CI shard: control | planning | scene | simulation
bash scripts/tests/smoke.sh          # requires ROS + xvfb
bash scripts/tests/integration.sh    # requires ROS + xvfb
```

### CI workflows

| Workflow | Trigger | What it checks |
| --- | --- | --- |
| `formatting.yml` | PR | Black, isort, clang-format |
| `type_check.yml` | PR | mypy strict on `src/` |
| `tests.yml` | PR | Parallel jobs: unit shards (4×), coverage gate, smoke, integration |

### Coverage and style

- Python unit test coverage target: **>= 90%** on `src/fret`.
- Formatting, docstrings, Doxygen, and naming follow
  [.guidelines/languages/py.md](.guidelines/languages/py.md),
  [.guidelines/languages/cpp.md](.guidelines/languages/cpp.md), and
  [.guidelines/style/naming.md](.guidelines/style/naming.md).

---

## Definition of Ready and Definition of Done

### Definition of Ready (before coding)

- [ ] Intent, scope, and **acceptance criteria** are written (GitHub issue or PR).
- [ ] Rigor level chosen (spec-first / spec-anchored / spec-as-source).
- [ ] Affected V-cycle levels and test levels identified.
- [ ] Linked `FR-*` / `SC-*` ids when behavior changes.
- [ ] No undocumented new runtime dependency or public API break.

### Definition of Done (before merge)

- [ ] All acceptance criteria verified by tests or documented manual checks.
- [ ] Focused commits; conventional commit messages.
- [ ] All required CI jobs green on the PR branch.
- [ ] `docs/` updated when requirements, architecture, or interfaces changed.
- [ ] No quality gate weakened to pass.
- [ ] Human reviewer merged (agents do not self-merge).

---

## Pull request workflow

1. Branch from `main` with a descriptive name (e.g. `feat/pillar-scenario`,
   `fix/planner-timeout`).
2. Make **focused commits**—one logical step each.
3. Open a PR against `main` with:
   - **Summary** — what changed and why (1–3 bullets).
   - **Test plan** — checklist of automated and manual verification.
   - **Traceability** — `FR-*` / `SC-*` / milestone references when applicable.
4. Ensure CI is green before requesting review.
5. Address feedback in new commits (avoid force-push to `main`).

Commit format follows
[.guidelines/workflow/commits.md](.guidelines/workflow/commits.md).

---

## ROS 2 contributions

When contributing ROS 2 components, also update build metadata:

### Adding a node

1. Python script in `src/fret/ros/` or C++ in `src/fret/src/`
2. `CMakeLists.txt` — build and install executable
3. Launch file in `src/fret/launch/`
4. Document topics, parameters, and purpose in [docs/modules/ros_nodes.md](docs/modules/ros_nodes.md)

### Adding URDF/XACRO

1. XACRO in `src/fret/urdf/<model>.xacro`
2. Optional mesh generator in `src/fret/mesh/<model>.py`
3. `CMakeLists.txt` install rules
4. MuJoCo preview: add MJCF under `src/fret/mjcf/` when applicable
5. Smoke test: `python3 scripts/view_mujoco.py --model dubins --scenario dubins_race --duration 30 --fps 60 --camera overview --dry-run`

### Adding configuration

1. YAML in `src/fret/config/` with commented parameters
2. `CMakeLists.txt` install rule
3. Reference from launch file or node parameters

### Adding dependencies

1. `package.xml` and `CMakeLists.txt`
2. Document in README if non-standard

---

## Rules for AI agents

These rules apply to Cursor agents, Copilot, Claude Code, and any
automated contributor. General agent behavior (evidence, no fabrication,
git safety, communication) follows
[.guidelines/agents/claude.md](.guidelines/agents/claude.md). FRET adds:

### Execution order

1. Confirm the spec / acceptance criteria exist for the V-cycle level
   being touched; update `docs/` when Level 1-2 changes.
2. Stubs + tests together when adding APIs (Level 3).
3. Implement (Level 4); run `scripts/check/pre_push_touched.sh` (or full
   `scripts/check/pre_push.sh`). See
   [Pre-push by blast radius](#pre-push-by-blast-radius).
4. Push only after the blast-radius gates pass locally (not only
   format/types when code/tests changed); wait for CI green on affected
   jobs; do not claim done while checks fail. Include a proportional
   proof report per the
   [`report-writing` skill](.cursor/skills/report-writing/SKILL.md) (also
   [AGENTS.md § Proof reports](AGENTS.md#proof-reports-mandatory)).

---

## Reference documents

When an external guide conflicts with this file, `.guidelines/`, or
[docs/guidelines.md](docs/guidelines.md), **this repository wins**.

| Document | Role |
| --- | --- |
| [.guidelines/](.guidelines/) | Shared method, writing, naming, and per-language style (submodule) |
| [docs/requirements.md](docs/requirements.md) | Level 1 — `FR-*` functional requirements |
| [docs/scenarios.md](docs/scenarios.md) | Level 1 — `SC-*` scenario validation |
| [README.md § Architecture](../README.md#architecture) | Level 2 — system design |
| [docs/interfaces.md](docs/interfaces.md) | Level 2 — typed contracts, QoS, FSMs |
| [docs/mujoco.md](docs/mujoco.md) | Level 2 — MuJoCo simulation integration |
| [docs/simulation.md](docs/simulation.md) | User guide — MuJoCo modes and quick start |
| [docs/releases.md](docs/releases.md) | Release specification (v1.x sim/CV, v2.x hardware, v3.0 north-star) |
| [docs/mujoco_physics_v1.2.md](docs/mujoco_physics_v1.2.md) | v1.2 physics SITL engineering spec |
| [docs/guidelines.md](docs/guidelines.md) | FRET-specific coding notes |
| [docs/dev-environment.md](docs/dev-environment.md) | Environment/debugging notes |

---

## Pre-merge checklist

Every contributor (human or agent) must confirm before merge, beyond the
shared [.guidelines/templates/pr.md](.guidelines/templates/pr.md)
checklist:

- [ ] `FR-*` / `SC-*` traceability updated when behavior changed.
- [ ] `bash scripts/tests/unit.sh` passes (when ROS workspace available).
- [ ] Public APIs typed and documented.

If any item fails, fix forward, do not merge.

---

## License

By contributing, you agree that your contributions will be licensed under the
MIT License.
