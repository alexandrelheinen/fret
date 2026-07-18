# Contributing to FRET

This document is the **single source of truth** for how humans and AI agents
contribute to FRET (Full-stack Robotic Effector Trajectories). It defines the
development constitution: Specification-Driven Development (SDD), the V-cycle
software development lifecycle, quality gates, and agent policy.

Do not duplicate these rules in other files. `AGENTS.md`, `CLAUDE.md`, and
`.github/copilot-instructions.md` exist only as entry points that point here.

For **coding conventions** (naming, formatting, typing, ROS 2 layout), see
[docs/guidelines.md](docs/guidelines.md). That file covers *how* code is
written; this file covers *how work is planned, specified, verified, and merged*.

## Table of contents

1. [Ecosystem context](#ecosystem-context)
2. [Development setup](#development-setup)
3. [Spec-driven development (SDD)](#spec-driven-development-sdd)
4. [FRET development stages (4-level V-cycle)](#fret-development-stages-4-level-v-cycle)
5. [Combining SDD, V-cycle, and TDD](#combining-sdd-v-cycle-and-tdd)
6. [Constraint layers for AI-assisted work](#constraint-layers-for-ai-assisted-work)
7. [Requirement traceability](#requirement-traceability)
8. [Quality gates](#quality-gates)
9. [Definition of Ready and Definition of Done](#definition-of-ready-and-definition-of-done)
10. [Pull request workflow](#pull-request-workflow)
11. [ROS 2 contributions](#ros-2-contributions)
12. [Rules for AI agents](#rules-for-ai-agents)
13. [Reference documents](#reference-documents)
14. [Pre-merge checklist](#pre-merge-checklist)

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

## Spec-driven development (SDD)

**Spec-driven development** treats written specifications—not code—as the
primary artifact. Code, tests, and design notes are derived from and validated
against those specs.

In FRET, the **project-level specification** lives in `docs/` (requirements,
scenarios, architecture, interfaces). Each **feature-level specification** is a
GitHub issue or PR description with concrete acceptance criteria.

### What a spec must contain

Before implementation starts, the spec for a change must be clear enough that an
independent reviewer (human or agent) could verify completion without guessing
intent:

| Element | Purpose | Where it lives |
| --- | --- | --- |
| **Intent** | Why the change exists | Issue title/body or PR summary |
| **Scope** | What is in and out of bounds | Issue or PR description |
| **Acceptance criteria** | Observable conditions of done | Issue checklist or PR test plan |
| **Traceability** | Link to `FR-*` / `SC-*` when applicable | Issue, PR, or test docstrings |
| **Constraints** | Non-negotiable rules (types, style, ROS layout) | This file + [docs/guidelines.md](docs/guidelines.md) |
| **Design notes** | Interfaces, modules, edge cases (when non-trivial) | PR description or `docs/` |

Write acceptance criteria in concrete, testable language (EARS-style: “When …,
the system shall …”). Ambiguous specs produce ambiguous code—refine the spec
before coding.

### SDD workflow in this repo

1. **Specify** — Capture intent, scope, and acceptance criteria in a GitHub
   issue. Map criteria to existing `FR-*` / `SC-*` identifiers when the change
   touches functional behavior.
2. **Plan** — For non-trivial work, identify which V-cycle levels are affected,
   which `docs/` files may need updates, and which test levels apply (unit /
   integration / scenario / SITL).
3. **Task** — Break the plan into focused commits. Each commit addresses one
   logical step and traces back to at least one acceptance criterion.
4. **Implement and verify** — Follow the V-cycle at each level: stubs and tests
   before implementation where applicable; run quality gates locally before push.
5. **Review** — PR review checks that code matches the spec and that tests prove
   the acceptance criteria. **Humans merge; agents do not.**

### Rigor levels

| Level | When to use | Spec artifact |
| --- | --- | --- |
| **Spec-first** | Any merged change | Issue or PR with acceptance criteria |
| **Spec-anchored** | Public API, architecture, or `docs/` changes | Above + design notes and test plan |
| **Spec-as-source** | Large or AI-assisted features | Above + explicit task breakdown before coding |

Default for FRET: **spec-first** minimum; **spec-anchored** when module
boundaries, interfaces, or requirements change.

### Rules

- Do not implement without a written spec (issue scope or PR description with
  acceptance criteria).
- When requirements change mid-task, update the spec first, then code and tests.
- Regressions: extend the spec with a failing test that encodes the bug, then fix.
- Do not add duplicate workflow documentation outside this file and
  [docs/guidelines.md](docs/guidelines.md).

Further reading:
[GitHub Spec Kit — Spec-Driven Development](https://github.com/github/spec-kit/blob/main/spec-driven.md),
[SDD and agentic AI for production-quality code](https://alexandrelheinen.pages.dev/articles/2026-04-22-ai-agents-sdd/)
(author's article on constraint layers and the SDD V-cycle).

---

## FRET development stages (4-level V-cycle)

FRET uses a **4-level V-cycle** as its software development lifecycle. Each
level has a descending artifact (specification or design) and an ascending
validation method. **Both sides of a level must be addressed before moving to the
next.**

An imperative order (implement, add, fix…) always implies the **full V-cycle**—not
just the code.

```
Level 1 — Functional specification  ◄──────────────►  Scenario / acceptance validation
  Level 2 — Architecture & interfaces  ◄──────────►  Integration validation
    Level 3 — Module stubs & public API  ◄────────►  Unit tests
      Level 4 — Implementation & private code  ◄──►  Full CI + SITL smoke tests
```

For agile delivery, treat each GitHub issue as one complete V (the **W-cycle**:
chained small V-cycles). The next task starts from the verified output of the
previous one.

### Level 1 — Functional specifications

| Side | Artifact | FRET location |
| --- | --- | --- |
| **Specify** | Numbered functional requirements, operational envelope, failure policies | [docs/requirements.md](docs/requirements.md) (`FR-*`) |
| **Verify** | Named scenarios with quantitative pass criteria | [docs/scenarios.md](docs/scenarios.md) (`SC-*`) |

**Rule:** No Level 2 work on new behavior until the relevant `FR-*` requirement
exists and has a matching `SC-*` scenario or test.

**CI gate:** Integration and scenario tests in `tests/integration/` and
`tests/simulation/`.

### Level 2 — Architecture and pipeline

| Side | Artifact | FRET location |
| --- | --- | --- |
| **Specify** | Layer decomposition, data flows, typed contracts, QoS, FSMs | [README.md § Architecture](../README.md#architecture), [docs/interfaces.md](docs/interfaces.md) |
| **Verify** | Inter-node contract tests | `tests/integration/`, `.github/workflows/tests.yml` (integration job) |

**Rule:** No Level 3 work on a new module boundary until
[docs/interfaces.md](docs/interfaces.md) defines the typed interface.

### Level 3 — Module stubs and unit tests

| Side | Artifact | FRET location |
| --- | --- | --- |
| **Specify** | Public APIs: typed signatures, docstrings, `NotImplementedError` bodies | `src/fret/<module>/` |
| **Verify** | Unit tests written **with** the stubs | `tests/` (mirrors `src/fret/`) |

Use `@pytest.mark.xfail(strict=True, raises=NotImplementedError)` for methods not
yet implemented. Remove `xfail` when Level 4 fills the stub.

**CI gates:** `.github/workflows/tests.yml`, `.github/workflows/type_check.yml`
(mypy strict on `src/`).

**Rule:** Stub + test files for a module land in the same commit. No Level 4
until stubs pass mypy and xfail tests behave as expected.

### Level 4 — Implementation

| Side | Artifact | FRET location |
| --- | --- | --- |
| **Specify** | Algorithms, data structures, ROS node wiring | `src/fret/` |
| **Verify** | Full test suite, coverage ≥ 90%, smoke launches | All CI workflows |

**Rule:** A feature is not done until CI is green and the relevant scenario
passes (pytest and/or SITL as appropriate).

### SDD mapping to the V-cycle (GitHub workflow)

The [SDD V-cycle article](https://alexandrelheinen.pages.dev/articles/2026-04-22-ai-agents-sdd/)
maps naturally to GitHub:

| V side | GitHub artifact | Agent / human action |
| --- | --- | --- |
| Left — Specify | **Issue** | Write what must be done, edge cases, tests that must pass (DoD) |
| Left — Design | **Issue + `docs/`** | Architecture notes, interface updates when needed |
| Right — Verify | **PR + CI** | Tests, formatting, types, integration; PR not mergeable until green |
| Right — Accept | **Human merge** | Review verifiable artifacts (logs, plots, videos for physical behavior) |

At the base of the V, the **agent loop** runs locally before each commit:
format → lint → type-check → unit tests → fix until green. That loop is where
most quality work happens.

---

## Combining SDD, V-cycle, and TDD

SDD, the V-cycle, and test-driven development (TDD) operate at different layers.
Use them together in this order:

```
SDD (what & why)  →  V-cycle (structure each level)  →  TDD (build each unit)
     spec                 design ↔ tests                    red → green → refactor
```

| Step | Method | Activity |
| --- | --- | --- |
| 1 | **SDD** | Write intent, scope, acceptance criteria |
| 2 | **V-cycle** | Map criteria to test levels (scenario → integration → unit) |
| 3 | **V-cycle** | Update architecture / interface docs when boundaries change |
| 4 | **TDD** | At Level 3–4: failing test → minimal code → refactor |
| 5 | **V-cycle** | Run tests bottom-up; CI must pass |
| 6 | **SDD** | Update spec/docs if behavior changed; PR proves all criteria |

**TDD rules** (Kent Beck): do not write production code without a failing test;
eliminate duplication. TDD executes **detailed design** at Level 3–4—it does not
replace scenario or integration specs from Level 1–2.

**Pause and re-spec** when acceptance criteria are ambiguous, scope grows beyond
the issue, or integration tests reveal an unspecified requirement.

---

## Constraint layers for AI-assisted work

To get production-quality output from coding agents, FRET uses three constraint
layers (described in detail in the
[SDD article](https://alexandrelheinen.pages.dev/articles/2026-04-22-ai-agents-sdd/)).
Together they make hallucination visible and expensive to ignore.

### Layer 1 — Agent instructions (point to this file)

`AGENTS.md`, `CLAUDE.md`, and `.github/copilot-instructions.md` must **only**
direct agents to this document and [docs/guidelines.md](docs/guidelines.md).
Keep them short. Models have limited context—essential rules live here, not
scattered across ten files.

### Layer 2 — CI/CD quality gates

Automated checks enforce the method for humans and agents alike. No weakening
gates to make CI green. See [Quality gates](#quality-gates).

Agents must run local validation (`scripts/check/pre_push.sh` or equivalent
steps) before pushing.

### Layer 3 — Human merge

Every change is merged by a maintainer after reviewing CI results and, when
relevant, physical or visual evidence (simulation plots, MuJoCo behavior, bag
files). The agent does not decide when work is done.

---

## Requirement traceability

Requirements must be traceable in both directions: from an acceptance criterion
down to the test that proves it, and from any test back to the criterion.

| Identifier | Document | Example |
| --- | --- | --- |
| `FR-<LAYER>-<NN>` | [docs/requirements.md](docs/requirements.md) | `FR-CTL-02` |
| `SC-<NN>` | [docs/scenarios.md](docs/scenarios.md) | `SC-01` |
| Release | [docs/releases.md](docs/releases.md) | v1.0 |

**Rules:**

- Reference `FR-*` / `SC-*` in issue text, PR descriptions, or test docstrings
  when asserting observable behavior.
- Every new functional requirement needs a scenario or test; a requirement with
  no verification is an incomplete spec.
- Do not silently skip tests (`skip`, `xfail`) without a `reason=` naming the
  blocking requirement or tracking issue.

---

## Quality gates

### Local validation

```bash
# Build and source
./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Recommended: all gates
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

- Python unit test coverage target: **≥ 90%** on `src/fret`
- Python: Google-style docstrings, `black` + `isort` (see [docs/guidelines.md](docs/guidelines.md))
- C++: Doxygen, `clang-format`
- Physical variables: `who_what` naming (e.g. `max_joint_velocity`, not `velocity_max`)

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

### Commit messages

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <imperative subject>

<body>

<footer>
```

Types: `feat`, `fix`, `docs`, `test`, `refactor`, `style`, `chore`, `build`, `ci`

Example:

```
feat(planning): add pillar avoidance scenario SC-05

Implement workspace occupancy integration for two-cylinder world.
Validates FR-PLN-01 and FR-SCN-03 in MuJoCo SITL.

Closes #42
```

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

These rules apply to Cursor agents, Copilot, Claude Code, and any automated
contributor.

### Context and intent

- Infer intent from the full conversation and issue scope, not only the latest message.
- Treat mid-task messages as refinements unless the user clearly changes direction.
- Read **this file** before making changes; it supersedes generic model habits.

### Scope and files

- Do not duplicate workflow rules in new markdown files.
- Do not edit unrelated files unless explicitly asked.
- An imperative order (implement, add, fix…) implies the **full V-cycle**, not code alone.

### Execution order

1. **SDD** — Confirm spec / acceptance criteria exist.
2. **V-cycle** — Identify level; update `docs/` when Level 1–2 change.
3. **Level 3** — Stubs + tests together when adding APIs.
4. **Level 4** — Implement; run `scripts/check/pre_push.sh` (or `--skip-ros` when ROS unavailable).
5. **PR** — Push only after **at minimum** `scripts/check/formatting.sh` and
   `scripts/check/types.sh` pass locally; ensure CI green; do not claim done
   while checks fail. Include a proportional **proof report** per the
   `report-writing` skill (see Communication below).

### Git safety

- Never force-push to `main`.
- Never skip hooks unless explicitly requested.
- Never commit secrets.
- Create commits when completing work that implies a PR.

### Communication

- Be precise; use code citations when referencing existing code.
- Keep responses proportional to task complexity.
- **Proof reports:** When creating or updating a PR, follow the
  [`report-writing` skill](.cursor/skills/report-writing/SKILL.md) (also
  [AGENTS.md § Proof reports](AGENTS.md#proof-reports-mandatory)). Report depth
  scales T0–T4 with diff size and problem difficulty; chronic or visual bugs
  require measurable evidence, not assertions.

---

## Reference documents

When an external guide conflicts with this file or [docs/guidelines.md](docs/guidelines.md),
**this repository wins**.

### FRET specifications

| Document | Role |
| --- | --- |
| [docs/requirements.md](docs/requirements.md) | Level 1 — `FR-*` functional requirements |
| [docs/scenarios.md](docs/scenarios.md) | Level 1 — `SC-*` scenario validation |
| [README.md § Architecture](../README.md#architecture) | Level 2 — system design |
| [docs/interfaces.md](docs/interfaces.md) | Level 2 — typed contracts, QoS, FSMs |
| [docs/mujoco.md](docs/mujoco.md) | Level 2 — MuJoCo simulation integration |
| [docs/simulation.md](docs/simulation.md) | User guide — MuJoCo modes and quick start |
| [docs/releases.md](docs/releases.md) | Release specification v1.0–v1.4 |
| [docs/mujoco_physics_v1.2.md](docs/mujoco_physics_v1.2.md) | v1.2 physics SITL engineering spec |
| [docs/guidelines.md](docs/guidelines.md) | Coding standards and formatting |

### Methodology (external)

| Topic | Resource |
| --- | --- |
| SDD | [GitHub Spec Kit — spec-driven.md](https://github.com/github/spec-kit/blob/main/spec-driven.md) |
| SDD + agents | [Author article — SDD and agentic AI](https://alexandrelheinen.pages.dev/articles/2026-04-22-ai-agents-sdd/) |
| V-model | [Teaching Agile — V-Model](https://teachingagile.com/sdlc/models/v-model) |
| TDD | [Kent Beck — Canon TDD](https://newsletter.kentbeck.com/p/canon-tdd) |
| ROS 2 | [ROS 2 documentation](https://docs.ros.org/) |

---

## Pre-merge checklist

Every contributor (human or agent) must confirm before merge:

- [ ] Spec is clear: intent, scope, and acceptance criteria addressed.
- [ ] Work follows SDD → V-cycle → TDD at the appropriate levels.
- [ ] `FR-*` / `SC-*` traceability updated when behavior changed.
- [ ] `bash scripts/check/formatting.sh` passes.
- [ ] `bash scripts/check/types.sh` passes.
- [ ] `bash scripts/tests/unit.sh` passes (when ROS workspace available).
- [ ] CI green on the PR branch.
- [ ] Documentation updated when specs or public APIs changed.
- [ ] No quality gate weakened to pass.
- [ ] No secrets committed.
- [ ] Public APIs typed and documented.

If any item fails, fix forward—do not merge.

---

## License

By contributing, you agree that your contributions will be licensed under the
MIT License.
