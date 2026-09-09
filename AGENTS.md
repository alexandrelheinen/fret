# AGENTS Instructions

This file is a bridge only. **Do not add rules here.**

Shared engineering guidelines live in [.guidelines/](.guidelines/) (a git
submodule):

- [.guidelines/workflow/sdd.md](.guidelines/workflow/sdd.md), [integration.md](.guidelines/workflow/integration.md), [tdd.md](.guidelines/workflow/tdd.md) — how work gets done
- [.guidelines/agents/writing.md](.guidelines/agents/writing.md) — how any prose should read
- [.guidelines/style/naming.md](.guidelines/style/naming.md) — naming
- [.guidelines/languages/py.md](.guidelines/languages/py.md), [cpp.md](.guidelines/languages/cpp.md), [cmake.md](.guidelines/languages/cmake.md), [sh.md](.guidelines/languages/sh.md) — Python, C++, CMake, shell

For FRET's own project context, ecosystem, quality gates, and merge
policy, read [CONTRIBUTING.md](CONTRIBUTING.md). For FRET-specific coding
notes, read [docs/guidelines.md](docs/guidelines.md). For durable
environment/debugging notes (ROS sourcing, pytest plugin conflicts,
headless MuJoCo, known pre-existing breakages), read
[docs/dev-environment.md](docs/dev-environment.md).

Conflict order: direct maintainer request > CONTRIBUTING.md > `.guidelines/`
> `docs/guidelines.md` > ROS 2 official documentation.

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
[`.cursor/skills/report-writing/SKILL.md`](.cursor/skills/report-writing/SKILL.md)
whenever you create or update a PR, finish a cloud-agent task, or the
maintainer asks for evidence. Follow its tier table (T0-T4) and layer
stack. Minimum: cite commands run and pass/fail results.
