# Claude instructions

This file is a bridge only. **Do not add rules here.**

@.guidelines/workflow/sdd.md
@.guidelines/workflow/integration.md
@.guidelines/workflow/tdd.md
@.guidelines/agents/writing.md
@.guidelines/style/naming.md
@.guidelines/languages/py.md
@.guidelines/languages/cpp.md
@.guidelines/languages/cmake.md
@.guidelines/languages/sh.md

For FRET's own project context, quality gates, and merge policy, read
[CONTRIBUTING.md](CONTRIBUTING.md). For FRET-specific coding notes, read
[docs/guidelines.md](docs/guidelines.md). For environment and debugging
notes (ROS sourcing, the `.venv` requirement, pytest plugin conflicts,
headless MuJoCo, known pre-existing breakages), read
[docs/dev-environment.md](docs/dev-environment.md).

Conflict order: direct maintainer request > CONTRIBUTING.md > `.guidelines/`
> `docs/guidelines.md` > ROS 2 official documentation.

## Pre-push gates (mandatory)

Formatting plus types alone is enough only for docs or comment changes.
Whenever code or tests change, size the local gate to the blast radius,
see [CONTRIBUTING.md § Pre-push by blast radius](CONTRIBUTING.md#pre-push-by-blast-radius).
Run the helper, which maps `git diff` paths to the matching CI shards:

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

After push, wait until CI is green for the affected jobs. A local subset
pass does not mean done.

## Proof reports (mandatory)

Prove that work works, not only that it was implemented and pushed. Load
[`.cursor/skills/report-writing/SKILL.md`](.cursor/skills/report-writing/SKILL.md)
when creating or updating a PR, finishing a cloud-agent task, or when the
maintainer asks for evidence. Minimum: cite the commands run and their
pass/fail results.
