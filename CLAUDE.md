# Claude instructions

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
[docs/guidelines.md](docs/guidelines.md). For environment/debugging notes,
read [docs/dev-environment.md](docs/dev-environment.md).

Before push, run at minimum:

```bash
bash scripts/check/formatting.sh
bash scripts/check/types.sh
```
