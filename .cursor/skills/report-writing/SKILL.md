---
name: report-writing
description: >-
  Write proportional proof reports when creating or updating pull requests,
  completing cloud-agent tasks, or when the maintainer asks whether work
  actually works. Load on every new PR creation, PR update, release fix, or
  long-running bug investigation. Depth scales with PR size and problem
  difficulty — one sentence for trivial tuning, engineer-grade evidence
  (metrics, graphs, renders) for large refactors and chronic failures.
---

# Report writing (proof that work works)

Agents must **prove** outcomes, not assert them. A report is the deliverable
that lets the maintainer decide without re-running your investigation.

Use this skill **whenever you create or update a PR**, finish a cloud-agent
task, or the user asks “does it work?”, “convince me”, or “show me evidence”.

Official agent policy also lives in [AGENTS.md](../../../AGENTS.md) (Proof
reports section) and [CONTRIBUTING.md](../../../CONTRIBUTING.md) (Rules for AI
agents).

---

## Choose report depth (mandatory)

Estimate **tier** from (a) diff size, (b) blast radius, (c) how long the
problem has resisted fix, (d) whether behavior is stochastic or visual.

| Tier | When | User-facing report |
| --- | --- | --- |
| **T0** | Typo, comment, pure format, single-line config with obvious effect | **One sentence** — what changed and that gates passed |
| **T1** | Small fix (≤3 files, ≤~80 LOC), deterministic pass/fail | **2–4 bullets** — problem, fix, verification command + result |
| **T2** | Multi-file feature/fix, CI repair, new test coverage | **Short sections** — Problem / Fix / Verification; include gate output or test counts |
| **T3** | Release blocker, flaky CI, physics/sim/render pipeline, cross-module bug | **Evidence report** — metrics table + at least one chart or before/after comparison |
| **T4** | Large refactor, architecture change, chronic issue (multiple failed PRs), release tag | **Engineer-grade report** — full layer stack below + artifacts the maintainer can open |

**Escalate one tier** when any of these apply:

- Prior agent PR claimed success but CI or user still red
- Stochastic behavior (RNG, timing, race, CI runner variance)
- Visual/export output (video, MuJoCo, plots) where logs lie
- Maintainer said they already know the failure mode (e.g. “time-compression”,
  “psychological manipulation”, “flake”) — show **mechanism**, not symptoms

**Never** ship T0 for a T3 problem.

---

## Engineer-grade report layers (T3–T4)

Build evidence in order. Each layer answers a skeptic question.

### Layer 1 — Reproduce verbatim

- Quote the **exact** CI log line, exception, or user-reported run URL
- Re-run the same command locally; show matching numbers (duration, error code)

*Answers: “Did you actually hit my failure?”*

### Layer 2 — Control variables

- Change **one** thing at a time (seed, timeout, flag, commit)
- Same environment assumptions as CI (EGL, pytest plugins, workflow flags)

*Answers: “Is this environment noise or your code?”*

### Layer 3 — State space / time domain

- Trajectories, tables, bar charts — not raw log dumps
- Plot **viewer time vs simulation time** when export/resampling is involved
- Success rates over N trials when behavior is stochastic

*Answers: “What actually happened in the system?”*

### Layer 4 — Pixel / artifact proof

- Screenshots, MP4 frame grabs, MuJoCo renders via the **same script** as CI
- Save under `/opt/cursor/artifacts/<topic>/` and embed in PR or summary

*Answers: “Would I see the same thing in the product?”*

### Layer 5 — Fix chain (decision guide)

Close with a short causal chain:

```
root cause → symptom → wrong fix (if any) → correct fix → verification
```

*Answers: “What should I merge and what should I watch?”*

---

## Exemplar (T4)

**Case:** Dubins physics release export — failed runs resampled from 300 s into
35 s clips (8.6× apparent speedup); rejecting fake clips exposed unseeded
planner flake (~25% CI failure).

**What we showed:**

1. CI error verbatim (`race_duration_s=300.0`, `max_cross_track_error_m=67.8`)
2. 8×8 trial success rate (unseeded 75% vs seeded 100%)
3. XY trajectory overlay (same physics, different plan)
4. Viewer clock vs sim clock plot (resampling mechanism)
5. MuJoCo overview snapshots (goal at x≈74 m only when seeded success)

Methodology is reusable; see `scripts/evidence_dubins_physics_seed.py` when
present on the branch.

---

## Where to write the report

| Audience | Location |
| --- | --- |
| Maintainer (chat) | Final turn summary — lead with verdict, then layers |
| PR reviewers | PR body: **Verification** section; embed images or link artifacts |
| Long investigations | Optional `docs/postmortems/<topic>.md` only if user asks |

PR body minimum (T1+):

```markdown
## Verification
- Commands run: `...`
- Result: pass / fail counts, timings, key metric
- [T3+] Evidence: charts or screenshots in summary or artifacts
```

---

## Anti-patterns (do not)

- “Should work” / “CI will pass” without command output
- Describing fixes without showing the **failure mode** first
- Replacing one deception with another (e.g. fake pass by skipping export checks)
- Wall of grep output instead of one chart
- Claiming flake is fixed from a single lucky run

---

## Commands to cite often

```bash
bash scripts/check/formatting.sh
bash scripts/check/types.sh
bash scripts/check/pre_push.sh --skip-ros
python3 -m pytest tests/ --ignore=tests/integration -p no:launch_testing -p no:launch_ros
```

For render/physics evidence:

```bash
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
```

---

## Cursor Automations (optional, maintainer)

Repo skills are loaded by the agent from `.cursor/skills/`; they are **not**
auto-triggered by GitHub events on their own. To run report-style review on
every PR open, create a [Cursor Automation](https://cursor.com/docs/cloud-agent/automations)
with trigger **PR opened** and a prompt that references this skill
(`report-writing`).
