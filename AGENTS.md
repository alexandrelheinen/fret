# AGENTS Instructions

Automated agents working in this repository must follow
[CONTRIBUTING.md](CONTRIBUTING.md) as the **single source of truth** for
development workflow, SDD, V-cycle stages, quality gates, and agent policy.

For coding conventions (Python/C++, ROS 2 layout, formatting, naming), follow
[docs/guidelines.md](docs/guidelines.md).

**Do not duplicate** workflow or V-cycle rules in this file. When instructions
conflict, resolve in this order:

1. Direct maintainer request in the active task
2. [CONTRIBUTING.md](CONTRIBUTING.md)
3. [docs/guidelines.md](docs/guidelines.md)
4. ROS 2 official documentation

An imperative order (implement, add, fix…) always implies the full V-cycle
described in [CONTRIBUTING.md](CONTRIBUTING.md), not code alone.

## Pre-push gates (mandatory)

Do **not** treat `formatting.sh` + `types.sh` as enough whenever code or tests
change. That floor missed a sibling assert on this line of work and broke CI.

**Canonical rule (path → CI shard):** see
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

Pushing with formatting, type-check, or **required-shard** failures is
unacceptable. After push, wait until CI is green for the affected jobs — local
subset pass ≠ done.

## Proof reports (mandatory)

Agents must **prove** that work works — not only implement and push. Report
depth is **proportional** to change size and problem difficulty (trivial tuning
→ one sentence; release blockers and chronic bugs → metrics, charts, renders).

Load the repo skill [`.cursor/skills/report-writing/SKILL.md`](.cursor/skills/report-writing/SKILL.md)
whenever you **create or update a PR**, finish a cloud-agent task, or the
maintainer asks for evidence. Follow its tier table (T0–T4) and layer stack
for engineer-grade cases.

Minimum: cite commands run and pass/fail results. For flaky CI, stochastic
sim, or visual/export pipelines, include measured before/after and at least
one chart or screenshot from the real pipeline.

## Cursor Cloud specific instructions

Durable, non-obvious notes for cloud agents. The base VM snapshot already has
ROS 2 Jazzy Desktop, headless-GL libraries, `xvfb`, and a
complete C/C++ toolchain installed system-wide. The startup update script only
refreshes Python deps (`pip install -e ".[dev,sim]"`, installed to `~/.local`;
this also reinstalls the `arco` git dependency from its `main` branch, so it
needs network access). Standard build/test/run commands live in
[README.md](README.md), [CONTRIBUTING.md](CONTRIBUTING.md), and `scripts/`.

- **Sourcing:** ROS is not sourced by default. Run
  `source /opt/ros/jazzy/setup.bash` and, after building,
  `source install/setup.bash`. `scripts/build.sh`, `scripts/tests/*.sh` source
  these themselves.
- **pytest vs ROS plugins (important):** The pip `pytest` (>=8, currently 9.x)
  is incompatible with ROS Jazzy's `launch_testing` / `launch_ros` pytest
  plugins (they use the removed `path` hook arg). With the ROS overlay sourced,
  plain `pytest` — and thus `scripts/tests/unit.sh` as written — aborts with a
  `PluginValidationError`. Run unit tests with those plugins disabled:
  `python3 -m pytest tests/ --ignore=tests/integration -p no:launch_testing -p no:launch_ros`.
  The same incompatibility blocks `tests/integration/` (launch_testing based)
  under pytest 9.
- **Headless MuJoCo:** viewer/render scripts need EGL. Export
  `MUJOCO_GL=egl PYOPENGL_PLATFORM=egl` (the system EGL libs are in the
  snapshot), e.g. `scripts/render_mujoco.py` / `scripts/video.sh`.
- **render_mujoco motion:** the default controller-tracked showcase path
  (`simulate_tracked_trajectory`) barely advances with the current controller
  config and renders a near-static clip; pass `--no-tracking` to render visible
  planned-path (ARCO RRT*) motion.
- **Known pre-existing breakages (not environment issues):** the pinned
 pinned `arco[mpc] @ v0.3.2` provides CasADi path-following / joint-space
 MPC. Pre-existing: `mypy` reports a missing `Any` import in
 `planner_node_ros.py`; and the ROS `planner_node` crashes at launch because
 `declare_parameter("start_configuration", [])` is inferred as `BYTE_ARRAY`
 under rclpy Jazzy. Other nodes (mujoco_bridge, controller, perception,
 scene) launch cleanly.
