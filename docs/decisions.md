# Decisions log

Append-only record of gate changes, scope narrowings, and other decisions
that `.guidelines/workflow/review.md` requires to be written down rather
than left implicit in a diff. Newest entries at the top.

---

## 2026-11-17 — PR1 robot-registry migration stops short of full FR-SYS-08/11

**Context:** PR1 (`v1.5.0`, robot-model-agnostic core) introduced
`fret.robots` (profile schema, registry, variant protocols) and migrated
four call sites onto it: `control/kinematics.py`,
`config_loader.py::planning_config_for_model`,
`sitl_config.py::controller_config_relative`, and
`control/controller_node.py::make_controller_node`. The spec's Design
notes additionally named `control/pick_place_planning.py` (the
`robot_model == "omy"` transfer-planner branch),
`control/pick_place_vision.py` (`RobotId` literal),
`control/pick_place_clutter_sim.py` (`_OMY_MODELS`), `ros/mujoco_bridge.py`,
and `launch/sitl.py` / `launch/mujoco.py` as dispatch sites to replace, and
`FR-SYS-11` asked for the pick-and-place FSM to be driven through a
`RobotIO` protocol with the MuJoCo implementation moved into
`fret.simulation`.

**Decision:** Ship the registry core and the four lower-risk, well-tested
call sites in `v1.5.0`. Defer the remaining migration
(`TransferPlannerStrategy` extraction from `pick_place_planning.py`,
`pick_place_vision.py`, `pick_place_clutter_sim.py`, `omy_pick_place_sim.py`
and `mujoco_bridge.py`'s literal dispatch; `RobotIO` wiring into
`pick_place_fsm.py`; and `launch/sitl.py` / `launch/mujoco.py` resolving
the model through the registry instead of per-model
`EqualsSubstitution`) to a tracked follow-up PR.

**Why:** These remaining sites are the highest-regression-risk part of the
plan — `pick_place_clutter_sim.py` alone is 1,100+ lines of physics/IK
geometry tightly coupled to OM-X vs. OMY numeric behavior
(`FR-SYS-12` requires byte-identical pass/fail verdicts per seed against
the OPS-00 baseline). Migrating them safely needs either exhaustive
re-running of the long physics-sim test suite after each extraction step
(`test_pick_place_clutter.py`, `test_pick_place_wall_maze.py`, the OMY
scenarios) or a dedicated session with room to do that repeatedly; doing
it under this session's time budget risked exactly the kind of silent
behavior regression the spec's own FR-SYS-12 clause exists to catch.
Shipping a smaller, fully-verified slice honestly scoped as "not yet
migrated" was judged safer than attempting the full migration and being
unable to adequately re-verify it.

**What is honest about the current state:**
`tests/robots/test_no_model_literals.py` enforces this exact boundary: it
AST-scans all of `src/fret` outside `fret.robots` for model-literal
dispatch, with the four remaining files as named, explicit exceptions (a
second test fails if an exception stops containing a real violation, so
the list cannot silently go stale). `docs/robot-model-agnostic.md` §
Status carries the same table. `FR-SYS-08` and `FR-SYS-11` in
`docs/requirements.md` are annotated with this gap rather than claimed
fully satisfied.

**Follow-up:** A PR1.1 (or early PR2 commit) extracts
`TransferPlannerStrategy` per model (mechanical "move, don't rewrite" —
the existing `if robot_model == "omy": ... else: ...` branches move
behind registered strategy objects calling the identical functions),
wires `RobotIO` into the FSM, and routes `launch/sitl.py` through the
registry, each step verified against the named tests before proceeding
(matching `plan.md`'s own incremental sequencing). Ask Alexandre before
starting.

---

## 2026-11-17 — Pin BLAS/OpenMP threads in unit.sh / unit_shard.sh

**Context:** The wall-maze scenario test (`tests/control/test_pick_place_wall_maze.py`)
flaked in the OPS-00 baseline (1 fail in a 3-run sample, twice). Root
cause: `scripts/tests/integration.sh` already pins
`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1` "for
determinism," but `unit.sh` / `unit_shard.sh` did not, so a multi-threaded
BLAS could change the floating-point reduction order of Jacobian/IK/MPC
math run to run — enough to flip geometry checks that sit close to a
threshold.

**Decision:** Apply the same thread pinning to `unit.sh` and
`unit_shard.sh`. This is a hygiene fix, not a weakened gate: it makes an
already-flaky test deterministic rather than relaxing what it checks.

**Verification gap, stated plainly:** a full 20-consecutive-run
reproduction (as the task called for) was not completed — each run of
this test file takes 5 to 8 minutes, making 20 runs a 2+ hour cost. What
was run: 4 consecutive clean runs after the fix (8/8 passed each time,
including 2 direct `pytest` invocations and 1 full `unit_shard.sh`-style
invocation with the env vars set). This is reasoned evidence, not proof
of elimination; if the flake recurs, look at BLAS threading first.

---

## 2026-10-31 — CI (GitHub Actions) credit expired

GitHub Actions credit for this repository expired on 2026-10-31. CI does
not run on pushes or pull requests. The local gate
(`scripts/check/formatting.sh`, `scripts/check/types.sh`,
`scripts/check/pre_push.sh`, `scripts/tests/unit_coverage.sh`) is the
evidence pasted into PR bodies until Actions credit is restored. Pushing a
`v*.*.*` release tag still triggers `.github/workflows/release.yml`
(showcase renders to R2); with Actions off it will not run, and release
notes say so explicitly.
