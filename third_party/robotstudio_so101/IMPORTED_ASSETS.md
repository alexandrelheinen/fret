# Imported SO-101 MJCF (Menagerie `robotstudio_so101`)

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101
Upstream commit: `3abca2e302625cec9429c49806121ea2855cef81` (2026-09-28, "Fix SO101
wrist_roll joint range upper limit (#324)")
License: Apache-2.0 (`LICENSE` in this directory, copied verbatim from upstream)
Local edits: none. Files are a byte-identical copy of the upstream
`robotstudio_so101/` subdirectory (`so101.png` thumbnail excluded — not needed by
any FRET scene or test).

PR2 vendors this subdirectory directly (not as a git submodule, unlike
`third_party/robotis_mujoco_menagerie/`): `robotstudio_so101/` lives inside the
single-repository `google-deepmind/mujoco_menagerie` monorepo together with dozens
of unrelated robot models, so a submodule pointed at that repository would pull in
everything else with it.

## Files

| File | Role |
| --- | --- |
| `so101.xml` | Robot MJCF: 6-body kinematic chain (`base` is fixed; `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper` each carry one revolute joint; `moving_jaw_so101_v1` carries the gripper-open/close joint), position actuators (`sts3215` class, kp 998.22 kv 2.731), visual + collision geometry, and the `gripperframe` end-effector site. |
| `scene.xml`, `scene_box.xml` | Upstream example scenes (ground plane / lighting); not used directly — FRET's own `mjcf/so101_tabletop.xml` composes the robot with FRET's table and workspace assets instead. |
| `assets/*.stl` | Visual/collision meshes (19 files). |
| `LICENSE` | Upstream Apache-2.0 license text. |
| `README.md`, `CHANGELOG.md` | Upstream documentation, unmodified. |

## Refresh procedure

Not scripted (single-robot, low-churn subdirectory); to refresh, re-run the
`curl` download of each file above from a new pinned commit SHA, update the SHA
in this file, and re-run `python3 scripts/gen_so101_kinematics.py --check`
(fails loudly if the joint chain, axes, or site changed) and
`tests/test_third_party_licenses.py`.
