"""Release variant YAMLs must carry showcase-honesty MPC / wall FAULT keys.

Variant files (``*_rrt.yml`` / ``*_sst.yml``) are full copies, not includes —
forgetting to copy ``fault_on_wall_contact`` / ``mpc_cspace_*`` silently
disables barriers on the clips that ``release.yml`` actually renders.
"""

from __future__ import annotations

from pathlib import Path

import yaml

_SCENARIO_DIR = Path("src/fret/config/scenarios")
_RELEASE_VARIANTS = (
    "omx_wall_maze_rrt.yml",
    "omx_wall_maze_sst.yml",
    "omy_clutter_rrt.yml",
    "omy_clutter_sst.yml",
)
_REQUIRED = (
    "mpc_cspace_clearance_rad",
    "mpc_cspace_samples",
    "mpc_weight_obstacle",
    "fault_on_wall_contact",
    "wall_contact_fault_steps",
)


def _params(path: Path) -> dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    block = (raw or {}).get("/**") or raw or {}
    return dict(block.get("ros__parameters") or block)


def test_release_variant_yamls_enable_mpc_wall_honesty() -> None:
    for name in _RELEASE_VARIANTS:
        path = _SCENARIO_DIR / name
        assert path.is_file(), name
        params = _params(path)
        for key in _REQUIRED:
            assert key in params, f"{name} missing {key}"
        assert bool(params["fault_on_wall_contact"]) is True
        assert float(params["mpc_weight_obstacle"]) >= 120.0
        assert int(params["wall_contact_fault_steps"]) >= 5


def test_omy_clutter_release_variants_use_named_planners() -> None:
    """RRT*/SST showcase clips must not share the YAML detour shortcut."""
    rrt = _params(_SCENARIO_DIR / "omy_clutter_rrt.yml")
    sst = _params(_SCENARIO_DIR / "omy_clutter_sst.yml")
    assert rrt.get("planner_algorithm") == "rrt_star"
    assert sst.get("planner_algorithm") == "sst"
    assert bool(rrt.get("prefer_transfer_detour", True)) is False
    assert bool(sst.get("prefer_transfer_detour", True)) is False


def test_omy_clutter_rrt_and_sst_transfer_paths_differ() -> None:
    """Named planners must produce distinct transfer trajectories."""
    import numpy as np

    from fret.control.pick_place_planning import plan_arm_transfer_path
    from fret.control.pick_place_sim import waypoints_from_scenario

    paths: list[np.ndarray] = []
    for name in ("omy_clutter_rrt.yml", "omy_clutter_sst.yml"):
        scenario = _SCENARIO_DIR / name
        wp = waypoints_from_scenario(scenario)
        path, straight = plan_arm_transfer_path(
            wp.lift_hover,
            wp.place_hover,
            scenario_path=scenario,
            seed_offset=0,
        )
        assert straight, f"{name}: wall must still block the chord"
        assert len(path) >= 3, f"{name}: expected planned path"
        paths.append(np.stack(path))
    rrt, sst = paths
    # Whole-path equality is the release bug (identical showcase clips).
    min_len = min(len(rrt), len(sst))
    assert not np.allclose(rrt[:min_len], sst[:min_len], atol=1e-3) or len(
        rrt
    ) != len(sst)
