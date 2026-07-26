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
