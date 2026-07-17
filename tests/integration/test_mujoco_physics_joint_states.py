"""V12-4 / FR-SIM-07: joint-state provenance under physics_mode."""

from __future__ import annotations

import numpy as np
import pytest

from fret.ros.mujoco_bridge import (
    _load_merged_bridge_config,
    _resolve_config_path,
    make_dubins_race_bridge_core,
    physics_config_from_bridge_yaml,
)


def _mujoco_available() -> bool:
    return make_dubins_race_bridge_core().has_mujoco_runtime


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_physics_joint_states_match_simulated_qpos() -> None:
    """V12-4: physics step advances qpos — no open-loop pose injection."""
    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([6.0, 6.0, 0.0]),
        initial_sst=np.array([6.0, 6.4, 0.0]),
        physics_config=physics,
    )
    assert core.physics_mode is True

    q0 = core.get_rrt_pose().copy()
    core.step_physics(np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    q1 = core.get_rrt_pose()
    assert q1[0] > q0[0], "physics step must advance RRT X from actuators"
    assert np.any(core.get_joint_velocities() != 0.0)

    with pytest.raises(RuntimeError, match="set_rrt_pose"):
        core.set_rrt_pose((0.0, 0.0, 0.0))
