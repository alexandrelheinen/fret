"""SO-101 pure-NumPy kinematics tests (FR-SIM-16, FR-SIM-17)."""

from __future__ import annotations

import ast
import inspect

import numpy as np
import pytest

from fret.control import kinematics_so101
from fret.control.kinematics_so101 import (
    SO101Kinematics,
    UnreachablePoseError,
    forward_kinematics_matrix,
    joint_limits_array,
    solve_ik,
)

mujoco = pytest.importorskip("mujoco")

_XML = (
    "third_party/robotstudio_so101/so101.xml"
)
_ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def _mujoco_oracle():
    model = mujoco.MjModel.from_xml_path(_XML)
    data = mujoco.MjData(model)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    qpos_adrs = [
        int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        for n in _ARM_JOINTS
    ]
    return model, data, site_id, qpos_adrs


def test_kinematics_so101_module_does_not_import_mujoco() -> None:
    """FR-SIM-16: the Pi runtime has no MuJoCo; this module must not need it."""
    source = inspect.getsource(kinematics_so101)
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            assert all(alias.name != "mujoco" for alias in node.names)
        if isinstance(node, ast.ImportFrom):
            assert node.module != "mujoco"


def test_fk_matches_mujoco() -> None:
    """FR-SIM-16: FK matches MuJoCo's gripperframe site pose to 1e-6."""
    model, data, site_id, qpos_adrs = _mujoco_oracle()
    limits = joint_limits_array()
    rng = np.random.default_rng(0)

    max_pos_err = 0.0
    max_rot_err = 0.0
    for _ in range(1000):
        q = rng.uniform(limits[:, 0], limits[:, 1])
        for adr, val in zip(qpos_adrs, q):
            data.qpos[adr] = val
        mujoco.mj_forward(model, data)
        T_mj = np.eye(4)
        T_mj[:3, :3] = data.site_xmat[site_id].reshape(3, 3)
        T_mj[:3, 3] = data.site_xpos[site_id]

        T_me = forward_kinematics_matrix(q)
        max_pos_err = max(
            max_pos_err, float(np.linalg.norm(T_mj[:3, 3] - T_me[:3, 3]))
        )
        max_rot_err = max(
            max_rot_err, float(np.max(np.abs(T_mj[:3, :3] - T_me[:3, :3])))
        )

    assert max_pos_err < 1e-6, f"max position error {max_pos_err}"
    assert max_rot_err < 1e-6, f"max rotation error {max_rot_err}"


def test_jacobian_matches_mujoco_mj_jacsite() -> None:
    """Finite-difference Jacobian agrees with MuJoCo's analytic ``mj_jacSite``."""
    model, data, site_id, qpos_adrs = _mujoco_oracle()
    limits = joint_limits_array()
    rng = np.random.default_rng(1)
    kin = SO101Kinematics()

    for _ in range(20):
        q = rng.uniform(limits[:, 0], limits[:, 1])
        for adr, val in zip(qpos_adrs, q):
            data.qpos[adr] = val
        mujoco.mj_forward(model, data)

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        cols = [
            int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
            for n in _ARM_JOINTS
        ]
        J_mj = np.vstack([jacp[:, cols], jacr[:, cols]])

        J_me = kin.jacobian(q)
        np.testing.assert_allclose(J_me[:3], J_mj[:3], atol=1e-4)


def test_ik_tip_down_workspace() -> None:
    """FR-SIM-17: targets inside the reachable workspace solve within 1 mm."""
    rng = np.random.default_rng(2)
    n_ok = 0
    for _ in range(50):
        radius = rng.uniform(0.12, 0.28)
        angle = rng.uniform(-1.5, 1.5)
        target = np.array(
            [radius * np.cos(angle), radius * np.sin(angle), 0.05]
        )
        result = solve_ik(target)
        if result.reachable:
            n_ok += 1
            T = forward_kinematics_matrix(result.q)
            assert float(np.linalg.norm(T[:3, 3] - target)) < 1e-3
            limits = joint_limits_array()
            assert np.all(result.q >= limits[:, 0] - 1e-9)
            assert np.all(result.q <= limits[:, 1] + 1e-9)
    assert n_ok >= 45, f"only {n_ok}/50 workspace targets solved"


def test_unreachable_is_typed() -> None:
    """FR-SIM-17: an out-of-workspace target raises, never silently clips."""
    far_away = np.array([5.0, 5.0, 5.0])
    with pytest.raises(UnreachablePoseError):
        SO101Kinematics().inverse_kinematics(far_away)

    result = solve_ik(far_away)
    assert result.reachable is False


def test_generated_constants_match_vendored_mjcf() -> None:
    """The hardcoded chain must match ``scripts/gen_so101_kinematics.py``."""
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo_root / "scripts"))
    import gen_so101_kinematics as gen

    chain, ee_site = gen.parse_so101_chain(repo_root / _XML)
    hardcoded = kinematics_so101.chain_constants()
    for jf, (name, pos, quat, rng) in zip(chain, hardcoded):
        assert jf.name == name
        assert jf.pos == pos
        assert jf.quat == quat
        assert jf.joint_range == rng
    assert (ee_site.pos, ee_site.quat) == kinematics_so101.ee_site_constants()
