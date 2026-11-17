"""FR-HW-66: motion diagnostics refuse without guards."""

from __future__ import annotations

from fret.diag.core import CheckStatus
from fret.diag.e2e import run_e2e_checks
from fret.diag.gripper import run_gripper_checks
from fret.diag.joints import run_joints_checks


def test_joints_fail_without_allow_motion() -> None:
    results = run_joints_checks(allow_motion=False, confirmed=True)
    assert results[0].status is CheckStatus.FAIL


def test_gripper_skip_without_confirm() -> None:
    results = run_gripper_checks(allow_motion=True, confirmed=False)
    assert results[0].status is CheckStatus.SKIP


def test_e2e_pass_with_guards() -> None:
    results = run_e2e_checks(allow_motion=True, confirmed=True)
    assert results[0].status is CheckStatus.PASS
