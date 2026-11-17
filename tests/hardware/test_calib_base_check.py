"""FR-HW-34: base-check error aggregation."""

from __future__ import annotations

import numpy as np
import pytest

from fret.calib.base_check import evaluate_base_check


def test_base_check_pass_and_fail() -> None:
    tips = [np.zeros(3) for _ in range(4)]
    boards = [np.array([0.001, 0.0, 0.0]) for _ in range(4)]
    ok = evaluate_base_check(tips, boards, pass_threshold_m=0.010)
    assert ok.passed
    bad_boards = [np.array([0.05, 0.0, 0.0]) for _ in range(4)]
    bad = evaluate_base_check(tips, bad_boards, pass_threshold_m=0.010)
    assert not bad.passed
    assert bad.max_error_m == pytest.approx(0.05)
