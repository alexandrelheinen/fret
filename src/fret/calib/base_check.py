"""``fret-calib base-check`` — FK tip vs known board points (FR-HW-34)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class BaseCheckResult:
    errors_m: tuple[float, ...]
    max_error_m: float
    passed: bool


def evaluate_base_check(
    tip_positions_m: Sequence[npt.ArrayLike],
    board_points_m: Sequence[npt.ArrayLike],
    *,
    pass_threshold_m: float = 0.010,
) -> BaseCheckResult:
    if len(tip_positions_m) < 4 or len(board_points_m) < 4:
        raise ValueError("need at least 4 tip/board point pairs")
    if len(tip_positions_m) != len(board_points_m):
        raise ValueError("tip and board point counts must match")
    errors: list[float] = []
    for tip, board in zip(tip_positions_m, board_points_m, strict=True):
        err = float(
            np.linalg.norm(
                np.asarray(tip, dtype=np.float64)
                - np.asarray(board, dtype=np.float64)
            )
        )
        errors.append(err)
    max_err = max(errors)
    return BaseCheckResult(
        errors_m=tuple(errors),
        max_error_m=max_err,
        passed=max_err <= pass_threshold_m,
    )
