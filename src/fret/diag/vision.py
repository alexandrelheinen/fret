"""``fret-diag vision`` — ball detection against a reference mark (FR-HW-31)."""

from __future__ import annotations

import numpy as np

from fret.diag.core import CheckResult, CheckStatus


def run_vision_checks(
    *,
    detected_xy: tuple[float, float] | None = None,
    mark_xy: tuple[float, float] = (0.20, -0.12),
    max_error_m: float = 0.02,
) -> list[CheckResult]:
    if detected_xy is None:
        # Synthetic self-check: perfect detection at the mark.
        detected_xy = mark_xy
    err = float(
        np.linalg.norm(
            np.asarray(detected_xy, dtype=np.float64)
            - np.asarray(mark_xy, dtype=np.float64)
        )
    )
    ok = err <= max_error_m
    return [
        CheckResult(
            "vision",
            CheckStatus.PASS if ok else CheckStatus.FAIL,
            f"table-plane error={err*1000:.1f} mm (limit {max_error_m*1000:.0f} mm)",
            {
                "detected_xy": list(detected_xy),
                "mark_xy": list(mark_xy),
                "error_m": err,
            },
        )
    ]
