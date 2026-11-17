"""``fret-diag loop`` — bridge cycle stats against the mock bus."""

from __future__ import annotations

import time

from fret.diag.core import CheckResult, CheckStatus
from fret.hardware.bridge_core import make_bridge_core_for_mock


def run_loop_checks(*, cycles: int = 50) -> list[CheckResult]:
    core, _bus = make_bridge_core_for_mock()
    core.connect()
    times: list[float] = []
    for i in range(cycles):
        t0 = time.perf_counter()
        core.cycle(float(i) * 0.02)
        times.append(time.perf_counter() - t0)
    times_sorted = sorted(times)
    p99 = times_sorted[max(0, int(0.99 * (len(times_sorted) - 1)))]
    ok = p99 < 0.020
    return [
        CheckResult(
            "loop",
            CheckStatus.PASS if ok else CheckStatus.FAIL,
            f"bridge cycle p99={p99*1000:.2f} ms over {cycles} cycles",
            {"p99_s": p99, "cycles": cycles},
        )
    ]
