"""PPP warehouse scenario orchestration (v1.0 E2E).

Pure-Python end-to-end runner for SC-v10 acceptance criteria V10-2 through
V10-5.  Wires planning, trajectory generation, PPP control, MuJoCo bridge
I/O, and the magnetic grasp FSM without a live ROS context.
"""

from __future__ import annotations

from fret.scenario.ppp_warehouse_runner import (
    PPPWarehouseRunner,
    PPPWarehouseRunResult,
)

__all__ = [
    "PPPWarehouseRunner",
    "PPPWarehouseRunResult",
]
