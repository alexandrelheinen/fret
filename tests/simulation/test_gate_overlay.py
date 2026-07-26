"""Unit tests for gate-cam overlay annotation helpers."""

from __future__ import annotations

import numpy as np

from fret.simulation.gate_overlay import annotate_gate_frame


def test_annotate_gate_frame_draws_pose_text() -> None:
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    # Yellow-ish blob near tennis HSV band.
    frame[40:80, 60:100] = (40, 200, 200)
    out = annotate_gate_frame(frame, (0.22, -0.20, 0.020), label="CV")
    assert out.shape == frame.shape
    assert out.dtype == np.uint8
    # Text + circle should change some pixels vs the blank canvas edges.
    assert float(out[:30].mean()) > 1.0
