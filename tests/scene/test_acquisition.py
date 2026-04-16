"""Tests for the scene acquisition layer (fret.scene.SceneAcquisition).

All functional tests are marked xfail(strict=True) because ``SceneAcquisition``
is a stub at this stage.  As implementation progresses, remove the decorator
from tests that should now pass.

Acceptance criteria (from docs/scenarios.md — SC-01 and FR-SCN-01/02/04):
  - ``get_latest_payload()`` returns an ``OccupancyUpdatePayload``.
  - ``payload.frame_id`` is always ``"world"``.
  - ``payload.obstacle_points`` shape is ``(N, 3)`` with N ≥ 0.
  - Calling before first message raises ``RuntimeError``.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.acquisition import SceneAcquisition


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction_requires_node() -> None:
    """SceneAcquisition must accept a ROS 2 node handle."""
    SceneAcquisition(node=None)  # type: ignore[arg-type]


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_get_latest_payload_returns_payload(mock_node: object) -> None:
    """get_latest_payload() must return an OccupancyUpdatePayload."""
    acq = SceneAcquisition(node=mock_node)
    payload = acq.get_latest_payload()
    assert isinstance(payload, OccupancyUpdatePayload)


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_frame_id_is_world(mock_node: object) -> None:
    """All returned payloads must carry frame_id == 'world' (FR-SCN-02)."""
    acq = SceneAcquisition(node=mock_node)
    payload = acq.get_latest_payload()
    assert payload.frame_id == "world"


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_obstacle_points_shape(mock_node: object) -> None:
    """obstacle_points must be shape (N, 3)."""
    acq = SceneAcquisition(node=mock_node)
    payload = acq.get_latest_payload()
    assert payload.obstacle_points.ndim == 2
    assert payload.obstacle_points.shape[1] == 3


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_raises_before_first_message(mock_node: object) -> None:
    """get_latest_payload() must raise RuntimeError before first message."""
    acq = SceneAcquisition(node=mock_node)
    with pytest.raises(RuntimeError):
        acq.get_latest_payload()
