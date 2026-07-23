"""Resolve checked-in PlotJuggler layout XML for a scenario."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import yaml

_LAYOUTS_DIR = Path(__file__).resolve().parent / "layouts"
_INDEX_NAME = "index.yaml"


def layouts_dir() -> Path:
    """Return the on-disk layouts directory."""
    return _LAYOUTS_DIR


def load_layout_index() -> Mapping[str, str]:
    """Return ``scenario_id → layout basename`` from ``layouts/index.yaml``."""
    index_path = layouts_dir() / _INDEX_NAME
    if not index_path.is_file():
        raise FileNotFoundError(
            f"missing PlotJuggler layout index: {index_path}"
        )
    data = yaml.safe_load(index_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(
            f"invalid layout index (expected mapping): {index_path}"
        )
    return {str(k): str(v) for k, v in data.items()}


def layout_path_for_scenario(scenario_id: str) -> Path:
    """Return absolute path to the PlotJuggler layout for ``scenario_id``.

    Raises:
        KeyError: unknown scenario_id.
        FileNotFoundError: index or XML missing.
    """
    index = load_layout_index()
    try:
        basename = index[scenario_id]
    except KeyError as exc:
        known = ", ".join(sorted(index))
        raise KeyError(
            f"no PlotJuggler layout for scenario {scenario_id!r}; "
            f"known: {known}"
        ) from exc
    path = layouts_dir() / basename
    if not path.is_file():
        raise FileNotFoundError(f"PlotJuggler layout missing: {path}")
    return path.resolve()
