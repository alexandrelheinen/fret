"""Tests for fret._cli module.

The ``fretsim`` entry point requires the ``ros2`` CLI tool which is not
available in the unit-test environment.  These tests verify only that the
module can be imported without error (covering the module-level imports).
"""

from __future__ import annotations


def test_cli_module_importable() -> None:
    """fret._cli must be importable without side-effects."""
    import fret._cli as cli

    assert hasattr(cli, "fretsim")
