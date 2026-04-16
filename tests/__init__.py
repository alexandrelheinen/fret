"""Test suite for FRET.

All test sub-packages mirror the source layout under ``src/fret/``.
Tests are written before implementation (V-cycle step 2); method bodies
that exercise unimplemented stubs are decorated with::

    @pytest.mark.xfail(strict=True, raises=NotImplementedError)

This decoration will cause CI to fail as soon as a stub is silently
implemented without a test update — ensuring test coverage is kept tight.
"""
