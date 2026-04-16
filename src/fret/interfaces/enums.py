"""Planning status and error code enumerations.

These enumerations define the vocabulary for communicating planning outcomes
across the FRET/ARCO boundary and through the ROS 2 Action interface.
"""

from __future__ import annotations

import enum


class PlanningStatus(enum.IntEnum):
    """High-level outcome of a planning request.

    Maps one-to-one to the ``status`` field of ``PlanRequest.action`` results.
    """

    SUCCESS = 0
    ABORTED = 1
    CANCELLED = 2


class ErrorCode(enum.IntEnum):
    """Structured error codes returned with a non-SUCCESS PlanningStatus.

    See docs/interfaces.md — Error Propagation for the full propagation table.
    """

    NONE = 0
    TIMEOUT = 1
    NO_PATH_FOUND = 2
    INVALID_CONFIGURATION = 3
    POST_PROCESS_FAILED = 4
    INTERNAL_ERROR = 99
