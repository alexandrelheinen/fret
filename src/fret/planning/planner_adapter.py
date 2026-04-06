"""Planner-agnostic adapter implementing the ARCO-FRET planning interface.

This module provides :class:`PlannerAdapter`, the single entry point for
motion planning requests in FRET.  The adapter validates incoming
:class:`PlanningRequest` dicts, delegates to a registered planner
implementation, enforces the configurable timeout, and returns a
:class:`PlanningResult` dict whose schema matches the contract defined in
``docs/arco/spec-integration-contract.md``.

**Design invariants:**

- The adapter is planner-agnostic.  Switching algorithms only requires
  changing the ``algorithm`` key in ``planner_config``; no adapter code
  changes are needed.
- All validation and schema enforcement live here, not in the planner.
- The adapter never raises on expected failure conditions (no-solution,
  timeout, stale occupancy).  All failures are represented as
  ``PlanningResult.status`` values and a ``failure_reason`` string.
- The ``state_validator`` callable decouples collision checking from the
  planning algorithm and from robot kinematics.  Callers supply the
  validator; the adapter and planner call it transparently.

**Supported algorithms:**

+------------------+------------------------------------------------------+
| ``algorithm``    | Implementation                                       |
+==================+======================================================+
| ``rrt_connect``  | :class:`~fret.planning.rrt_connect.RRTConnect`       |
+------------------+------------------------------------------------------+

**Diagnostics emitted:**

Every :class:`PlanningResult` contains:

- ``solve_time``     — wall-clock duration of the planning call (seconds).
- ``node_count``     — total tree nodes explored (0 when not applicable).
- ``failure_reason`` — human-readable string when planning failed.

See also:
    ``docs/arco/issue-05-arco-planner-adapter-in-fret.md`` —
    full specification and acceptance criteria.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from fret.perception.occupancy_adapter import OccupancyAdapter
from fret.planning.rrt_connect import RRTConnect

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

CANONICAL_FRAME = "world"

#: Algorithm identifiers supported by :class:`PlannerAdapter`.
SUPPORTED_ALGORITHMS = frozenset({"rrt_connect"})

#: Default planner configuration (mirrors ``config/planner.yaml``).
DEFAULT_CONFIG: Dict[str, Any] = {
    "algorithm": "rrt_connect",
    "default_timeout": 5.0,
    "rrt_connect": {
        "step_size": 0.05,
        "max_iterations": 10_000,
        "goal_bias": 0.1,
        "rng_seed": None,
    },
}


# ---------------------------------------------------------------------------
# PlannerAdapter
# ---------------------------------------------------------------------------


class PlannerAdapter:
    """Planner-agnostic adapter for ARCO-FRET motion planning.

    Validates :ref:`PlanningRequest <spec-integration-contract>` dicts,
    delegates to the selected planner, enforces timeouts, and returns
    :ref:`PlanningResult <spec-integration-contract>` dicts.

    Args:
        occupancy_adapter: :class:`~fret.perception.occupancy_adapter.OccupancyAdapter`
            holding the current obstacle scene.  Used to detect stale
            occupancy and optionally by the ``state_validator``.
        joint_limits: Per-joint ``(lower, upper)`` bounds in radians
            (revolute) or meters (prismatic).  Length must equal the robot
            DOF declared in planning requests.
        state_validator: Callable ``(joint_positions: list[float]) -> bool``
            that returns ``True`` when a configuration is collision-free.
            If ``None``, a free-space validator (always ``True``) is used.
            **Important**: for real planning the caller must supply a
            validator that uses the occupancy adapter and forward
            kinematics; the free-space default is for testing only.
        config: Planner configuration dict.  Missing keys fall back to
            :data:`DEFAULT_CONFIG`.  The ``rrt_connect`` sub-dict is
            forwarded verbatim to :class:`~fret.planning.rrt_connect.RRTConnect`.

    Raises:
        ValueError: If ``joint_limits`` is empty or contains invalid bounds.

    Example::

        import math
        from fret.perception import OccupancyAdapter
        from fret.planning import PlannerAdapter

        occupancy = OccupancyAdapter(inflation_radius=0.05)
        occupancy.update(obstacle_points)

        def validator(q):
            # For a real robot, apply FK and check all arm links.
            ee = forward_kinematics(q)
            return occupancy.is_free(ee)

        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=[(-math.pi, math.pi)] * 4,
            state_validator=validator,
        )

        result = adapter.plan(planning_request)
        if result["status"] == "success":
            print(f"Path: {len(result['path'])} waypoints, "
                  f"solved in {result['solve_time']:.3f} s")
    """

    def __init__(
        self,
        occupancy_adapter: OccupancyAdapter,
        joint_limits: List[Tuple[float, float]],
        state_validator: Optional[Callable[[List[float]], bool]] = None,
        config: Optional[Dict[str, Any]] = None,
    ) -> None:
        if not joint_limits:
            raise ValueError("joint_limits must be non-empty")
        for i, (lo, hi) in enumerate(joint_limits):
            if lo >= hi:
                raise ValueError(
                    f"joint_limits[{i}]: lower bound {lo} must be < "
                    f"upper bound {hi}"
                )

        self._occupancy = occupancy_adapter
        self._joint_limits = list(joint_limits)
        self._dof = len(joint_limits)

        # Merge caller config with defaults.
        merged: Dict[str, Any] = _deep_merge(DEFAULT_CONFIG, config or {})
        self._config = merged

        # Use caller validator or fall back to free-space.
        self._state_validator: Callable[[List[float]], bool] = (
            state_validator
            if state_validator is not None
            else _free_space_validator
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def occupancy_adapter(self) -> OccupancyAdapter:
        """The occupancy adapter used by this planner adapter."""
        return self._occupancy

    @property
    def joint_limits(self) -> List[Tuple[float, float]]:
        """Per-joint ``(lower, upper)`` bounds."""
        return list(self._joint_limits)

    @property
    def dof(self) -> int:
        """Robot degrees of freedom."""
        return self._dof

    @property
    def config(self) -> Dict[str, Any]:
        """Resolved planner configuration (merged with defaults)."""
        return dict(self._config)

    # ------------------------------------------------------------------
    # Main planning entry point
    # ------------------------------------------------------------------

    def plan(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a planning request and return a PlanningResult.

        Validates the request, checks occupancy freshness, runs the
        selected planner within the configured timeout, and packages the
        result with diagnostics.

        Args:
            request: ``PlanningRequest`` dict matching the schema in
                ``docs/arco/spec-integration-contract.md``.  Required
                fields: ``request_id``, ``start_joint_positions``,
                ``goal_joint_positions``, ``joint_count``,
                ``occupancy_stamp``, ``planner_config``
                (must contain ``algorithm``), ``reference_frame``.
                ``timeout`` is optional; when omitted the adapter uses
                ``config["default_timeout"]`` (default: 5.0 s).
                ``occupancy_stamp`` must be a POSIX timestamp
                (``time.time()`` clock source).

        Returns:
            ``PlanningResult`` dict with fields: ``request_id``,
            ``status``, ``path``, ``waypoint_count``, ``solve_time``,
            ``node_count``, ``failure_reason``, ``reference_frame``.

        Raises:
            TypeError: If ``request`` is not a dict.
        """
        if not isinstance(request, dict):
            raise TypeError(
                f"request must be a dict, got {type(request).__name__}"
            )

        t_start = time.monotonic()

        # Validate request schema and semantics.
        invalid_reason = self._validate_request(request)
        if invalid_reason is not None:
            return self._result(
                request_id=request.get("request_id", ""),
                status="invalid_request",
                path=None,
                node_count=0,
                solve_time=time.monotonic() - t_start,
                failure_reason=invalid_reason,
                reference_frame=request.get(
                    "reference_frame", CANONICAL_FRAME
                ),
            )

        request_id: str = request["request_id"]
        start: List[float] = request["start_joint_positions"]
        goal: List[float] = request["goal_joint_positions"]
        # Use request timeout if provided; fall back to config default_timeout.
        timeout: float = float(
            request.get(
                "timeout",
                self._config.get("default_timeout", 5.0),
            )
        )
        planner_cfg: Dict[str, Any] = request["planner_config"]
        algorithm: str = planner_cfg.get("algorithm", "rrt_connect")

        # Check occupancy freshness.
        # occupancy_stamp must be a POSIX timestamp (time.time()) per the
        # integration contract (spec-integration-contract.md Section 3.1).
        occupancy_stamp: float = request["occupancy_stamp"]
        max_age: float = self._config.get("max_occupancy_age", 2.0)
        occupancy_age = time.time() - occupancy_stamp
        if occupancy_age > max_age:
            return self._result(
                request_id=request_id,
                status="invalid_request",
                path=None,
                node_count=0,
                solve_time=time.monotonic() - t_start,
                failure_reason=(
                    f"stale_occupancy: age {occupancy_age:.2f} s "
                    f"exceeds max_occupancy_age {max_age:.2f} s"
                ),
                reference_frame=CANONICAL_FRAME,
            )

        # Dispatch to the selected algorithm.
        if algorithm not in SUPPORTED_ALGORITHMS:
            return self._result(
                request_id=request_id,
                status="invalid_request",
                path=None,
                node_count=0,
                solve_time=time.monotonic() - t_start,
                failure_reason=(
                    f"unsupported algorithm '{algorithm}'; "
                    f"supported: {sorted(SUPPORTED_ALGORITHMS)}"
                ),
                reference_frame=CANONICAL_FRAME,
            )

        path, node_count, fail_reason = self._run_planner(
            algorithm=algorithm,
            start=start,
            goal=goal,
            timeout=timeout,
            planner_cfg=planner_cfg,
        )

        solve_time = time.monotonic() - t_start

        if path is not None:
            return self._result(
                request_id=request_id,
                status="success",
                path=path,
                node_count=node_count,
                solve_time=solve_time,
                failure_reason=None,
                reference_frame=CANONICAL_FRAME,
            )

        # Map internal failure reason to PlanningResult status.
        if fail_reason == "timeout":
            status = "timeout"
        else:
            status = "no_plan_found"

        return self._result(
            request_id=request_id,
            status=status,
            path=None,
            node_count=node_count,
            solve_time=solve_time,
            failure_reason=fail_reason,
            reference_frame=CANONICAL_FRAME,
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _validate_request(self, request: Dict[str, Any]) -> Optional[str]:
        """Validate the PlanningRequest dict.

        Args:
            request: Candidate PlanningRequest dict.

        Returns:
            ``None`` if valid, or a string describing the first violation.
        """
        required_fields = [
            "request_id",
            "start_joint_positions",
            "goal_joint_positions",
            "joint_count",
            "occupancy_stamp",
            "planner_config",
            "reference_frame",
        ]
        for field in required_fields:
            if field not in request:
                return f"missing required field: '{field}'"

        if not request["request_id"]:
            return "request_id must be non-empty"

        if request["reference_frame"] != CANONICAL_FRAME:
            return (
                f"reference_frame must be '{CANONICAL_FRAME}', "
                f"got '{request['reference_frame']}'"
            )

        joint_count: int = request["joint_count"]
        if joint_count != self._dof:
            return f"joint_count {joint_count} != adapter DOF {self._dof}"

        start = request["start_joint_positions"]
        goal = request["goal_joint_positions"]
        if len(start) != joint_count:
            return (
                f"len(start_joint_positions) {len(start)} != "
                f"joint_count {joint_count}"
            )
        if len(goal) != joint_count:
            return (
                f"len(goal_joint_positions) {len(goal)} != "
                f"joint_count {joint_count}"
            )

        # timeout is optional; if present it must be a positive number.
        if "timeout" in request:
            timeout = request["timeout"]
            if not isinstance(timeout, (int, float)) or timeout <= 0:
                return f"timeout must be a positive number, got {timeout!r}"

        planner_cfg = request["planner_config"]
        if not isinstance(planner_cfg, dict):
            return "planner_config must be a dict"
        if "algorithm" not in planner_cfg:
            return "planner_config must contain 'algorithm'"

        return None

    def _run_planner(
        self,
        algorithm: str,
        start: List[float],
        goal: List[float],
        timeout: float,
        planner_cfg: Dict[str, Any],
    ) -> Tuple[Optional[List[List[float]]], int, Optional[str]]:
        """Instantiate and run the selected planner.

        Args:
            algorithm: Algorithm identifier (e.g. ``"rrt_connect"``).
            start: Start joint configuration.
            goal: Goal joint configuration.
            timeout: Maximum planning time in seconds.
            planner_cfg: Caller-supplied planner configuration dict.
                Values override the adapter's own ``config`` defaults.

        Returns:
            Tuple of ``(path, node_count, failure_reason)``.
        """
        if algorithm == "rrt_connect":
            return self._run_rrt_connect(start, goal, timeout, planner_cfg)
        # Unreachable: algorithm is checked before _run_planner is called.
        return (
            None,
            0,
            f"unsupported algorithm '{algorithm}'",
        )  # pragma: no cover

    def _run_rrt_connect(
        self,
        start: List[float],
        goal: List[float],
        timeout: float,
        planner_cfg: Dict[str, Any],
    ) -> Tuple[Optional[List[List[float]]], int, Optional[str]]:
        """Run the RRT-Connect planner.

        Args:
            start: Start joint configuration.
            goal: Goal joint configuration.
            timeout: Maximum planning time in seconds.
            planner_cfg: Caller-supplied planner config dict.  The
                ``rrt_connect`` sub-dict overrides adapter defaults.

        Returns:
            Tuple of ``(path, node_count, failure_reason)``.
        """
        # Merge adapter defaults with caller-supplied rrt_connect config.
        default_rrt: Dict[str, Any] = self._config.get("rrt_connect", {})
        caller_rrt: Dict[str, Any] = planner_cfg.get("rrt_connect", {})
        rrt_cfg: Dict[str, Any] = {**default_rrt, **caller_rrt}

        planner = RRTConnect(
            joint_limits=self._joint_limits,
            state_validator=self._state_validator,
            step_size=float(rrt_cfg.get("step_size", 0.05)),
            max_iterations=int(rrt_cfg.get("max_iterations", 10_000)),
            goal_bias=float(rrt_cfg.get("goal_bias", 0.1)),
            rng_seed=rrt_cfg.get("rng_seed"),
        )
        return planner.plan(start, goal, timeout)

    @staticmethod
    def _result(
        *,
        request_id: str,
        status: str,
        path: Optional[List[List[float]]],
        node_count: int,
        solve_time: float,
        failure_reason: Optional[str],
        reference_frame: str,
    ) -> Dict[str, Any]:
        """Assemble a PlanningResult dict.

        Args:
            request_id: Echoed from the originating request.
            status: One of ``"success"``, ``"no_plan_found"``,
                ``"timeout"``, ``"invalid_request"``.
            path: List of waypoints or ``None``.
            node_count: Nodes explored by the planner.
            solve_time: Wall-clock planning duration in seconds.
            failure_reason: Human-readable reason string or ``None``.
            reference_frame: Coordinate frame (always ``"world"``).

        Returns:
            Validated PlanningResult dict.
        """
        waypoint_count = len(path) if path is not None else 0
        return {
            "request_id": request_id,
            "status": status,
            "path": path,
            "waypoint_count": waypoint_count,
            "solve_time": solve_time,
            "node_count": node_count,
            "failure_reason": failure_reason,
            "reference_frame": reference_frame,
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _free_space_validator(joint_positions: List[float]) -> bool:
    """Default state validator: always returns ``True`` (no obstacles).

    Args:
        joint_positions: Joint configuration (unused).

    Returns:
        Always ``True``.
    """
    return True


def _deep_merge(
    base: Dict[str, Any], override: Dict[str, Any]
) -> Dict[str, Any]:
    """Recursively merge ``override`` into ``base``.

    Nested dicts are merged recursively; all other values in ``override``
    replace the corresponding values in ``base``.

    Args:
        base: Default configuration dict.
        override: Caller-supplied overrides.

    Returns:
        Merged dict (a new object; ``base`` is not mutated).
    """
    result = dict(base)
    for key, val in override.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(val, dict)
        ):
            result[key] = _deep_merge(result[key], val)
        else:
            result[key] = val
    return result
