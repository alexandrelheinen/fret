"""Bidirectional RRT-Connect planner for continuous joint spaces.

Implements the RRT-Connect algorithm (Kuffner & LaValle, 2000) for
collision-aware path planning in joint space.  The planner is suitable for
obstacle-rich environments and operates entirely in pure Python with no
external dependencies.

**Algorithm outline:**

Two trees grow simultaneously — one rooted at the start configuration and one
at the goal.  At each iteration the algorithm extends the first tree toward a
random sample, then tries to connect the second tree greedily to the new node.
The trees are swapped each iteration so both grow at approximately the same
rate.  When the trees are connected, the path is extracted, reversed where
needed, and returned.

**Collision model:**

All collision queries are delegated to a caller-supplied ``state_validator``
callable with signature ``(joint_positions: list[float]) -> bool``.  Returning
``True`` means the configuration is collision-free; ``False`` means it is in
collision.  The planner never queries the occupancy model directly, which keeps
the algorithm robot-agnostic and fully testable with mock validators.

**Timeout policy:**

The planner checks a deadline (``time.monotonic()`` stamp) at the start of
every iteration.  When the deadline is exceeded the planner stops and reports
``"timeout"`` as the failure reason.  The check has sub-millisecond overhead
and does not require threads or signals.

**Path smoothing:**

After a raw RRT-Connect path is found, a greedy shortcut pass removes
redundant waypoints.  Each shortcut is validated with the same collision
checker used during tree building (interpolated segment check).

See also:
    ``docs/arco/issue-05-arco-planner-adapter-in-fret.md`` —
    full specification and acceptance criteria.
"""

from __future__ import annotations

import math
import random
import time
from typing import Callable, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Internal tree structure
# ---------------------------------------------------------------------------


class _Tree:
    """Binary tree used internally by RRT-Connect.

    Each node stores a joint configuration and a reference to its parent.
    The root node has parent index ``-1``.

    Attributes:
        nodes: List of joint configuration arrays, one per node.
        parents: Parent index for each node; ``-1`` for the root.
    """

    __slots__ = ("nodes", "parents")

    def __init__(self, root: List[float]) -> None:
        self.nodes: List[List[float]] = [list(root)]
        self.parents: List[int] = [-1]

    def add(self, config: List[float], parent_idx: int) -> int:
        """Add a node and return its index.

        Args:
            config: Joint configuration for the new node.
            parent_idx: Index of the parent node in :attr:`nodes`.

        Returns:
            Index of the newly added node.
        """
        self.nodes.append(list(config))
        self.parents.append(parent_idx)
        return len(self.nodes) - 1

    def nearest(self, config: List[float]) -> Tuple[int, List[float]]:
        """Find the nearest node by squared Euclidean distance in joint space.

        Args:
            config: Query configuration.

        Returns:
            Tuple of ``(index, node_config)`` for the nearest node.
        """
        best_idx = 0
        best_sq = math.inf
        for i, node in enumerate(self.nodes):
            sq = sum((a - b) * (a - b) for a, b in zip(node, config))
            if sq < best_sq:
                best_sq = sq
                best_idx = i
        return best_idx, self.nodes[best_idx]

    def path_to_root(self, idx: int) -> List[List[float]]:
        """Extract the path from ``idx`` to the root, in root-first order.

        Args:
            idx: Leaf node index.

        Returns:
            Ordered list of configurations from root to the given node.
        """
        path: List[List[float]] = []
        current = idx
        while current != -1:
            path.append(self.nodes[current])
            current = self.parents[current]
        return list(reversed(path))

    @property
    def node_count(self) -> int:
        """Total number of nodes in the tree."""
        return len(self.nodes)


# ---------------------------------------------------------------------------
# Distance helper
# ---------------------------------------------------------------------------


def _dist(a: List[float], b: List[float]) -> float:
    """Euclidean distance between two configurations."""
    return math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))


# ---------------------------------------------------------------------------
# RRTConnect planner
# ---------------------------------------------------------------------------


class RRTConnect:
    """Bidirectional RRT-Connect planner for continuous joint spaces.

    Suitable for obstacle-rich environments; requires no external libraries.

    Args:
        joint_limits: List of ``(lower, upper)`` bounds for each joint.
            Prismatic joints are in meters; revolute joints are in radians.
        state_validator: Callable ``(joint_positions) -> bool`` returning
            ``True`` when the configuration is collision-free.
        step_size: Maximum extension step in the same units as the joint
            space (uses the Euclidean norm across all joints).  Smaller
            values give finer collision resolution but more iterations.
        max_iterations: Upper bound on RRT-Connect iterations.  Each
            iteration performs one EXTEND and one CONNECT attempt.
        goal_bias: Probability of choosing the goal configuration as the
            random sample instead of a uniformly random sample.  Values
            in ``[0.0, 0.5]`` are typical.
        rng_seed: Integer seed for the internal random-number generator.
            ``None`` uses an unpredictable seed.

    Raises:
        ValueError: If any argument violates its constraints.

    Example::

        def validator(q):
            return not occupancy.is_occupied(fk(q))

        planner = RRTConnect(
            joint_limits=[(-math.pi, math.pi)] * 4,
            state_validator=validator,
        )
        path, node_count, fail_reason = planner.plan(start, goal, timeout=5.0)
        if path is not None:
            print(f"Found path with {len(path)} waypoints.")
    """

    def __init__(
        self,
        joint_limits: List[Tuple[float, float]],
        state_validator: Callable[[List[float]], bool],
        step_size: float = 0.05,
        max_iterations: int = 10_000,
        goal_bias: float = 0.1,
        rng_seed: Optional[int] = None,
    ) -> None:
        if not joint_limits:
            raise ValueError("joint_limits must be non-empty")
        for i, (lo, hi) in enumerate(joint_limits):
            if lo >= hi:
                raise ValueError(
                    f"joint_limits[{i}]: lower bound {lo} must be < upper "
                    f"bound {hi}"
                )
        if step_size <= 0.0:
            raise ValueError(f"step_size must be positive, got {step_size}")
        if max_iterations <= 0:
            raise ValueError(
                f"max_iterations must be positive, got {max_iterations}"
            )
        if not (0.0 <= goal_bias <= 1.0):
            raise ValueError(
                f"goal_bias must be in [0.0, 1.0], got {goal_bias}"
            )

        self._joint_limits = list(joint_limits)
        self._dof = len(joint_limits)
        self._state_validator = state_validator
        self._step_size = step_size
        self._max_iterations = max_iterations
        self._goal_bias = goal_bias
        self._rng = random.Random(rng_seed)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        start: List[float],
        goal: List[float],
        timeout: float,
    ) -> Tuple[Optional[List[List[float]]], int, Optional[str]]:
        """Find a collision-free path from start to goal.

        Args:
            start: Starting joint configuration (length equals DOF).
            goal: Goal joint configuration (length equals DOF).
            timeout: Maximum wall-clock time allowed in seconds.

        Returns:
            A three-tuple ``(path, node_count, failure_reason)``:

            - ``path`` is a list of waypoints (each a list of joint
              positions) from ``start`` to ``goal``, or ``None`` on
              failure.
            - ``node_count`` is the total number of nodes created across
              both trees.
            - ``failure_reason`` is ``None`` on success; otherwise a
              human-readable string (``"timeout"`` or ``"no_solution"``).

        Raises:
            ValueError: If ``start`` or ``goal`` length mismatches DOF,
                or if ``timeout`` is not positive.
        """
        if len(start) != self._dof:
            raise ValueError(f"start length {len(start)} != DOF {self._dof}")
        if len(goal) != self._dof:
            raise ValueError(f"goal length {len(goal)} != DOF {self._dof}")
        if timeout <= 0.0:
            raise ValueError(f"timeout must be positive, got {timeout}")

        # Validate start and goal configurations.
        if not self._state_validator(start):
            return None, 0, "start_in_collision"
        if not self._state_validator(goal):
            return None, 0, "goal_in_collision"

        # If start == goal (within step_size tolerance), return trivially.
        if _dist(start, goal) < self._step_size:
            return [list(start), list(goal)], 2, None

        deadline = time.monotonic() + timeout

        tree_a = _Tree(start)
        tree_b = _Tree(goal)
        # Track which tree is "start" tree for path extraction.
        a_is_start = True

        for _ in range(self._max_iterations):
            if time.monotonic() > deadline:
                total = tree_a.node_count + tree_b.node_count
                return None, total, "timeout"

            # Sample: goal_bias fraction of the time use the root of tree_b.
            if self._rng.random() < self._goal_bias:
                q_rand = list(tree_b.nodes[0])
            else:
                q_rand = self._random_config()

            # EXTEND tree_a toward q_rand.
            ext_status, ext_idx = self._extend(tree_a, q_rand)
            if ext_status != "trapped":
                q_new = tree_a.nodes[ext_idx]
                # CONNECT tree_b to the new node.
                conn_status, conn_idx = self._connect(tree_b, q_new)
                if conn_status == "reached":
                    # Trees are connected — extract path.
                    path_a = tree_a.path_to_root(ext_idx)
                    path_b = tree_b.path_to_root(conn_idx)
                    if a_is_start:
                        raw_path = path_a + list(reversed(path_b))
                    else:
                        raw_path = list(reversed(path_b)) + path_a
                    total = tree_a.node_count + tree_b.node_count
                    smoothed = self._smooth(raw_path)
                    return smoothed, total, None

            # Swap trees each iteration for balanced growth.
            tree_a, tree_b = tree_b, tree_a
            a_is_start = not a_is_start

        total = tree_a.node_count + tree_b.node_count
        return None, total, "no_solution"

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _random_config(self) -> List[float]:
        """Sample a uniformly random configuration within joint limits."""
        return [self._rng.uniform(lo, hi) for lo, hi in self._joint_limits]

    def _steer(
        self, q_near: List[float], q_target: List[float]
    ) -> List[float]:
        """Move from q_near toward q_target by at most step_size.

        Args:
            q_near: Current node configuration.
            q_target: Target configuration.

        Returns:
            New configuration at most ``step_size`` away from ``q_near``
            in the direction of ``q_target``.
        """
        d = _dist(q_near, q_target)
        if d <= self._step_size:
            return list(q_target)
        scale = self._step_size / d
        return [a + scale * (b - a) for a, b in zip(q_near, q_target)]

    def _extend(self, tree: _Tree, q: List[float]) -> Tuple[str, int]:
        """Extend the tree one step toward ``q``.

        Args:
            tree: Tree to extend.
            q: Target configuration.

        Returns:
            Tuple ``(status, node_index)`` where ``status`` is one of
            ``"reached"`` (``q_new == q``), ``"advanced"`` (moved
            toward ``q`` but not there yet), or ``"trapped"`` (new
            configuration in collision).  ``node_index`` is ``-1`` on
            ``"trapped"``.
        """
        near_idx, q_near = tree.nearest(q)
        q_new = self._steer(q_near, q)
        if not self._state_validator(q_new):
            return "trapped", -1
        new_idx = tree.add(q_new, near_idx)
        if _dist(q_new, q) < 1e-9:
            return "reached", new_idx
        return "advanced", new_idx

    def _connect(self, tree: _Tree, q: List[float]) -> Tuple[str, int]:
        """Greedily connect the tree to ``q``.

        Repeatedly calls :meth:`_extend` until the tree reaches ``q``
        or gets trapped.

        Args:
            tree: Tree to connect.
            q: Target configuration.

        Returns:
            Tuple ``(status, last_node_index)`` where ``status`` is
            ``"reached"`` or ``"trapped"``.
        """
        status = "advanced"
        last_idx = -1
        while status == "advanced":
            status, last_idx = self._extend(tree, q)
        return status, last_idx

    def _segment_valid(self, q1: List[float], q2: List[float]) -> bool:
        """Check whether the straight-line segment is collision-free.

        Samples the segment at ``step_size`` intervals and validates each
        intermediate configuration.

        Args:
            q1: Start configuration.
            q2: End configuration.

        Returns:
            ``True`` if all sampled configurations are valid.
        """
        d = _dist(q1, q2)
        # Minimum 2 steps ensures both q1 and q2 are validated even when d
        # is smaller than step_size (i.e. step_count would be 1 or 0).
        step_count = max(2, int(math.ceil(d / self._step_size)))
        for i in range(step_count + 1):
            t = i / step_count
            q = [a + t * (b - a) for a, b in zip(q1, q2)]
            if not self._state_validator(q):
                return False
        return True

    def _smooth(self, path: List[List[float]]) -> List[List[float]]:
        """Remove redundant waypoints via greedy shortcut smoothing.

        Iterates through the path and, for each waypoint, attempts to
        connect directly to the furthest reachable waypoint.  The result
        has fewer waypoints while remaining collision-free.

        Args:
            path: Raw path from RRT-Connect, ordered start to goal.

        Returns:
            Smoothed path with redundant waypoints removed.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Find the furthest waypoint directly reachable from path[i].
            j = len(path) - 1
            while j > i + 1:
                if self._segment_valid(path[i], path[j]):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed
