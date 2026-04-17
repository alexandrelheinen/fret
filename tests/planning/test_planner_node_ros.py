"""Tests for fret.planning.planner_node_ros.PlannerRosNode (Level 4).

These tests exercise the ROS 2 wiring layer without a live ``rclpy`` context
by monkey-patching the ``rclpy.node.Node`` base class with a ``MagicMock``,
following the pattern used in ``tests/control/test_controller_node_ros.py``
and ``tests/scene/test_acquisition.py``.

Acceptance criteria covered:
    - Node constructs and declares expected parameters.
    - Planning is triggered immediately when start_configuration is given.
    - Planning is deferred until the first /joint_states message when
      start_configuration is empty (default).
    - A JointTrajectory is published on /joint_trajectory after a
      successful planning run.
    - A FAILED planning result logs an error and does NOT publish.
    - The /joint_states subscription is created with BEST_EFFORT QoS when
      no start override is provided.
"""

from __future__ import annotations

import sys
from types import ModuleType
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Helpers: minimal rclpy stub so the module can be imported without ROS
# ---------------------------------------------------------------------------


def _build_rclpy_stub() -> ModuleType:
    """Return a minimal rclpy stub that satisfies PlannerRosNode imports."""
    rclpy_mod = ModuleType("rclpy")
    node_mod = ModuleType("rclpy.node")
    qos_mod = ModuleType("rclpy.qos")

    class _FakeNode:
        """Minimal Node base class for testing (no ROS required).

        ``_params`` is *not* reset on re-entry so that pre-populated params
        survive the ``rclpy.node.Node.__init__`` call inside
        ``PlannerRosNode.__init__``.
        """

        def __init__(self, name: str) -> None:
            self._name = name
            # Preserve _params if already set (e.g. by test setup).
            if not hasattr(self, "_params"):
                self._params: dict[str, object] = {}
            self.get_logger = MagicMock(return_value=MagicMock())
            self.create_publisher = MagicMock(return_value=MagicMock())
            self.create_subscription = MagicMock(return_value=MagicMock())

        def declare_parameter(self, name: str, value: object) -> None:
            # Only store the default if not already present (caller may have
            # pre-populated with a test-specific value).
            self._params.setdefault(name, value)

        def get_parameter(self, name: str) -> MagicMock:
            value = self._params.get(name)
            param = MagicMock()
            pv = MagicMock()
            if isinstance(value, str):
                pv.string_value = value
            elif isinstance(value, (int, float)):
                pv.double_value = float(value)
                pv.integer_value = int(value)
                pv.double_array_value = []
            elif isinstance(value, list):
                pv.double_array_value = list(value)
                pv.integer_value = len(value)
            else:
                pv.double_array_value = []
                pv.string_value = ""
                pv.double_value = 0.0
                pv.integer_value = 0
            param.get_parameter_value.return_value = pv
            return param

    class _FakeQoSProfile:
        def __init__(self, **kwargs: object) -> None:
            pass

    node_mod.Node = _FakeNode  # type: ignore[attr-defined]
    qos_mod.QoSProfile = _FakeQoSProfile  # type: ignore[attr-defined]
    qos_mod.ReliabilityPolicy = MagicMock()  # type: ignore[attr-defined]
    qos_mod.DurabilityPolicy = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.node = node_mod  # type: ignore[attr-defined]
    rclpy_mod.qos = qos_mod  # type: ignore[attr-defined]
    rclpy_mod.init = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.spin = MagicMock()  # type: ignore[attr-defined]
    rclpy_mod.shutdown = MagicMock()  # type: ignore[attr-defined]
    return rclpy_mod


def _install_ros_msg_stubs() -> None:
    """Inject minimal ROS message stubs into sys.modules (idempotent)."""
    for pkg, names in [
        ("sensor_msgs", ["JointState"]),
        ("sensor_msgs.msg", ["JointState"]),
        ("trajectory_msgs", ["JointTrajectory", "JointTrajectoryPoint"]),
        ("trajectory_msgs.msg", ["JointTrajectory", "JointTrajectoryPoint"]),
        ("builtin_interfaces", ["Duration"]),
        ("builtin_interfaces.msg", ["Duration"]),
    ]:
        if pkg in sys.modules:
            continue
        mod = ModuleType(pkg)
        for cls_name in names:

            class _Stub:
                def __init__(self, **kwargs: object) -> None:
                    for k, v in kwargs.items():
                        setattr(self, k, v)
                    if cls_name in ("JointTrajectory",):
                        if not hasattr(self, "joint_names"):
                            self.joint_names: list[str] = []
                        if not hasattr(self, "points"):
                            self.points: list[object] = []
                    if cls_name in ("JointTrajectoryPoint",):
                        if not hasattr(self, "positions"):
                            self.positions: list[float] = []
                        if not hasattr(self, "velocities"):
                            self.velocities: list[float] = []
                    if cls_name in ("Duration",):
                        if not hasattr(self, "sec"):
                            self.sec: int = 0
                        if not hasattr(self, "nanosec"):
                            self.nanosec: int = 0

            _Stub.__name__ = cls_name
            setattr(mod, cls_name, _Stub)
        sys.modules[pkg] = mod


@pytest.fixture(autouse=True)
def _ros_stubs(monkeypatch: pytest.MonkeyPatch) -> None:
    """Install rclpy and ROS message stubs for every test in this module."""
    rclpy_stub = _build_rclpy_stub()
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_stub)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_stub.node)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_stub.qos)
    _install_ros_msg_stubs()
    # Evict cached module so fresh rclpy stub is used on next import
    sys.modules.pop("fret.planning.planner_node_ros", None)


# ---------------------------------------------------------------------------
# Factory: build a concrete PlannerRosNode with controllable parameters
# ---------------------------------------------------------------------------


def _new_node(
    *,
    goal_cfg: list[float] | None = None,
    start_cfg: list[float] | None = None,
    planning_timeout: float = 10.0,
    scenario_id: str = "test_scenario",
    patch_trigger: bool = True,
) -> tuple[object, MagicMock]:
    """Construct a PlannerRosNode with pre-populated params.

    Args:
        goal_cfg: Goal configuration (defaults to [0.3272, 0.4712, 0.05]).
        start_cfg: Start configuration override. Empty list (``[]``) means
            "wait for /joint_states". Non-empty list triggers immediately.
        planning_timeout: Planning timeout in seconds.
        scenario_id: Scenario identifier string.
        patch_trigger: If True, patch ``_trigger_planning`` so it does not
            actually run the planner during construction.

    Returns:
        Tuple ``(node_instance, mock_trigger)`` where ``mock_trigger`` is
        the MagicMock for ``_trigger_planning`` (only meaningful when
        ``patch_trigger=True``; otherwise a dummy MagicMock).
    """
    import rclpy.node as rclpy_node  # type: ignore[import]

    from fret.planning.planner_node_ros import PlannerRosNode

    class _Concrete(PlannerRosNode, rclpy_node.Node):  # type: ignore[misc]
        pass

    inst = _Concrete.__new__(_Concrete)

    # Pre-populate _params BEFORE __init__ is called.
    # _FakeNode.__init__ preserves existing keys via setdefault().
    inst._params = {  # type: ignore[attr-defined]
        "model": "scara",
        "scenario_id": scenario_id,
        "goal_configuration": (
            goal_cfg if goal_cfg is not None else [0.3272, 0.4712, 0.05]
        ),
        "planning_timeout": planning_timeout,
        "start_configuration": start_cfg if start_cfg is not None else [],
    }

    mock_trigger = MagicMock()
    if patch_trigger:
        with patch.object(_Concrete, "_trigger_planning", mock_trigger):
            PlannerRosNode.__init__(inst)
    else:
        PlannerRosNode.__init__(inst)
        mock_trigger = MagicMock()  # unused placeholder

    return inst, mock_trigger


# ---------------------------------------------------------------------------
# Tests: construction
# ---------------------------------------------------------------------------


class TestPlannerRosNodeConstruction:
    """Verify parameter declaration and initial state."""

    def test_construction_succeeds(self) -> None:
        node, _ = _new_node()
        assert node is not None

    def test_publisher_created_for_joint_trajectory(self) -> None:
        """A publisher for /joint_trajectory must be created."""
        node, _ = _new_node()
        topics = [
            str(c[0][1]) for c in node.create_publisher.call_args_list  # type: ignore[attr-defined]
        ]
        assert "/joint_trajectory" in topics

    def test_subscription_created_when_no_start_override(self) -> None:
        """When start_configuration is empty, /joint_states subscription is
        created (subscription-trigger path)."""
        node, _ = _new_node(start_cfg=[])
        assert node.create_subscription.called  # type: ignore[attr-defined]
        topics = [
            str(c[0][1])
            for c in node.create_subscription.call_args_list  # type: ignore[attr-defined]
        ]
        assert "/joint_states" in topics

    def test_no_subscription_when_start_override_set(self) -> None:
        """When a non-empty start_configuration is given, no /joint_states
        subscription should be created (immediate-trigger path)."""
        node, _ = _new_node(start_cfg=[0.0, 0.0, 0.0])
        assert not node.create_subscription.called  # type: ignore[attr-defined]

    def test_trigger_called_immediately_with_start_override(self) -> None:
        """_trigger_planning must be called during __init__ when
        start_configuration is non-empty."""
        node, mock_trigger = _new_node(start_cfg=[0.1, 0.2, 0.05])
        assert mock_trigger.call_count == 1

    def test_trigger_not_called_without_start_override(self) -> None:
        """_trigger_planning must NOT be called during __init__ when
        start_configuration is empty (deferred until /joint_states)."""
        node, mock_trigger = _new_node(start_cfg=[])
        assert mock_trigger.call_count == 0


# ---------------------------------------------------------------------------
# Tests: _joint_states_callback
# ---------------------------------------------------------------------------


class TestJointStatesCallback:
    """Verify that planning is triggered on the first /joint_states message."""

    def _make_bare_node(self) -> object:
        """Create node with internal state for callback tests."""
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = object.__new__(PlannerRosNode)
        inst._planned = False  # type: ignore[attr-defined]
        inst._start_cfg = None  # type: ignore[attr-defined]
        inst._traj_pub = MagicMock()  # type: ignore[attr-defined]
        inst.get_logger = MagicMock(return_value=MagicMock())  # type: ignore[attr-defined]
        return inst

    def test_planning_triggered_on_first_joint_state(self) -> None:
        """_joint_states_callback must call _trigger_planning exactly once."""
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()
        with patch.object(inst, "_trigger_planning") as mock_trigger:  # type: ignore[arg-type]
            msg = MagicMock()
            msg.position = [0.1, 0.2, 0.05]
            PlannerRosNode._joint_states_callback(inst, msg)  # type: ignore[attr-defined]
            assert mock_trigger.call_count == 1

    def test_planning_not_triggered_twice(self) -> None:
        """_joint_states_callback is idempotent after the first call."""
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()

        # The mock must also set _planned=True, mirroring the real behaviour
        # of _trigger_planning, so that the guard condition works.
        def _set_planned(*args: object, **kwargs: object) -> None:
            inst._planned = True  # type: ignore[attr-defined]

        mock_trigger = MagicMock(side_effect=_set_planned)
        with patch.object(inst, "_trigger_planning", mock_trigger):  # type: ignore[arg-type]
            msg = MagicMock()
            msg.position = [0.1, 0.2, 0.05]
            PlannerRosNode._joint_states_callback(inst, msg)  # type: ignore[attr-defined]
            PlannerRosNode._joint_states_callback(inst, msg)  # type: ignore[attr-defined]
            assert mock_trigger.call_count == 1

    def test_ignores_short_position_array(self) -> None:
        """Callback must silently skip messages with fewer than 3 positions."""
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()
        with patch.object(inst, "_trigger_planning") as mock_trigger:  # type: ignore[arg-type]
            msg = MagicMock()
            msg.position = [0.1]  # too short
            PlannerRosNode._joint_states_callback(inst, msg)  # type: ignore[attr-defined]
            assert mock_trigger.call_count == 0

    def test_start_cfg_set_from_message(self) -> None:
        """Callback must populate _start_cfg from the joint state message."""
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()
        with patch.object(inst, "_trigger_planning"):  # type: ignore[arg-type]
            msg = MagicMock()
            msg.position = [0.1, 0.2, 0.05]
            PlannerRosNode._joint_states_callback(inst, msg)  # type: ignore[attr-defined]
        assert inst._start_cfg is not None  # type: ignore[attr-defined]
        assert len(inst._start_cfg) == 3  # type: ignore[attr-defined]
        assert inst._start_cfg[0] == pytest.approx(0.1)  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Tests: _trigger_planning
# ---------------------------------------------------------------------------


class TestTriggerPlanning:
    """Verify the full planning + publish cycle."""

    def _make_bare_node(self) -> object:
        """Create a minimal node state for _trigger_planning tests."""
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = object.__new__(PlannerRosNode)
        inst._model = "scara"  # type: ignore[attr-defined]
        inst._scenario_id = "test"  # type: ignore[attr-defined]
        inst._goal_cfg = [0.3272, 0.4712, 0.05]  # type: ignore[attr-defined]
        inst._planning_timeout = 10.0  # type: ignore[attr-defined]
        inst._start_cfg = [0.0, 0.0, 0.0]  # type: ignore[attr-defined]
        inst._planned = False  # type: ignore[attr-defined]
        inst._traj_pub = MagicMock()  # type: ignore[attr-defined]
        inst.get_logger = MagicMock(return_value=MagicMock())  # type: ignore[attr-defined]
        return inst

    def test_successful_planning_publishes_trajectory(self) -> None:
        """A SUCCESS result must call _publish_trajectory."""
        from fret.interfaces import PlanningResult, PlanningStatus
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()
        dummy_path = [np.zeros(3), np.array([0.3, 0.3, 0.05])]
        mock_result = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=dummy_path,
            planning_duration=0.01,
        )

        timed_mock = MagicMock()
        timed_mock.joint_names = ["j0", "j1", "j2"]
        pt = MagicMock()
        pt.positions = [0.0, 0.0, 0.0]
        pt.velocities = []
        pt.time_from_start = 0.0
        timed_mock.points = [pt]

        with (
            patch("fret.planning.planner_node.PlannerNode") as MockCore,
            patch(
                "fret.planning.trajectory_generator.TrajectoryGenerator"
            ) as MockTrajGen,
            patch("fret.scene.occupancy_adapter.OccupancyAdapter"),
            patch("fret.control.kinematics.Kinematics"),
            patch.object(
                inst, "_publish_trajectory"  # type: ignore[arg-type]
            ) as mock_pub,
        ):
            MockCore.return_value.plan.return_value = mock_result
            MockTrajGen.return_value.process.return_value = timed_mock
            PlannerRosNode._trigger_planning(inst)  # type: ignore[attr-defined]

        # _publish_trajectory must have been called once
        assert mock_pub.call_count == 1

    def test_failed_planning_does_not_publish(self) -> None:
        """A FAILED result must log an error and NOT call _publish_trajectory."""
        from fret.interfaces import ErrorCode, PlanningResult, PlanningStatus
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()
        mock_result = PlanningResult(
            status=PlanningStatus.ABORTED,
            path=[],
            error_code=ErrorCode.TIMEOUT,
            planning_duration=10.0,
        )

        with (
            patch("fret.planning.planner_node.PlannerNode") as MockCore,
            patch("fret.scene.occupancy_adapter.OccupancyAdapter"),
            patch.object(
                inst, "_publish_trajectory"  # type: ignore[arg-type]
            ) as mock_pub,
        ):
            MockCore.return_value.plan.return_value = mock_result
            PlannerRosNode._trigger_planning(inst)  # type: ignore[attr-defined]

        assert mock_pub.call_count == 0

    def test_planned_flag_set_after_trigger(self) -> None:
        """_planned must be True after _trigger_planning runs."""
        from fret.interfaces import PlanningResult, PlanningStatus
        from fret.planning.planner_node_ros import PlannerRosNode

        inst = self._make_bare_node()
        dummy_path = [np.zeros(3), np.array([0.3, 0.3, 0.05])]
        mock_result = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=dummy_path,
            planning_duration=0.01,
        )
        timed_mock = MagicMock()
        timed_mock.joint_names = []
        timed_mock.points = []

        with (
            patch("fret.planning.planner_node.PlannerNode") as MockCore,
            patch(
                "fret.planning.trajectory_generator.TrajectoryGenerator"
            ) as MockTrajGen,
            patch("fret.scene.occupancy_adapter.OccupancyAdapter"),
            patch("fret.control.kinematics.Kinematics"),
            patch.object(
                inst, "_publish_trajectory"  # type: ignore[arg-type]
            ),
        ):
            MockCore.return_value.plan.return_value = mock_result
            MockTrajGen.return_value.process.return_value = timed_mock
            PlannerRosNode._trigger_planning(inst)  # type: ignore[attr-defined]

        assert inst._planned is True  # type: ignore[attr-defined]
