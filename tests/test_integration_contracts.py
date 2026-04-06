"""Tests validating ARCO-FRET integration contract assumptions.

These tests exercise the data contract defined in
docs/arco/spec-integration-contract.md and the frame/unit conventions defined
in docs/arco/spec-frames-and-units.md.

All tests are deterministic and use only in-process fixtures. No ROS runtime
or external dependencies are required.
"""

import math
import time
import unittest
import uuid


# ---------------------------------------------------------------------------
# Contract fixtures
# ---------------------------------------------------------------------------

ROBOT_DOF = 4
JOINT_LIMITS = [
    (-math.pi, math.pi),  # joint_1
    (-math.pi / 2, math.pi / 2),  # joint_2
    (-math.pi, math.pi),  # joint_3
    (-math.pi / 2, math.pi / 2),  # joint_4
]

# Default tolerance thresholds from spec
MAX_OCCUPANCY_AGE = 2.0  # seconds
MAX_TRACKING_ERROR = 0.1  # radians (RMS)
TRACKING_FAILURE_WINDOW = 0.5  # seconds
TF_LOOKUP_TIMEOUT = 0.5  # seconds
MIN_VOXEL_SIZE = 0.001  # meters
MAX_VOXEL_SIZE = 1.0  # meters
MAX_POINT_COUNT = 500_000

VALID_PLANNER_STATUSES = {"success", "no_plan_found", "timeout", "invalid_request"}
VALID_EXECUTION_STATUSES = {"executing", "succeeded", "tracking_failure", "aborted"}
VALID_OCCUPANCY_SOURCES = {"lidar", "depth_camera", "simulated"}
CANONICAL_FRAME = "world"


def _make_planning_request(
    *,
    request_id=None,
    start=None,
    goal=None,
    occupancy_stamp=None,
    timeout=5.0,
    algorithm="rrt_star",
    reference_frame=CANONICAL_FRAME,
):
    """Build a valid PlanningRequest fixture."""
    now = time.time()
    start = start or [0.0] * ROBOT_DOF
    goal = goal or [0.5] * ROBOT_DOF
    return {
        "request_id": request_id if request_id is not None else str(uuid.uuid4()),
        "start_joint_positions": start,
        "goal_joint_positions": goal,
        "joint_count": ROBOT_DOF,
        "occupancy_stamp": occupancy_stamp if occupancy_stamp is not None else now,
        "timeout": timeout,
        "planner_config": {"algorithm": algorithm},
        "reference_frame": reference_frame,
    }


def _make_planning_result(
    *,
    request_id=None,
    status="success",
    path=None,
    solve_time=0.1,
    node_count=42,
    failure_reason=None,
    reference_frame=CANONICAL_FRAME,
):
    """Build a valid PlanningResult fixture."""
    if path is None and status == "success":
        path = [[0.0] * ROBOT_DOF, [0.25] * ROBOT_DOF, [0.5] * ROBOT_DOF]
    waypoint_count = len(path) if path is not None else 0
    return {
        "request_id": request_id if request_id is not None else str(uuid.uuid4()),
        "status": status,
        "path": path,
        "waypoint_count": waypoint_count,
        "solve_time": solve_time,
        "node_count": node_count,
        "failure_reason": failure_reason,
        "reference_frame": reference_frame,
    }


def _make_occupancy_update(
    *,
    update_id=None,
    stamp=None,
    points=None,
    voxel_size=0.05,
    source="simulated",
    reference_frame=CANONICAL_FRAME,
):
    """Build a valid OccupancyUpdatePayload fixture."""
    if points is None:
        points = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
    return {
        "update_id": update_id if update_id is not None else str(uuid.uuid4()),
        "stamp": stamp if stamp is not None else time.time(),
        "reference_frame": reference_frame,
        "points": points,
        "point_count": len(points),
        "voxel_size": voxel_size,
        "source": source,
    }


def _make_execution_feedback(
    *,
    request_id=None,
    status="executing",
    stamp=None,
    progress=0.5,
    current_joint_positions=None,
    tracking_error=0.01,
    message=None,
):
    """Build a valid ExecutionFeedback fixture."""
    return {
        "request_id": request_id if request_id is not None else str(uuid.uuid4()),
        "status": status,
        "stamp": stamp if stamp is not None else time.time(),
        "progress": progress,
        "current_joint_positions": current_joint_positions or [0.0] * ROBOT_DOF,
        "tracking_error": tracking_error,
        "message": message,
    }


# ---------------------------------------------------------------------------
# Helper validators (mirrors contract rules)
# ---------------------------------------------------------------------------


def validate_planning_request(req):
    """Validate a PlanningRequest dict against contract invariants.

    Args:
        req: Dictionary representing a PlanningRequest payload.

    Raises:
        ValueError: If any invariant is violated.
    """
    required = [
        "request_id",
        "start_joint_positions",
        "goal_joint_positions",
        "joint_count",
        "occupancy_stamp",
        "timeout",
        "planner_config",
        "reference_frame",
    ]
    for field in required:
        if field not in req:
            raise ValueError(f"Missing required field: {field}")
    if not req["request_id"]:
        raise ValueError("request_id must be non-empty")
    if req["reference_frame"] != CANONICAL_FRAME:
        raise ValueError(
            f"reference_frame must be '{CANONICAL_FRAME}', "
            f"got '{req['reference_frame']}'"
        )
    if req["joint_count"] != len(req["start_joint_positions"]):
        raise ValueError("joint_count does not match len(start_joint_positions)")
    if req["joint_count"] != len(req["goal_joint_positions"]):
        raise ValueError("joint_count does not match len(goal_joint_positions)")
    if req["timeout"] <= 0:
        raise ValueError("timeout must be positive")
    if "algorithm" not in req["planner_config"]:
        raise ValueError("planner_config must contain 'algorithm'")


def validate_planning_result(result):
    """Validate a PlanningResult dict against contract invariants.

    Args:
        result: Dictionary representing a PlanningResult payload.

    Raises:
        ValueError: If any invariant is violated.
    """
    required = [
        "request_id",
        "status",
        "path",
        "waypoint_count",
        "solve_time",
        "node_count",
        "failure_reason",
        "reference_frame",
    ]
    for field in required:
        if field not in result:
            raise ValueError(f"Missing required field: {field}")
    if result["status"] not in VALID_PLANNER_STATUSES:
        raise ValueError(f"Invalid status: {result['status']}")
    if result["status"] == "success":
        if result["path"] is None:
            raise ValueError("path must not be null when status is 'success'")
        if result["waypoint_count"] != len(result["path"]):
            raise ValueError("waypoint_count does not match len(path)")
    else:
        if result["path"] is not None:
            raise ValueError("path must be null when status is not 'success'")
        if result["waypoint_count"] != 0:
            raise ValueError("waypoint_count must be 0 when status is not 'success'")
    if result["solve_time"] < 0:
        raise ValueError("solve_time must be non-negative")


def validate_occupancy_update(payload):
    """Validate an OccupancyUpdatePayload dict against contract invariants.

    Args:
        payload: Dictionary representing an OccupancyUpdatePayload.

    Raises:
        ValueError: If any invariant is violated.
    """
    required = [
        "update_id",
        "stamp",
        "reference_frame",
        "points",
        "point_count",
        "voxel_size",
        "source",
    ]
    for field in required:
        if field not in payload:
            raise ValueError(f"Missing required field: {field}")
    if not payload["update_id"]:
        raise ValueError("update_id must be non-empty")
    if payload["reference_frame"] != CANONICAL_FRAME:
        raise ValueError(
            f"reference_frame must be '{CANONICAL_FRAME}', "
            f"got '{payload['reference_frame']}'"
        )
    if payload["point_count"] != len(payload["points"]):
        raise ValueError("point_count does not match len(points)")
    if not (MIN_VOXEL_SIZE <= payload["voxel_size"] <= MAX_VOXEL_SIZE):
        raise ValueError(
            f"voxel_size {payload['voxel_size']} outside valid range "
            f"[{MIN_VOXEL_SIZE}, {MAX_VOXEL_SIZE}]"
        )
    if payload["source"] not in VALID_OCCUPANCY_SOURCES:
        raise ValueError(f"Invalid source: {payload['source']}")
    if payload["point_count"] > MAX_POINT_COUNT:
        raise ValueError(
            f"point_count {payload['point_count']} exceeds maximum {MAX_POINT_COUNT}"
        )
    for i, pt in enumerate(payload["points"]):
        if len(pt) != 3:
            raise ValueError(f"Point {i} must have exactly 3 coordinates")


def validate_execution_feedback(feedback):
    """Validate an ExecutionFeedback dict against contract invariants.

    Args:
        feedback: Dictionary representing an ExecutionFeedback payload.

    Raises:
        ValueError: If any invariant is violated.
    """
    required = [
        "request_id",
        "status",
        "stamp",
        "progress",
        "current_joint_positions",
        "tracking_error",
        "message",
    ]
    for field in required:
        if field not in feedback:
            raise ValueError(f"Missing required field: {field}")
    if feedback["status"] not in VALID_EXECUTION_STATUSES:
        raise ValueError(f"Invalid status: {feedback['status']}")
    if not (0.0 <= feedback["progress"] <= 1.0):
        raise ValueError(f"progress {feedback['progress']} outside [0.0, 1.0]")
    if feedback["tracking_error"] < 0:
        raise ValueError("tracking_error must be non-negative")
    if feedback["status"] not in ("executing",) and feedback["message"] is None:
        raise ValueError("message is required for terminal status values")


def check_occupancy_freshness(occupancy_stamp, planning_call_time):
    """Check whether an occupancy snapshot is fresh enough to plan with.

    Args:
        occupancy_stamp: POSIX float timestamp of the occupancy snapshot.
        planning_call_time: POSIX float timestamp of the planning call.

    Returns:
        True if the snapshot is within MAX_OCCUPANCY_AGE; False otherwise.
    """
    return (planning_call_time - occupancy_stamp) <= MAX_OCCUPANCY_AGE


def check_tracking_failure(tracking_error, duration):
    """Determine whether a sustained tracking error constitutes a failure.

    Args:
        tracking_error: RMS joint tracking error in radians.
        duration: Duration of the sustained error in seconds.

    Returns:
        True if the tracking error exceeds the threshold for the required
        window; False otherwise.
    """
    return (
        tracking_error > MAX_TRACKING_ERROR
        and duration >= TRACKING_FAILURE_WINDOW
    )


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------


class TestPlanningRequestSchema(unittest.TestCase):
    """Schema validation tests for PlanningRequest payloads."""

    def test_valid_request_passes_validation(self):
        """A correctly formed PlanningRequest must pass validation."""
        req = _make_planning_request()
        validate_planning_request(req)  # must not raise

    def test_missing_required_field_raises(self):
        """Removing any required field must raise ValueError."""
        required_fields = [
            "request_id",
            "start_joint_positions",
            "goal_joint_positions",
            "joint_count",
            "occupancy_stamp",
            "timeout",
            "planner_config",
            "reference_frame",
        ]
        for field in required_fields:
            with self.subTest(field=field):
                req = _make_planning_request()
                del req[field]
                with self.assertRaises(ValueError):
                    validate_planning_request(req)

    def test_wrong_reference_frame_raises(self):
        """A reference_frame other than 'world' must raise ValueError."""
        req = _make_planning_request(reference_frame="base_link")
        with self.assertRaises(ValueError):
            validate_planning_request(req)

    def test_joint_count_mismatch_raises(self):
        """joint_count not matching actual list length must raise ValueError."""
        req = _make_planning_request()
        req["joint_count"] = ROBOT_DOF + 1  # intentional mismatch
        with self.assertRaises(ValueError):
            validate_planning_request(req)

    def test_negative_timeout_raises(self):
        """A non-positive timeout must raise ValueError."""
        req = _make_planning_request(timeout=-1.0)
        with self.assertRaises(ValueError):
            validate_planning_request(req)

    def test_zero_timeout_raises(self):
        """A zero timeout must raise ValueError."""
        req = _make_planning_request(timeout=0.0)
        with self.assertRaises(ValueError):
            validate_planning_request(req)

    def test_missing_algorithm_in_config_raises(self):
        """planner_config without 'algorithm' key must raise ValueError."""
        req = _make_planning_request()
        req["planner_config"] = {}
        with self.assertRaises(ValueError):
            validate_planning_request(req)

    def test_empty_request_id_raises(self):
        """An empty request_id must raise ValueError."""
        req = _make_planning_request(request_id="")
        with self.assertRaises(ValueError):
            validate_planning_request(req)

    def test_request_id_is_valid_uuid(self):
        """The fixture request_id must be a valid UUID v4."""
        req = _make_planning_request()
        parsed = uuid.UUID(req["request_id"], version=4)
        self.assertEqual(str(parsed), req["request_id"])


class TestPlanningResultSchema(unittest.TestCase):
    """Schema validation tests for PlanningResult payloads."""

    def test_valid_success_result_passes(self):
        """A success result with a valid path must pass validation."""
        result = _make_planning_result(status="success")
        validate_planning_result(result)

    def test_valid_no_plan_found_result_passes(self):
        """A no_plan_found result with null path must pass validation."""
        result = _make_planning_result(
            status="no_plan_found",
            path=None,
            failure_reason="Exhausted budget after 10000 nodes",
        )
        validate_planning_result(result)

    def test_success_requires_non_null_path(self):
        """status='success' with null path must raise ValueError."""
        result = _make_planning_result(status="success")
        result["path"] = None
        with self.assertRaises(ValueError):
            validate_planning_result(result)

    def test_failure_requires_null_path(self):
        """status='no_plan_found' with non-null path must raise ValueError."""
        result = _make_planning_result(status="no_plan_found", path=None)
        result["path"] = [[0.0] * ROBOT_DOF]
        with self.assertRaises(ValueError):
            validate_planning_result(result)

    def test_waypoint_count_must_match_path_length(self):
        """waypoint_count not matching len(path) must raise ValueError."""
        result = _make_planning_result(status="success")
        result["waypoint_count"] = 99  # intentional mismatch
        with self.assertRaises(ValueError):
            validate_planning_result(result)

    def test_failure_waypoint_count_must_be_zero(self):
        """waypoint_count must be 0 for non-success results."""
        result = _make_planning_result(status="no_plan_found", path=None)
        result["waypoint_count"] = 3  # wrong
        with self.assertRaises(ValueError):
            validate_planning_result(result)

    def test_invalid_status_raises(self):
        """An unrecognized status value must raise ValueError."""
        result = _make_planning_result(status="unknown_status")
        with self.assertRaises(ValueError):
            validate_planning_result(result)

    def test_all_valid_statuses_accepted(self):
        """Each valid status value must be accepted without error."""
        for status in VALID_PLANNER_STATUSES:
            with self.subTest(status=status):
                if status == "success":
                    result = _make_planning_result(status=status)
                else:
                    result = _make_planning_result(
                        status=status,
                        path=None,
                        failure_reason="test reason",
                    )
                validate_planning_result(result)

    def test_negative_solve_time_raises(self):
        """A negative solve_time must raise ValueError."""
        result = _make_planning_result(solve_time=-0.1)
        with self.assertRaises(ValueError):
            validate_planning_result(result)

    def test_request_id_echoed_in_result(self):
        """PlanningResult.request_id must echo PlanningRequest.request_id."""
        req = _make_planning_request()
        result = _make_planning_result(request_id=req["request_id"])
        self.assertEqual(req["request_id"], result["request_id"])


class TestOccupancyUpdateSchema(unittest.TestCase):
    """Schema validation tests for OccupancyUpdatePayload."""

    def test_valid_payload_passes(self):
        """A correctly formed OccupancyUpdatePayload must pass validation."""
        payload = _make_occupancy_update()
        validate_occupancy_update(payload)

    def test_empty_point_cloud_passes(self):
        """An empty point list is a valid occupancy update."""
        payload = _make_occupancy_update(points=[])
        validate_occupancy_update(payload)

    def test_wrong_frame_raises(self):
        """A reference_frame other than 'world' must raise ValueError."""
        payload = _make_occupancy_update(reference_frame="sensor_frame")
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_point_count_mismatch_raises(self):
        """point_count not matching len(points) must raise ValueError."""
        payload = _make_occupancy_update()
        payload["point_count"] = 99  # intentional mismatch
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_voxel_size_below_minimum_raises(self):
        """voxel_size below MIN_VOXEL_SIZE must raise ValueError."""
        payload = _make_occupancy_update(voxel_size=MIN_VOXEL_SIZE / 2)
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_voxel_size_above_maximum_raises(self):
        """voxel_size above MAX_VOXEL_SIZE must raise ValueError."""
        payload = _make_occupancy_update(voxel_size=MAX_VOXEL_SIZE + 0.1)
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_voxel_size_at_boundaries_passes(self):
        """voxel_size exactly at MIN and MAX boundaries must pass."""
        for size in [MIN_VOXEL_SIZE, MAX_VOXEL_SIZE]:
            with self.subTest(voxel_size=size):
                payload = _make_occupancy_update(voxel_size=size)
                validate_occupancy_update(payload)

    def test_invalid_source_raises(self):
        """An unrecognized source value must raise ValueError."""
        payload = _make_occupancy_update(source="unknown_sensor")
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_all_valid_sources_accepted(self):
        """Each valid source value must be accepted without error."""
        for source in VALID_OCCUPANCY_SOURCES:
            with self.subTest(source=source):
                payload = _make_occupancy_update(source=source)
                validate_occupancy_update(payload)

    def test_malformed_point_raises(self):
        """A point with wrong dimension must raise ValueError."""
        payload = _make_occupancy_update(points=[[0.1, 0.2]])  # missing Z
        payload["point_count"] = 1
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_point_count_exceeds_maximum_raises(self):
        """point_count above MAX_POINT_COUNT must raise ValueError."""
        payload = _make_occupancy_update()
        payload["point_count"] = MAX_POINT_COUNT + 1
        payload["points"] = [[0.0, 0.0, 0.0]] * (MAX_POINT_COUNT + 1)
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)

    def test_empty_update_id_raises(self):
        """An empty update_id must raise ValueError."""
        payload = _make_occupancy_update(update_id="")
        with self.assertRaises(ValueError):
            validate_occupancy_update(payload)


class TestExecutionFeedbackSchema(unittest.TestCase):
    """Schema validation tests for ExecutionFeedback payloads."""

    def test_valid_executing_feedback_passes(self):
        """A valid 'executing' feedback must pass validation."""
        feedback = _make_execution_feedback(status="executing")
        validate_execution_feedback(feedback)

    def test_valid_succeeded_feedback_passes(self):
        """A valid 'succeeded' feedback must pass validation."""
        feedback = _make_execution_feedback(
            status="succeeded", progress=1.0, message="Trajectory completed."
        )
        validate_execution_feedback(feedback)

    def test_progress_above_one_raises(self):
        """progress > 1.0 must raise ValueError."""
        feedback = _make_execution_feedback(progress=1.1)
        with self.assertRaises(ValueError):
            validate_execution_feedback(feedback)

    def test_negative_progress_raises(self):
        """progress < 0.0 must raise ValueError."""
        feedback = _make_execution_feedback(progress=-0.1)
        with self.assertRaises(ValueError):
            validate_execution_feedback(feedback)

    def test_negative_tracking_error_raises(self):
        """A negative tracking_error must raise ValueError."""
        feedback = _make_execution_feedback(tracking_error=-0.01)
        with self.assertRaises(ValueError):
            validate_execution_feedback(feedback)

    def test_invalid_status_raises(self):
        """An unrecognized status must raise ValueError."""
        feedback = _make_execution_feedback(status="running")
        with self.assertRaises(ValueError):
            validate_execution_feedback(feedback)

    def test_terminal_status_requires_message(self):
        """Non-executing terminal statuses must have a non-null message."""
        for status in ("succeeded", "tracking_failure", "aborted"):
            with self.subTest(status=status):
                feedback = _make_execution_feedback(
                    status=status, progress=1.0, message=None
                )
                with self.assertRaises(ValueError):
                    validate_execution_feedback(feedback)

    def test_all_valid_statuses_accepted(self):
        """Each valid execution status must be accepted without error."""
        for status in VALID_EXECUTION_STATUSES:
            with self.subTest(status=status):
                msg = None if status == "executing" else "test message"
                prog = 1.0 if status in ("succeeded",) else 0.5
                feedback = _make_execution_feedback(
                    status=status, progress=prog, message=msg
                )
                validate_execution_feedback(feedback)


class TestFailureSemantics(unittest.TestCase):
    """Tests for contract-defined failure semantics."""

    def test_no_plan_found_result_structure(self):
        """no_plan_found result must have null path and zero waypoints."""
        result = _make_planning_result(
            status="no_plan_found",
            path=None,
            failure_reason="No collision-free path found",
        )
        validate_planning_result(result)
        self.assertIsNone(result["path"])
        self.assertEqual(result["waypoint_count"], 0)
        self.assertIsNotNone(result["failure_reason"])

    def test_stale_occupancy_detection(self):
        """Occupancy older than MAX_OCCUPANCY_AGE must be detected as stale."""
        now = time.time()
        stale_stamp = now - (MAX_OCCUPANCY_AGE + 0.1)
        self.assertFalse(check_occupancy_freshness(stale_stamp, now))

    def test_fresh_occupancy_passes(self):
        """Occupancy within MAX_OCCUPANCY_AGE must be accepted as fresh."""
        now = time.time()
        fresh_stamp = now - (MAX_OCCUPANCY_AGE - 0.1)
        self.assertTrue(check_occupancy_freshness(fresh_stamp, now))

    def test_occupancy_exactly_at_age_limit_passes(self):
        """Occupancy exactly at MAX_OCCUPANCY_AGE must pass (boundary)."""
        now = time.time()
        boundary_stamp = now - MAX_OCCUPANCY_AGE
        self.assertTrue(check_occupancy_freshness(boundary_stamp, now))

    def test_stale_occupancy_result_structure(self):
        """stale_occupancy failure must use 'invalid_request' status."""
        result = _make_planning_result(
            status="invalid_request",
            path=None,
            failure_reason="stale_occupancy: age=2.5s exceeds limit=2.0s",
        )
        validate_planning_result(result)
        self.assertIn("stale_occupancy", result["failure_reason"])

    def test_tracking_failure_not_triggered_below_threshold(self):
        """Tracking error below MAX_TRACKING_ERROR must not trigger failure."""
        self.assertFalse(
            check_tracking_failure(MAX_TRACKING_ERROR - 0.01, TRACKING_FAILURE_WINDOW)
        )

    def test_tracking_failure_not_triggered_below_window(self):
        """Tracking error above threshold but below window must not fail."""
        self.assertFalse(
            check_tracking_failure(MAX_TRACKING_ERROR + 0.01, TRACKING_FAILURE_WINDOW - 0.1)
        )

    def test_tracking_failure_triggered_above_threshold_and_window(self):
        """Tracking error above threshold for full window must trigger failure."""
        self.assertTrue(
            check_tracking_failure(MAX_TRACKING_ERROR + 0.01, TRACKING_FAILURE_WINDOW)
        )

    def test_transform_unavailable_feedback_structure(self):
        """transform_unavailable must produce 'aborted' feedback with message."""
        feedback = _make_execution_feedback(
            status="aborted",
            progress=0.0,
            message="transform_unavailable: world->sensor_frame",
        )
        validate_execution_feedback(feedback)
        self.assertEqual(feedback["status"], "aborted")
        self.assertIn("transform_unavailable", feedback["message"])

    def test_controller_tracking_failure_feedback_structure(self):
        """Controller tracking failure must produce correct feedback."""
        feedback = _make_execution_feedback(
            status="tracking_failure",
            tracking_error=MAX_TRACKING_ERROR + 0.05,
            message="RMS joint error exceeded threshold for 0.5s",
        )
        validate_execution_feedback(feedback)
        self.assertEqual(feedback["status"], "tracking_failure")
        self.assertGreater(feedback["tracking_error"], MAX_TRACKING_ERROR)


class TestFrameAndUnitConventions(unittest.TestCase):
    """Tests for canonical frame and unit conventions."""

    def test_all_boundary_payloads_use_world_frame(self):
        """All boundary payloads must declare 'world' as reference_frame."""
        req = _make_planning_request()
        result = _make_planning_result(request_id=req["request_id"])
        occ = _make_occupancy_update()

        for payload, name in [(req, "PlanningRequest"), (occ, "OccupancyUpdate")]:
            with self.subTest(payload=name):
                self.assertEqual(payload["reference_frame"], CANONICAL_FRAME)

        self.assertEqual(result["reference_frame"], CANONICAL_FRAME)

    def test_joint_positions_are_floats_in_radians_range(self):
        """Joint positions must be floats; fixture values must be in radian range."""
        req = _make_planning_request()
        for i, pos in enumerate(req["start_joint_positions"]):
            with self.subTest(joint=i):
                self.assertIsInstance(pos, float)
                lo, hi = JOINT_LIMITS[i]
                self.assertGreaterEqual(pos, lo)
                self.assertLessEqual(pos, hi)

    def test_occupancy_points_are_3d_vectors(self):
        """All occupancy points must be 3-element lists."""
        occ = _make_occupancy_update(
            points=[[0.0, 0.1, 0.2], [1.0, 2.0, 3.0], [-0.5, 0.5, 0.0]]
        )
        occ["point_count"] = 3
        validate_occupancy_update(occ)
        for pt in occ["points"]:
            self.assertEqual(len(pt), 3)

    def test_timestamp_is_posix_float(self):
        """Timestamps must be positive POSIX float seconds."""
        occ = _make_occupancy_update()
        self.assertIsInstance(occ["stamp"], float)
        self.assertGreater(occ["stamp"], 0.0)
        # POSIX time for year 2020 is roughly 1.58e9
        self.assertGreater(occ["stamp"], 1.58e9)

    def test_voxel_size_is_in_meters(self):
        """Default voxel_size must be 0.05 m (5 cm) as specified."""
        occ = _make_occupancy_update()
        self.assertAlmostEqual(occ["voxel_size"], 0.05)

    def test_progress_is_dimensionless_fraction(self):
        """progress must be a float in [0.0, 1.0]."""
        for value in [0.0, 0.25, 0.5, 0.75, 1.0]:
            fb = _make_execution_feedback(progress=value)
            validate_execution_feedback(fb)

    def test_joint_count_equals_robot_dof(self):
        """joint_count in request must match ROBOT_DOF constant."""
        req = _make_planning_request()
        self.assertEqual(req["joint_count"], ROBOT_DOF)
        self.assertEqual(len(req["start_joint_positions"]), ROBOT_DOF)
        self.assertEqual(len(req["goal_joint_positions"]), ROBOT_DOF)


class TestEndToEndDryRun(unittest.TestCase):
    """End-to-end dry-run test validating full payload compatibility.

    This test exercises the complete request/update/result/feedback lifecycle
    without any external dependencies, confirming that all payloads are
    schema-compatible and that IDs are correctly echoed across the boundary.
    """

    def test_full_pipeline_dry_run(self):
        """Simulate a complete plan-and-execute cycle with valid payloads."""
        # 1. Scene acquisition produces occupancy update
        scene_stamp = time.time()
        occ = _make_occupancy_update(
            stamp=scene_stamp,
            points=[[0.3, 0.0, 0.5], [0.3, 0.1, 0.5], [0.3, -0.1, 0.5]],
            voxel_size=0.05,
            source="simulated",
        )
        occ["point_count"] = 3
        validate_occupancy_update(occ)

        # 2. FRET builds planning request from current robot state
        req_id = str(uuid.uuid4())
        req = _make_planning_request(
            request_id=req_id,
            start=[0.0, 0.0, 0.0, 0.0],
            goal=[1.0, 0.5, -0.5, 0.3],
            occupancy_stamp=scene_stamp,
            timeout=5.0,
        )
        validate_planning_request(req)

        # Confirm occupancy is fresh at planning time
        self.assertTrue(check_occupancy_freshness(occ["stamp"], time.time()))

        # 3. ARCO returns a successful planning result
        path = [
            [0.0, 0.0, 0.0, 0.0],
            [0.25, 0.125, -0.125, 0.075],
            [0.5, 0.25, -0.25, 0.15],
            [0.75, 0.375, -0.375, 0.225],
            [1.0, 0.5, -0.5, 0.3],
        ]
        result = _make_planning_result(
            request_id=req_id,
            status="success",
            path=path,
            solve_time=0.42,
            node_count=1234,
        )
        validate_planning_result(result)

        # Verify request_id is echoed
        self.assertEqual(result["request_id"], req["request_id"])

        # Verify path waypoints have correct dimensionality
        for waypoint in result["path"]:
            self.assertEqual(len(waypoint), ROBOT_DOF)

        # 4. FRET executes and emits feedback
        feedbacks = [
            _make_execution_feedback(
                request_id=req_id, status="executing", progress=0.0
            ),
            _make_execution_feedback(
                request_id=req_id, status="executing", progress=0.5
            ),
            _make_execution_feedback(
                request_id=req_id,
                status="succeeded",
                progress=1.0,
                current_joint_positions=path[-1],
                tracking_error=0.002,
                message="Trajectory completed successfully.",
            ),
        ]

        prev_progress = -1.0
        for fb in feedbacks:
            validate_execution_feedback(fb)
            self.assertGreaterEqual(fb["progress"], prev_progress)
            self.assertEqual(fb["request_id"], req_id)
            prev_progress = fb["progress"]

        # Final feedback must show success
        self.assertEqual(feedbacks[-1]["status"], "succeeded")
        self.assertAlmostEqual(feedbacks[-1]["progress"], 1.0)

    def test_no_plan_found_pipeline_dry_run(self):
        """Simulate a failed planning cycle producing correct failure payloads."""
        scene_stamp = time.time()
        occ = _make_occupancy_update(stamp=scene_stamp)
        validate_occupancy_update(occ)

        req_id = str(uuid.uuid4())
        req = _make_planning_request(
            request_id=req_id,
            occupancy_stamp=scene_stamp,
        )
        validate_planning_request(req)

        # ARCO cannot find a plan
        result = _make_planning_result(
            request_id=req_id,
            status="no_plan_found",
            path=None,
            solve_time=5.0,
            node_count=50000,
            failure_reason="Budget exhausted; no collision-free path exists.",
        )
        validate_planning_result(result)
        self.assertIsNone(result["path"])

        # FRET emits abort feedback
        feedback = _make_execution_feedback(
            request_id=req_id,
            status="aborted",
            progress=0.0,
            message="Planning failed: no_plan_found",
        )
        validate_execution_feedback(feedback)
        self.assertEqual(feedback["status"], "aborted")
        self.assertEqual(feedback["request_id"], req_id)

    def test_stale_occupancy_pipeline_dry_run(self):
        """Simulate a planning rejection due to stale occupancy."""
        stale_stamp = time.time() - (MAX_OCCUPANCY_AGE + 1.0)

        req_id = str(uuid.uuid4())
        req = _make_planning_request(
            request_id=req_id,
            occupancy_stamp=stale_stamp,
        )
        # The request itself is structurally valid
        validate_planning_request(req)

        # But the occupancy is stale
        self.assertFalse(check_occupancy_freshness(stale_stamp, time.time()))

        # ARCO rejects with invalid_request + stale_occupancy reason
        result = _make_planning_result(
            request_id=req_id,
            status="invalid_request",
            path=None,
            solve_time=0.001,
            node_count=0,
            failure_reason="stale_occupancy: age=3.0s exceeds limit=2.0s",
        )
        validate_planning_result(result)
        self.assertIn("stale_occupancy", result["failure_reason"])


if __name__ == "__main__":
    unittest.main()
