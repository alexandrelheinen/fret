"""Tests for Dubins vehicle state handling against compiled ARCO."""

from __future__ import annotations

import numpy as np
from arco.guidance.vehicle import DubinsVehicle

from fret.scenario.dubins_race_runner import _place_vehicle, _sync_vehicle_pose


def test_place_vehicle_moves_the_pose() -> None:
    vehicle = DubinsVehicle()
    _place_vehicle(vehicle, 1.5, -2.0, 0.75)
    assert vehicle.pose == (1.5, -2.0, 0.75)


def test_place_vehicle_keeps_the_filtered_command_state() -> None:
    """The physics sync rewrites the pose every tick and must not brake.

    ARCO v0.5.0 makes the pose read-only and offers ``reset``, which zeroes
    the speed and turn-rate filters. Losing them each tick would hand the
    path-following MPC a standing vehicle it never commanded.
    """
    vehicle = DubinsVehicle()
    vehicle.speed = 0.36
    vehicle.turn_rate = -0.2

    _place_vehicle(vehicle, 0.4, 0.9, -1.1)

    assert vehicle.pose == (0.4, 0.9, -1.1)
    assert vehicle.speed == 0.36
    assert vehicle.turn_rate == -0.2


def test_sync_vehicle_pose_copies_a_simulated_pose() -> None:
    vehicle = DubinsVehicle()
    vehicle.speed = 0.25
    pose = np.array([2.0, 3.0, 0.5], dtype=np.float64)

    _sync_vehicle_pose(vehicle, pose)

    assert vehicle.pose == (2.0, 3.0, 0.5)
    assert vehicle.speed == 0.25
