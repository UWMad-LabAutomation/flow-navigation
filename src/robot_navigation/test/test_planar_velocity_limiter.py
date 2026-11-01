# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check physical speed limits across directions and malformed commands."""

import math

import pytest

from robot_navigation.planar_velocity_limiter import limit_velocity


@pytest.mark.parametrize('angle', [i * math.pi / 12 for i in range(24)])
def test_total_speed_and_direction(angle):
    """Allow the same speed in every direction without faster diagonals."""
    for speed in [0.0, 0.1, 0.63, 0.9, 2.0]:
        vx, vy, wz = limit_velocity(
            speed * math.cos(angle), speed * math.sin(angle), 0.4, 0.63, 0.8)
        assert math.hypot(vx, vy) == pytest.approx(min(speed, 0.63))
        assert vx * math.sin(angle) - vy * math.cos(angle) == pytest.approx(0)
        assert wz == 0.4


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', range(3))
def test_nonfinite_command_stops(value, axis):
    """Never forward NaN or infinity into the collision monitor or base."""
    command = [0.2, 0.2, 0.2]
    command[axis] = value
    assert limit_velocity(*command, 0.63, 0.8) == (0, 0, 0)


def test_yaw_limit():
    """Limit yaw independently of the translation direction."""
    assert limit_velocity(0, 0, 2.0, 0.63, 0.8) == (0, 0, 0.8)
    assert limit_velocity(0, 0, -2.0, 0.63, 0.8) == (0, 0, -0.8)
