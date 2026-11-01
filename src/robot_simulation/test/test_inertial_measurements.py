# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check simulation covariance and prevent absolute simulator attitude fusion."""

from nav_msgs.msg import Odometry
from robot_simulation.inertial_measurements import prepare_imu, prepare_wheel
from sensor_msgs.msg import Imu


def test_simulator_attitude_is_unavailable():
    """Simulated IMU must behave like raw gyro/accel rather than a compass."""
    raw = Imu()
    raw.orientation.z = 0.4
    raw.angular_velocity.y = 0.12
    prepared = prepare_imu(raw)
    assert prepared.orientation_covariance[0] == -1
    assert prepared.orientation.z == 0
    assert prepared.angular_velocity.y == 0.12
    assert prepared.angular_velocity_covariance[8] == 1e-6


def test_existing_covariance_is_preserved():
    """Never replace supplied measurement variance with a simulation fallback."""
    raw = Imu()
    raw.angular_velocity_covariance[8] = 0.01
    assert prepare_imu(raw).angular_velocity_covariance[8] == 0.01
    wheel = Odometry()
    wheel.twist.covariance[35] = 0.02
    assert prepare_wheel(wheel).twist.covariance[35] == 0.02


def test_wheel_lateral_motion_is_preserved():
    """Holonomic lateral velocity must remain a measured EKF input."""
    wheel = Odometry()
    wheel.twist.twist.linear.y = 0.5
    prepared = prepare_wheel(wheel)
    assert prepared.twist.twist.linear.y == 0.5
    assert prepared.twist.covariance[7] > 0
    assert prepared.twist.covariance[35] > prepared.twist.covariance[7]
