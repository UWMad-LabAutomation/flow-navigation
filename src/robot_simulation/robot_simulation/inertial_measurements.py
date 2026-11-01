# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Supply explicit simulation measurement uncertainty to wheel/IMU fusion."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def prepare_imu(message):
    """Expose gyro/acceleration noise without exporting Gazebo ground-truth attitude."""
    message.orientation.x = message.orientation.y = message.orientation.z = 0.0
    message.orientation.w = 1.0
    message.orientation_covariance = [-1.0] + [0.0] * 8
    if not any(message.angular_velocity_covariance):
        message.angular_velocity_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]
    if not any(message.linear_acceleration_covariance):
        message.linear_acceleration_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
    return message


def prepare_wheel(message):
    """Give raw simulated wheel velocities finite, nonzero uncertainty."""
    if not any(message.twist.covariance):
        for index, variance in [(0, 2.5e-5), (7, 2.5e-5), (14, 1.0),
                                (21, 1.0), (28, 1.0), (35, 4e-4)]:
            message.twist.covariance[index] = variance
    return message


class InertialMeasurements(Node):
    """Republish simulated measurements with uncertainty and no absolute attitude."""

    def __init__(self):
        """Subscribe to raw bridges and supply EKF input topics."""
        super().__init__('inertial_measurements')
        self.imu_pub = self.create_publisher(Imu, '/camera/imu', 10)
        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/camera/imu/raw', self.on_imu, qos_profile_sensor_data)
        self.wheel_sub = self.create_subscription(
            Odometry, '/wheel/odometry/raw', self.on_wheel, qos_profile_sensor_data)

    def on_imu(self, message):
        """Forward gyro samples with the simulated noise covariance."""
        self.imu_pub.publish(prepare_imu(message))

    def on_wheel(self, message):
        """Forward wheel measurements with simulation uncertainty."""
        self.wheel_pub.publish(prepare_wheel(message))


def main(args=None):
    """Run simulation measurement preparation."""
    rclpy.init(args=args)
    node = InertialMeasurements()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
