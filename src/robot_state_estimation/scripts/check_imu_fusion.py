#!/usr/bin/env python3
# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Verify pitched-camera gyro fusion despite deliberately biased wheel yaw rate."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster


def main():
    """Run isolated synthetic sensor inputs against the actual EKF launch."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=76)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    log = (args.output / 'ekf.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'robot_state_estimation', 'state_estimation.launch.py',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('imu_fusion_check')
    wheel_pub = node.create_publisher(Odometry, '/wheel/odometry', 10)
    imu_pub = node.create_publisher(Imu, '/camera/imu', 10)
    output = []
    subscription = node.create_subscription(Odometry, '/odom', output.append, 100)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = 'camera_imu_frame'
    transform.transform.translation.z = 2.0
    pitch = math.radians(26.48)
    transform.transform.rotation.y = math.sin(pitch / 2)
    transform.transform.rotation.w = math.cos(pitch / 2)
    broadcaster.sendTransform(transform)

    def publish():
        stamp = node.get_clock().now().to_msg()
        wheel = Odometry()
        wheel.header.stamp = stamp
        wheel.header.frame_id = 'odom'
        wheel.child_frame_id = 'base_footprint'
        wheel.pose.pose.orientation.w = 1.0
        wheel.pose.pose.position.x = 100.0  # Must not be fused as position.
        wheel.twist.twist.linear.y = 0.2
        wheel.twist.twist.angular.z = 0.3  # Deliberately 0.1 rad/s too high.
        wheel.twist.covariance[0] = wheel.twist.covariance[7] = 2.5e-5
        wheel.twist.covariance[35] = 4e-4
        wheel_pub.publish(wheel)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'camera_imu_frame'
        imu.orientation_covariance[0] = -1.0
        # Sensor-frame components of a true base yaw rate of 0.2 rad/s.
        imu.angular_velocity.x = -math.sin(pitch) * 0.2
        imu.angular_velocity.z = math.cos(pitch) * 0.2
        for index in (0, 4, 8):
            imu.angular_velocity_covariance[index] = 1e-6
        imu.linear_acceleration.x = 5.0  # Elevator/gravity acceleration ignored.
        imu_pub.publish(imu)

    timer = node.create_timer(0.02, publish)
    try:
        end = time.monotonic() + 12
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            assert process.poll() is None, 'EKF exited; inspect ekf.log'
        assert len(output) > 100, 'Missing filtered odometry'
        samples = output[-100:]
        gyro_error = max(abs(m.twist.twist.angular.z - 0.2) for m in samples)
        lateral_error = max(abs(m.twist.twist.linear.y - 0.2) for m in samples)
        assert gyro_error < 0.01, gyro_error
        assert lateral_error < 0.01, lateral_error
        assert abs(samples[-1].pose.pose.position.x) < 10, 'Wheel position was double-counted'
        assert abs(samples[-1].pose.pose.position.z) < 1e-6
        assert abs(samples[-1].twist.twist.linear.x) < 0.01, 'Acceleration was fused'
        result = {
            'wheel_yaw_rate': 0.3, 'true_imu_yaw_rate': 0.2,
            'filtered_yaw_rate': samples[-1].twist.twist.angular.z,
            'max_steady_yaw_rate_error': gyro_error,
            'filtered_lateral_velocity': samples[-1].twist.twist.linear.y,
            'camera_pitch_degrees': 26.48, 'filtered_messages': len(output),
            'orientation_and_acceleration_fused': False,
        }
        (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
        print('PASS', json.dumps(result), flush=True)
    finally:
        del timer, subscription
        node.destroy_node()
        rclpy.shutdown()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if process.poll() is not None:
                break
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        log.close()


if __name__ == '__main__':
    main()
