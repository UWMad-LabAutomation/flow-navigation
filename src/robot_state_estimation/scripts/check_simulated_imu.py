#!/usr/bin/env python3
# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check gyro axes and filtered odometry during isolated holonomic simulation."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def seconds(stamp):
    """Convert a ROS timestamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    """Run the default simulated IMU/EKF and verify measured motion."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=78)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GZ_PARTITION'] = 'flow_imu_check_' + uuid.uuid4().hex
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    log = (args.output / 'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'robot_simulation', 'flow_base_sim.launch.py',
        'gui:=true', 'camera_fps:=10', 'use_imu:=true',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('simulated_imu_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    imu, wheel, filtered, truth = [], [], [], []
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GazeboNode
    gazebo = GazeboNode()

    def ground_truth(message):
        for p in message.pose:
            if p.name == 'flow_base':
                q = p.orientation
                heading = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                truth.append([message.header.stamp.sec + message.header.stamp.nsec * 1e-9,
                              p.position.x, p.position.y, heading])

    gazebo.subscribe(Pose_V, '/world/flow_base_world/pose/info', ground_truth)
    subscriptions = [
        node.create_subscription(Imu, '/camera/imu', imu.append, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/wheel/odometry', wheel.append,
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', filtered.append, qos_profile_sensor_data),
    ]
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)

    def until(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(node, timeout_sec=0.02)
            assert process.poll() is None, 'Simulation exited'
        assert predicate(), 'Timed out waiting for simulated measurements'

    try:
        until(lambda: len(imu) > 20 and len(wheel) > 10 and len(filtered) > 10
              and len(truth) > 10, 30)
        until(lambda: buffer.can_transform(
            'base_footprint', 'camera_imu_frame', rclpy.time.Time()), 10)
        start = seconds(filtered[-1].header.stamp)
        maneuvers = [('forward', 0.3, 0.0, 0.0, 5),
                     ('left_curve', 0.25, 0.0, 0.3, 6),
                     ('sidestep_curve', 0.2, 0.15, -0.25, 6),
                     ('forward_again', 0.3, 0.0, 0.0, 5),
                     ('stop', 0.0, 0.0, 0.0, 2)]
        for name, vx, vy, wz, duration in maneuvers:
            segment_start = seconds(filtered[-1].header.stamp)
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = vx, vy, wz
            end = time.monotonic() + 45
            while (seconds(filtered[-1].header.stamp) - segment_start < duration
                   and time.monotonic() < end):
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.02)
            assert seconds(filtered[-1].header.stamp) - segment_start >= duration
            print('MANEUVER_DONE', name, flush=True)
        publisher.publish(Twist())
        q = buffer.lookup_transform(
            'base_footprint', 'camera_imu_frame', rclpy.time.Time()).transform.rotation
        # Third row of R(base <- sensor), including the actual camera pitch.
        row = (2 * (q.x * q.z - q.w * q.y),
               2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y))
        reference = np.array(list({p[0]: p for p in truth}.values()))
        reference = reference[np.argsort(reference[:, 0])]
        reference[:, 3] = np.unwrap(reference[:, 3])
        estimated = np.array([[seconds(m.header.stamp), m.pose.pose.position.x,
                               m.pose.pose.position.y, math.atan2(
                                   2 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z
                                        + m.pose.pose.orientation.x * m.pose.pose.orientation.y),
                                   1 - 2 * (m.pose.pose.orientation.y ** 2
                                            + m.pose.pose.orientation.z ** 2))]
                              for m in filtered if seconds(m.header.stamp) >= start])
        estimated = estimated[(estimated[:, 0] >= reference[0, 0])
                              & (estimated[:, 0] <= reference[-1, 0])]
        actual = np.column_stack([np.interp(estimated[:, 0], reference[:, 0], reference[:, i])
                                  for i in (1, 2, 3)])
        position_error = np.linalg.norm(estimated[:, 1:3] - actual[:, :2], axis=1)
        heading_error = np.arctan2(np.sin(estimated[:, 3] - actual[:, 2]),
                                   np.cos(estimated[:, 3] - actual[:, 2]))
        moving = [m for m in imu if start <= seconds(m.header.stamp) <= reference[-1, 0]]
        times = np.array([seconds(m.header.stamp) for m in moving])
        projected = np.array([row[0] * m.angular_velocity.x + row[1] * m.angular_velocity.y
                              + row[2] * m.angular_velocity.z for m in moving])
        true_rate = np.gradient(reference[:, 3], reference[:, 0])
        rate_error = projected - np.interp(times, reference[:, 0], true_rate)
        integrated = np.concatenate(([0], np.cumsum(
            (projected[:-1] + projected[1:]) * 0.5 * np.diff(times))))
        gyro_heading_error = integrated - (np.interp(times, reference[:, 0], reference[:, 3])
                                           - np.interp(times[0], reference[:, 0], reference[:, 3]))
        accuracy = {
            'duration_sim_s': float(estimated[-1, 0] - estimated[0, 0]),
            'gyro_rate_rmse_rad_s': float(np.sqrt(np.mean(rate_error ** 2))),
            'integrated_gyro_heading_max_error_deg': float(np.degrees(
                np.max(np.abs(gyro_heading_error)))),
            'ekf_position_rmse_m': float(np.sqrt(np.mean(position_error ** 2))),
            'ekf_position_max_error_m': float(np.max(position_error)),
            'ekf_heading_rmse_deg': float(np.degrees(np.sqrt(np.mean(heading_error ** 2)))),
            'ekf_heading_max_error_deg': float(np.degrees(np.max(np.abs(heading_error)))),
            'comparison': 'Gazebo truth interpolated to each sensor/EKF simulation timestamp',
        }
        assert accuracy['gyro_rate_rmse_rad_s'] < 0.01, accuracy
        assert accuracy['ekf_position_max_error_m'] < 0.1, accuracy
        assert accuracy['ekf_heading_max_error_deg'] < 2.0, accuracy
        owners = [p.node_name for p in node.get_publishers_info_by_topic('/odom')]
        assert owners == ['ekf_filter_node'], owners
        tf_nodes = sorted({p.node_name for p in node.get_publishers_info_by_topic('/tf')})
        assert set(tf_nodes) <= {'ekf_filter_node', 'robot_state_publisher'}, tf_nodes
        assert 'ekf_filter_node' in tf_nodes
        assert buffer.can_transform('odom', 'base_footprint', rclpy.time.Time())
        assert imu[-1].orientation_covariance[0] == -1
        rates = {name: (len(messages) - 1) / (seconds(messages[-1].header.stamp)
                 - seconds(messages[0].header.stamp)) for name, messages in
                 [('imu', imu), ('wheel', wheel), ('filtered', filtered)]}
        result = {
            'rates_sim_hz': rates, 'tf_publisher_nodes': tf_nodes,
            'odom_publishers': owners, 'imu_frame': imu[-1].header.frame_id,
            'accuracy': accuracy,
            'imu_orientation_available': False,
        }
        np.savez_compressed(args.output / 'accuracy_samples.npz', reference=reference,
                            estimated=estimated, position_error=position_error,
                            heading_error=heading_error, imu_times=times,
                            gyro_rate_error=rate_error, gyro_heading_error=gyro_heading_error)
        (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
        print('PASS', json.dumps(result), flush=True)
    finally:
        publisher.publish(Twist())
        del subscriptions, listener
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
