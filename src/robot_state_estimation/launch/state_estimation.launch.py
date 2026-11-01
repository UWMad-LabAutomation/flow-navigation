# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Fuse wheel velocities and a D435i gyro without launching Gazebo or a driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the sole publisher of filtered odometry and odom-to-base TF."""
    share = get_package_share_directory('robot_state_estimation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/camera/imu'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('ekf_params', default_value=os.path.join(share, 'config/ekf.yaml')),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[LaunchConfiguration('ekf_params'), {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'odom0': LaunchConfiguration('wheel_odom_topic'),
                'imu0': LaunchConfiguration('imu_topic'),
                'base_link_frame': LaunchConfiguration('base_frame'),
            }],
            remappings=[('odometry/filtered', '/odom')], output='screen',
        ),
    ])
