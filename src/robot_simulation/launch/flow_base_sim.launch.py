#!/usr/bin/env python3
"""Launch the FLOW base Gazebo simulation and ROS bridges."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo, the ROS-Gazebo bridges, and the static robot frames."""
    simulation_share = get_package_share_directory('robot_simulation')
    description_share = get_package_share_directory('robot_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(simulation_share, 'worlds', 'flow_base.sdf')
    world_file = LaunchConfiguration('world')
    description_file = os.path.join(
        description_share, 'urdf', 'flow_base.urdf.xacro')
    robot_description = Command(['xacro ', description_file])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/flow_base/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/flow_base/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/flow_base/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/flow_base/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/flow_base/cmd_vel', '/cmd_vel'),
            ('/model/flow_base/odometry', '/odom'),
            ('/model/flow_base/tf', '/tf'),
            ('/model/flow_base/scan', '/scan'),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # Gazebo scopes a sensor's LaserScan frame as model/link/sensor. Keep that
    # frame connected to the URDF lidar frame used by Nav2 and RViz.
    laser_frame_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'laser_link', 'flow_base/laser_link/laser'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='SDF world file to load in Gazebo.',
        ),
        gz_sim,
        bridge,
        robot_state_publisher,
        laser_frame_transform,
    ])
