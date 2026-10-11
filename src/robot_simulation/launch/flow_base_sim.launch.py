#!/usr/bin/env python3
"""Launch the FLOW base Gazebo simulation and ROS bridges."""

import math
import os
from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_simulation.sensor_configuration import (
    CAMERA_POSE_ARGUMENTS, configure_sensor_world, DEPTH_TOPIC, INFO_TOPIC,
    POINTS_TOPIC, sensor_launch_arguments,
)
import xacro


def _launch_simulation(context):
    """Apply the selected sensor before starting Gazebo and its matching bridges."""
    description_share = get_package_share_directory('robot_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    sensor = LaunchConfiguration('sensor').perform(context)
    world_file = LaunchConfiguration('world').perform(context)
    pose = {name: LaunchConfiguration(name).perform(context)
            for name in CAMERA_POSE_ARGUMENTS}
    if float(pose['camera_ground_distance']) <= 0:
        raise ValueError('camera_ground_distance must be positive')
    image_width = int(pose['camera_image_width'])
    image_height = int(pose['camera_image_height'])
    if image_width <= 0 or image_height <= 0:
        raise ValueError('camera image dimensions must be positive integers')
    description_file = os.path.join(
        description_share, 'urdf', 'flow_base.urdf.xacro')
    robot_description = xacro.process_file(
        description_file, mappings=pose).toxml()
    configured_world = configure_sensor_world(
        world_file, robot_description, sensor,
        horizontal_fov=float(LaunchConfiguration('camera_hfov').perform(context)),
        near=float(LaunchConfiguration('camera_near').perform(context)),
        far=float(LaunchConfiguration('camera_far').perform(context)),
        image_width=image_width,
        image_height=image_height,
    )
    gui = LaunchConfiguration('gui').perform(context).lower()
    if gui not in ('true', 'false'):
        raise ValueError('gui must be true or false')
    gz_args = '-r ' if gui == 'true' else '-r -s --headless-rendering '

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args + shlex.quote(str(configured_world)),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/model/flow_base/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/model/flow_base/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/flow_base/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]
    remappings = [
        ('/model/flow_base/cmd_vel', '/cmd_vel'),
        ('/model/flow_base/odometry', '/odom'),
        ('/model/flow_base/tf', '/tf'),
    ]
    if sensor == 'camera':
        bridge_arguments.extend([
            DEPTH_TOPIC + '@sensor_msgs/msg/Image[gz.msgs.Image',
            INFO_TOPIC + '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ])
        remappings.extend([
            (DEPTH_TOPIC, '/camera/depth/image_raw'),
            (INFO_TOPIC, '/camera/depth/camera_info'),
        ])
    else:
        bridge_arguments.append(
            '/model/flow_base/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
        remappings.append(('/model/flow_base/scan', '/scan'))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_arguments,
        remappings=remappings,
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

    actions = [
        LogInfo(msg=f'FLOW sensor={sensor}; generated world={configured_world}'),
        gz_sim, bridge, robot_state_publisher,
    ]
    if sensor == 'camera':
        # Harmonic's depth-camera cloud contains X-forward/Y-left/Z-up points,
        # even when optical_frame_id labels its messages. Correct only the
        # cloud header; the images and CameraInfo correctly use optical axes.
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_points_bridge',
            parameters=[{
                'override_frame_id': 'camera_depth_frame',
                'bridge_names': ['depth_points'],
                'bridges.depth_points.ros_topic_name': '/camera/depth/points',
                'bridges.depth_points.gz_topic_name': POINTS_TOPIC,
                'bridges.depth_points.ros_type_name': 'sensor_msgs/msg/PointCloud2',
                'bridges.depth_points.gz_type_name': 'gz.msgs.PointCloudPacked',
                'bridges.depth_points.direction': 'GZ_TO_ROS',
                # Prefer fresh clouds over delivery of queued old frames.
                'bridges.depth_points.qos_profile': 'SENSOR_DATA',
                'bridges.depth_points.publisher_queue': 1,
            }],
            output='screen',
        ))
        half_fov = math.radians(float(pose['camera_hfov'])) / 2.0
        actions.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='camera_to_scan',
            remappings=[('cloud_in', '/camera/depth/points'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': True,
                'target_frame': 'camera_scan_frame',
                'transform_tolerance': 0.1,
                'queue_size': 1,
                # Heights relative to the level scan frame at Z=0.22 m:
                # keep obstacles 0.10 to 1.00 m above ground, reject the floor.
                'min_height': -0.12,
                'max_height': 0.78,
                'angle_min': -half_fov,
                # Small padding keeps a point exactly on the last bin edge
                # inside the converter's allocated ranges array.
                'angle_max': half_fov + 1.0e-5,
                'angle_increment': math.radians(0.5),
                'scan_time': 0.1,
                'range_min': 0.15,
                'range_max': float(LaunchConfiguration('camera_far').perform(context)),
                # No depth return is unknown, not evidence of clear space.
                'use_inf': False,
                'inf_epsilon': 1.0,
            }],
            output='screen',
        ))
    if sensor == 'lidar':
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'laser_link', 'flow_base/laser_link/laser'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
    # Preserve lookup of assets next to the input world after making a copy.
    actions.insert(0, AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(Path(world_file).resolve().parent)))
    return actions


def generate_launch_description():
    """Declare FLOW simulation options, including the sensing source."""
    simulation_share = get_package_share_directory('robot_simulation')
    return LaunchDescription(sensor_launch_arguments() + [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(simulation_share, 'worlds', 'flow_base.sdf'),
            description='SDF world file to load in Gazebo.',
        ),
        OpaqueFunction(function=_launch_simulation),
    ])
