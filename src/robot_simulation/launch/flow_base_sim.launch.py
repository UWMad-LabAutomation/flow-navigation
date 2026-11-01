#!/usr/bin/env python3
"""Launch the FLOW base Gazebo simulation and ROS bridges."""

import math
import os
from pathlib import Path
import shlex
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_simulation.graphics_configuration import graphics_environment
from robot_simulation.sensor_configuration import (
    CAMERA_POSE_ARGUMENTS, configure_sensor_world, DEPTH_TOPIC, IMU_TOPIC,
    INFO_TOPIC, POINTS_TOPIC, sensor_launch_arguments,
)
import xacro


def _launch_simulation(context):
    """Apply the selected sensor before starting Gazebo and its matching bridges."""
    description_share = get_package_share_directory('robot_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    sensor = LaunchConfiguration('sensor').perform(context)
    camera_backend = LaunchConfiguration('camera_backend').perform(context)
    use_imu_text = LaunchConfiguration('use_imu').perform(context).lower()
    if use_imu_text not in ('true', 'false'):
        raise ValueError('use_imu must be true or false')
    use_imu = use_imu_text == 'true'
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
        vertical_fov=float(LaunchConfiguration('camera_vfov').perform(context)),
        near=float(LaunchConfiguration('camera_near').perform(context)),
        far=float(LaunchConfiguration('camera_far').perform(context)),
        image_width=image_width,
        image_height=image_height,
        fps=float(LaunchConfiguration('camera_fps').perform(context)),
        camera_backend=camera_backend,
        ray_width=int(LaunchConfiguration('camera_ray_width').perform(context)),
        ray_height=int(LaunchConfiguration('camera_ray_height').perform(context)),
        imu_enabled=use_imu,
    )
    gui = LaunchConfiguration('gui').perform(context).lower()
    if gui not in ('true', 'false'):
        raise ValueError('gui must be true or false')
    gz_args = '-r ' if gui == 'true' else '-r -s '
    if gui == 'false' and (sensor == 'lidar' or camera_backend == 'render'):
        gz_args += '--headless-rendering '
    # Bind time to this world, even when another Gazebo process owns /clock.
    world_name = ET.parse(configured_world).find('world').get('name')
    world_clock = f'/world/{world_name}/clock'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args + shlex.quote(str(configured_world)),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge_arguments = [
        world_clock + '@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/model/flow_base/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/model/flow_base/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    ]
    remappings = [
        (world_clock, '/clock'),
        ('/model/flow_base/cmd_vel', '/cmd_vel'),
        ('/model/flow_base/odometry', '/wheel/odometry/raw' if use_imu else '/odom'),
    ]
    if not use_imu:
        bridge_arguments.append('/model/flow_base/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
        remappings.append(('/model/flow_base/tf', '/tf'))
    if sensor == 'camera' and camera_backend == 'render':
        bridge_arguments.extend([
            DEPTH_TOPIC + '@sensor_msgs/msg/Image[gz.msgs.Image',
            INFO_TOPIC + '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ])
        remappings.extend([
            (DEPTH_TOPIC, '/camera/depth/image_raw'),
            (INFO_TOPIC, '/camera/depth/camera_info'),
        ])
    elif sensor == 'lidar':
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
        # Keep leftover Gazebo worlds from sharing camera and command topics.
        # Respect an explicit partition so external Gazebo tools can join it.
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get(
            'GZ_PARTITION') or f'flow_{uuid.uuid4().hex}'),
        LogInfo(msg=f'FLOW sensor={sensor}; camera_backend={camera_backend}; '
                    f'generated world={configured_world}'),
        gz_sim, bridge, robot_state_publisher,
    ]
    graphics = graphics_environment()
    if gui == 'true' and graphics:
        actions[0:0] = [SetEnvironmentVariable(name, value)
                        for name, value in graphics.items()] + [
                            LogInfo(msg='FLOW WSLg GUI graphics: Mesa D3D12 GPU rendering')]
    if use_imu:
        actions.extend([
            Node(
                package='ros_gz_bridge', executable='parameter_bridge', name='imu_bridge',
                arguments=[IMU_TOPIC + '@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                remappings=[(IMU_TOPIC, '/camera/imu/raw')],
                parameters=[{'override_frame_id': 'camera_imu_frame'}], output='screen'),
            Node(
                package='robot_simulation', executable='inertial_measurements',
                parameters=[{'use_sim_time': True}], output='screen'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('robot_state_estimation'),
                    'launch', 'state_estimation.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()),
        ])
    if sensor == 'camera' and camera_backend == 'cpu':
        plugin_lib = Path(get_package_prefix('robot_cpu_camera')) / 'lib'
        if not (plugin_lib / 'libflow_cpu_camera.so').is_file():
            raise RuntimeError('Build robot_cpu_camera with colcon before camera launch')
        actions.insert(0, AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', str(plugin_lib)))
    if sensor == 'camera' and camera_backend == 'render':
        renderer = Path(get_package_prefix('robot_simulation')) / 'opt/camera_renderer'
        renderer_lib = renderer / 'lib/gz-rendering-8/engine-plugins'
        marker = renderer / 'flow_projection_fix.txt'
        if not (marker.is_file()
                and marker.read_text().startswith('gz-rendering8_8.2.3\npatch_sha256=')
                and (renderer_lib / 'libgz-rendering-ogre2.so').is_file()):
            raise RuntimeError(
                'Build the local D435i projection fix before camera launch: '
                'python3 src/robot_simulation/scripts/build_camera_renderer.py')
        # Scope the corrected Ogre2 plugin to this launch; leave system Gazebo intact.
        plugin_paths = [str(renderer_lib)]
        if os.environ.get('GZ_RENDERING_PLUGIN_PATH'):
            plugin_paths.append(os.environ['GZ_RENDERING_PLUGIN_PATH'])
        actions.insert(0, SetEnvironmentVariable(
            'GZ_RENDERING_PLUGIN_PATH', os.pathsep.join(plugin_paths)))
        library_paths = [str(renderer_lib), str(renderer / 'lib')]
        if os.environ.get('LD_LIBRARY_PATH'):
            library_paths.append(os.environ['LD_LIBRARY_PATH'])
        actions.insert(1, SetEnvironmentVariable(
            'LD_LIBRARY_PATH', os.pathsep.join(library_paths)))
        actions.insert(2, LogInfo(msg=f'FLOW camera renderer={renderer_lib}'))
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
                'scan_time': 1.0 / float(LaunchConfiguration('camera_fps').perform(context)),
                'range_min': float(LaunchConfiguration('camera_near').perform(context)),
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
